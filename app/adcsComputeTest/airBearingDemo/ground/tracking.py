#!/usr/bin/env python3
"""
ISS tracker -> SPID MD-03 (ROT2) over USB.

What it does:
- Downloads the current ISS (ZARYA) TLE from CelesTrak.
- Computes Az/El and angular rates at your site using Skyfield/SGP4.
- Applies min-elevation, deadband, and slew-rate limiting.
- Sends smooth setpoints to the MD-03 at a fixed update rate.

Prereqs:
  pip install pyserial skyfield sgp4 numpy requests pytz
"""

import time
import math
import logging
from datetime import datetime, timedelta, timezone
import requests
import numpy as np

from skyfield.api import Loader, wgs84, EarthSatellite
from md03_rot2 import MD03Rot2  # <-- our driver from earlier

# ------------------- CONFIG -------------------
# Your ground station (degrees, meters)
OBS_LAT_DEG = 37.105327   # Utah Tech area (edit to your site)
OBS_LON_DEG = -113.566677
OBS_ALT_M   = 770.0

# Safety / control
MIN_ELEV_DEG      = 5.0     # don't track below this
UPDATE_HZ         = 5.0     # control loop rate (Hz)
DEADBAND_DEG      = 0.3     # don't command if |error| < deadband
MAX_SLEW_DEG_S_AZ = 12.0    # cap commanded azimuth slew rate (deg/s)
MAX_SLEW_DEG_S_EL = 8.0     # cap commanded elevation slew rate (deg/s)
LEAD_TIME_S       = 0.25    # small predictive lead (s), combats latency

# Serial port (None = auto-detect)
SERIAL_PORT = None
BAUD        = 9600

# TLE source (CelesTrak ISS, NORAD 25544)
TLE_URL = "https://celestrak.org/NORAD/Elements/gp.php?CATNR=25544&FORMAT=tle"  # ISS (ZARYA)
# ------------------------------------------------

log = logging.getLogger("ISS-Tracker")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

# Cache dir for Skyfield
load = Loader("/tmp/.skyfield-cache")
ts = load.timescale()

def fetch_iss_tle():
    """Return (name, line1, line2) for ISS."""
    try:
        r = requests.get(TLE_URL, timeout=5)
        r.raise_for_status()
        lines = [ln.strip() for ln in r.text.splitlines() if ln.strip()]
        # Expect exactly 3 lines: name + 2 lines
        if len(lines) >= 3:
            # Use the last 3 (CelesTrak sometimes returns two recent entries)
            name = lines[-3]
            line1 = lines[-2]
            line2 = lines[-1]
            if line1.startswith("1 ") and line2.startswith("2 "):
                log.info(f"Fetched TLE epoch fragment: {line1[18:32]}")
                return name, line1, line2
        raise RuntimeError("Unexpected TLE format from CelesTrak")
    except Exception as e:
        raise RuntimeError(f"Failed to fetch ISS TLE: {e}")

def build_satellite():
    name, l1, l2 = fetch_iss_tle()
    sat = EarthSatellite(l1, l2, name, ts)
    log.info(f"Using TLE for: {name}")
    return sat

def topocentric_azel_and_rates(satellite, observer, t0, dt=0.4):
    """
    Returns (az_deg, el_deg, az_rate_deg_s, el_rate_deg_s, range_km) at UTC time t0.
    Rates via finite difference across dt seconds.
    """
    t  = ts.from_datetime(t0)
    tp = ts.from_datetime(t0 + timedelta(seconds=dt))

    topo  = (satellite - observer).at(t)
    alt, az, dist = topo.altaz()
    topo2 = (satellite - observer).at(tp)
    alt2, az2, dist2 = topo2.altaz()

    el  = alt.degrees
    azd = az.degrees
    el2 = alt2.degrees
    az2d= az2.degrees

    # unwrap az for smooth rate
    d_az = az2d - azd
    if d_az > 180:  d_az -= 360
    if d_az < -180: d_az += 360

    az_rate = d_az / dt
    el_rate = (el2 - el) / dt
    rng_km  = dist.km
    return azd, el, az_rate, el_rate, rng_km

def clamp_slew(prev_cmd, target, max_rate_deg_s, dt):
    """Rate-limit target toward previous command."""
    max_step = max_rate_deg_s * dt
    delta = target - prev_cmd
    # wrap az difference to [-180,180] for az axis
    if isinstance(target, float) and "az" in clamp_slew.__name__:
        pass
    if delta >  max_step: return prev_cmd + max_step
    if delta < -max_step: return prev_cmd - max_step
    return target

def wrap_az(az):
    """Wrap azimuth to [0,360)."""
    az = math.fmod(az, 360.0)
    if az < 0: az += 360.0
    return az

def main():
    # Build observer and satellite
    observer = wgs84.latlon(OBS_LAT_DEG, OBS_LON_DEG, OBS_ALT_M)
    sat = build_satellite()

    # Open MD-03
    ctl = MD03Rot2(port=SERIAL_PORT, baud=BAUD)
    try:
        # Initialize commanded point to current rotor pos
        cur_az, cur_el, *_ = ctl.status()
        cmd_az, cmd_el = cur_az, cur_el
        log.info(f"Starting at rotor AZ={cur_az:.1f} EL={cur_el:.1f}")

        dt = 1.0 / UPDATE_HZ
        while True:
            now = datetime.now(timezone.utc)

            # Compute look angles and rates
            az, el, az_rate, el_rate, rng = topocentric_azel_and_rates(sat, observer, now)

            # Respect horizon mask
            if el < MIN_ELEV_DEG:
                # Below horizon: if we’re tracking, gracefully hold or park
                log.debug(f"Below MIN_EL ({el:.1f} < {MIN_ELEV_DEG}): holding.")
                time.sleep(dt)
                continue

            # Predictive lead
            az_lead = wrap_az(az + az_rate * LEAD_TIME_S)
            el_lead = el + el_rate * LEAD_TIME_S

            # Deadband around current command to avoid chatter
            if abs(((az_lead - cmd_az + 540) % 360) - 180) < DEADBAND_DEG and abs(el_lead - cmd_el) < DEADBAND_DEG:
                time.sleep(dt)
                continue

            # Rate limiting toward the lead point
            # Azimuth wrap-safe interpolation:
            # Compute shortest signed delta from cmd_az -> az_lead
            delta_az = ((az_lead - cmd_az + 540) % 360) - 180
            max_step_az = MAX_SLEW_DEG_S_AZ * dt
            if delta_az >  max_step_az: delta_az =  max_step_az
            if delta_az < -max_step_az: delta_az = -max_step_az
            cmd_az = wrap_az(cmd_az + delta_az)

            # Elevation limit
            delta_el = el_lead - cmd_el
            max_step_el = MAX_SLEW_DEG_S_EL * dt
            if delta_el >  max_step_el: delta_el =  max_step_el
            if delta_el < -max_step_el: delta_el = -max_step_el
            cmd_el = max(MIN_ELEV_DEG, min(90.0, cmd_el + delta_el))

            # Send command
            ctl.set_position(cmd_az, cmd_el)
            log.info(f"AZ={az:.1f} EL={el:.1f}  -> CMD {cmd_az:.1f}/{cmd_el:.1f}  rng={rng:.0f} km")
            time.sleep(dt)

    except KeyboardInterrupt:
        log.info("Stopping (Ctrl+C).")
        try:
            ctl.stop()
        except Exception:
            pass
    finally:
        ctl.close()

if __name__ == "__main__":
    main()
