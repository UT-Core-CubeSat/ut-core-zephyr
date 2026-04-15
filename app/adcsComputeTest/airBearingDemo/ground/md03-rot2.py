#!/usr/bin/env python3
import time
import logging
import serial
import serial.tools.list_ports

log = logging.getLogger("MD03")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

START = 0x57
END   = 0x20
CMD_STOP   = 0x0F
CMD_STATUS = 0x1F
CMD_SET    = 0x2F

class MD03Rot2:
    def __init__(self, port=None, baud=9600, timeout=1.0):
        self.port = port or self._autodetect_port()
        self.ser = serial.Serial(
            port=self.port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )
        log.info(f"Opened {self.ser.port} @ {self.ser.baudrate} (8N1)")
        # learn PH/PV from a status read
        _, _, self.ph, self.pv = self.status()
        log.info(f"Controller resolution: PH={self.ph} (az), PV={self.pv} (el)")

    @staticmethod
    def _autodetect_port():
        # Prefer ttyACM*, then ttyUSB*
        ports = list(serial.tools.list_ports.comports())
        candidates = []
        for p in ports:
            desc = (p.description or "").lower()
            mfr  = (p.manufacturer or "").lower()
            if any(s in desc for s in ("spid", "md-03", "md03")) or "spid" in mfr:
                candidates.append(p.device)
        if not candidates:
            for p in ports:
                if "/ttyACM" in p.device or "/ttyUSB" in p.device:
                    candidates.append(p.device)
        if not candidates:
            raise RuntimeError("Could not find MD-03 serial port. Plug it in and try again.")
        log.info(f"Candidate device(s): {candidates}")
        return candidates[0]

    # -------- packet helpers (ROT2 protocol) --------
    @staticmethod
    def _ascii4(n):
        """Four ASCII digits '0000'..'9999' as bytes from integer n (0..9999)."""
        n = max(0, min(9999, int(round(n))))
        return bytes(f"{n:04d}", "ascii")

    @staticmethod
    def _cmd_packet(h_digits=b"0000", ph=0, v_digits=b"0000", pv=0, cmd=CMD_STATUS):
        return bytes([START]) + h_digits + bytes([ph]) + v_digits + bytes([pv, cmd, END])

    def _send(self, packet):
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()

    def _read_exact(self, nbytes=12):
        data = self.ser.read(nbytes)
        if len(data) != nbytes:
            raise TimeoutError(f"Expected {nbytes}B, got {len(data)}B")
        return data

    # -------- high-level commands --------
    def stop(self):
        """Stop motion immediately; returns (az_deg, el_deg, PH, PV)."""
        pkt = self._cmd_packet(b"0000", 0, b"0000", 0, CMD_STOP)
        self._send(pkt)
        return self._decode_response(self._read_exact())

    def status(self):
        """Query current position; returns (az_deg, el_deg, PH, PV)."""
        pkt = self._cmd_packet(b"0000", 0, b"0000", 0, CMD_STATUS)
        self._send(pkt)
        return self._decode_response(self._read_exact())

    def set_position(self, az_deg=None, el_deg=None):
        """
        Command a new az/el. MD-03 ignores PH/PV in the command; it uses its own
        resolution, but ROT2 expects H/V digits computed with the controller's PH/PV.
        """
        # If a component is None, keep current value
        cur_az, cur_el, ph, pv = self.status()
        az = cur_az if az_deg is None else float(az_deg)
        el = cur_el if el_deg is None else float(el_deg)

        # Encode per ROT2: H = PH * (az + 360), V = PV * (el + 360)
        # PH/PV come from controller (status response)
        H = ph * (az + 360.0)
        V = pv * (el + 360.0)
        h_digits = self._ascii4(H)
        v_digits = self._ascii4(V)

        pkt = self._cmd_packet(h_digits, ph, v_digits, pv, CMD_SET)
        self._send(pkt)
        # SET has no reply; poll until near target
        return az, el

    def wait_until_reached(self, az_target, el_target, tol_deg=1.0, timeout_s=120):
        t0 = time.time()
        while True:
            az, el, *_ = self.status()
            if abs((az - az_target)) <= tol_deg and abs((el - el_target)) <= tol_deg:
                return True
            if (time.time() - t0) > timeout_s:
                return False
            time.sleep(0.2)

    # -------- decoding --------
    @staticmethod
    def _decode_response(resp):
        """
        Response: 12B: [0]=0x57, [1..4]=H1..H4, [5]=PH, [6..9]=V1..V4, [10]=PV, [11]=0x20
        az = (H1*100 + H2*10 + H3 + H4/10) - 360
        el = (V1*100 + V2*10 + V3 + V4/10) - 360
        """
        if len(resp) != 12 or resp[0] != START or resp[-1] != END:
            raise ValueError(f"Bad response: {resp.hex()}")
        H1, H2, H3, H4 = resp[1], resp[2], resp[3], resp[4]
        PH = resp[5]
        V1, V2, V3, V4 = resp[6], resp[7], resp[8], resp[9]
        PV = resp[10]
        az = (H1 * 100) + (H2 * 10) + (H3) + (H4 / 10.0) - 360.0
        el = (V1 * 100) + (V2 * 10) + (V3) + (V4 / 10.0) - 360.0
        return az, el, PH, PV

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="SPID MD-03 (ROT2) controller")
    ap.add_argument("--port", help="Serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=9600)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status")
    sub.add_parser("stop")

    p_set = sub.add_parser("set")
    p_set.add_argument("--az", type=float, help="Azimuth degrees")
    p_set.add_argument("--el", type=float, help="Elevation degrees")
    p_set.add_argument("--wait", action="store_true", help="Wait until reached")
    p_set.add_argument("--tol", type=float, default=1.0)
    p_set.add_argument("--timeout", type=float, default=180.0)

    args = ap.parse_args()
    ctl = MD03Rot2(port=args.port, baud=args.baud)
    try:
        if args.cmd == "status":
            az, el, ph, pv = ctl.status()
            print(f"AZ={az:.1f}°, EL={el:.1f}° (PH={ph}, PV={pv})")
        elif args.cmd == "stop":
            az, el, ph, pv = ctl.stop()
            print(f"STOPPED @ AZ={az:.1f}°, EL={el:.1f}°")
        elif args.cmd == "set":
            target_az, target_el = ctl.set_position(args.az, args.el)
            print(f"Commanded: AZ={target_az:.1f}°, EL={target_el:.1f}°")
            if args.wait:
                ok = ctl.wait_until_reached(target_az, target_el, tol_deg=args.tol, timeout_s=args.timeout)
                print("Reached." if ok else "Timed out.")
    finally:
        ctl.close()
