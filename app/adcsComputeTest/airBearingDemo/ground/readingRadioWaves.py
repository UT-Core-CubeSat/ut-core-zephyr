#!/usr/bin/env python3
"""
HackRF -> NBFM demod -> 48 kHz mono audio -> UDP (localhost:7355)
Use with Direwolf:  direwolf -r 48000 udp:7355
Requires: SoapySDR, numpy, scipy
    pip install SoapySDR numpy scipy
"""
import SoapySDR
from SoapySDR import *               # type: ignore
import numpy as np
from scipy.signal import decimate, firwin, lfilter
import socket, time

# ---------- CONFIG ----------
CENTER_HZ   = 145_825_000          # ISS APRS downlink (FM) 145.825 MHz
FS_IQ       = 2_000_000            # 2 MS/s complex (HackRF sweet spot)
AUDIO_FS    = 48_000               # Direwolf-friendly sample rate
RF_BW       = 200_000              # front-end BW ~200 kHz
GAIN_DB     = 20                   # adjust as needed
UDP_HOST    = "127.0.0.1"
UDP_PORT    = 7355                 # we'll send raw 16-bit PCM here
CHUNK_IQ    = 4096 * 12            # samples per read (tweak)
WB_DEVIATION= 5_000                # NBFM-ish (voice/APRS fits fine)
# -----------------------------

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- SDR setup (HackRF via Soapy) ---
args = dict(driver="hackrf")
sdr = SoapySDR.Device(args)
sdr.setSampleRate(SOAPY_SDR_RX, 0, FS_IQ)
sdr.setFrequency(SOAPY_SDR_RX, 0, CENTER_HZ)
sdr.setBandwidth(SOAPY_SDR_RX, 0, RF_BW)
sdr.setGain(SOAPY_SDR_RX, 0, GAIN_DB)

# Stream
rx = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])  # complex float32
sdr.activateStream(rx)

# --- Design audio LPF (limit post-discriminator audio ~4 kHz) ---
# We'll FM-demod then lowpass and decimate to 48 kHz.
# Overall decimation factor:
decim_total = FS_IQ // 240_000        # 2e6 -> 240 kHz (factor ~8 or so)
fs_post1 = FS_IQ // decim_total
# Second stage: 240 kHz -> 48 kHz (factor 5)
decim2 = fs_post1 // AUDIO_FS

lp_cut = 4_000.0 / (fs_post1/2.0)     # normalized cutoff after first decim
b_lpf1 = firwin(129, lp_cut)

def fm_demod(iq):
    # complex to phase
    phase = np.angle(iq)
    # unwrap and diff
    dphase = np.diff(np.unwrap(phase))
    # scale roughly to audio
    return dphase * (fs_post1/(2*np.pi)) / (WB_DEVIATION/ (2*np.pi))

try:
    print(f"Streaming from HackRF @ {FS_IQ/1e6:.1f} MS/s, center {CENTER_HZ/1e6:.3f} MHz")
    # State for decimation filter
    zi = np.zeros(128)  # lpf state placeholder (not strictly needed with lfilter returning zi if wanted)

    while True:
        sr = sdr.readStream(rx, [np.empty(CHUNK_IQ, np.complex64)], CHUNK_IQ, timeoutUs=250000)
        if sr.ret > 0:
            iq = sr.buff[0][:sr.ret]

            # Stage 1: decimate complex IQ down to ~240 kHz to reduce load
            iq_dec = decimate(iq, decim_total, ftype='fir', zero_phase=True)

            # FM discriminator
            audio_wide = fm_demod(iq_dec)

            # Lowpass (limit to ~4 kHz)
            audio_filt = lfilter(b_lpf1, 1.0, audio_wide)

            # Final decimation to 48 kHz
            audio_out = decimate(audio_filt, decim2, ftype='fir', zero_phase=True)

            # Normalize & clip to int16
            audio_out = np.clip(audio_out * 1000.0, -32767, 32767).astype(np.int16)

            # Send via UDP as raw PCM16 mono
            sock.sendto(audio_out.tobytes(), (UDP_HOST, UDP_PORT))
        else:
            time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    sdr.deactivateStream(rx)
    sdr.closeStream(rx)
