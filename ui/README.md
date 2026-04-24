# UT-CORE Ground Station + M5Stamp CAN Bridge

This folder contains two pieces that work together:

- `ground_station/`: Python UI (PyQt5 + pyqtgraph) that listens for UDP CAN packets.
- `m5_can_bridge/`: M5Stamp S3 firmware that receives CAN frames and forwards them to your laptop over UDP.

The flow is:

1. M5Stamp S3 reads CAN (or simulated frames).
2. M5Stamp sends 21-byte UDP packets to your laptop on port 5005.
3. Python UI receives packets, visualizes them, and logs per-node CSV files.

---

## 1) Prerequisites

Install these first:

- Python 3.10+ (with `pip`)
- Git (if cloning the repo)
- One firmware workflow:
  - PlatformIO (recommended), or
  - Arduino IDE
- USB-C cable for flashing the M5Stamp S3
- Windows hotspot capability (2.4 GHz)

Python packages are listed in `ground_station/requirements.txt`:

- `PyQt5>=5.15`
- `pyqtgraph>=0.13`

If you need PlatformIO Core CLI, install it with:

```powershell
python -m pip install --user platformio
```

---

## 2) Configure The M5 Bridge

Open `m5_can_bridge/src/main.cpp` and set these values:

- `WIFI_SSID`
- `WIFI_PASSWORD`
- `LAPTOP_IP`
- `UDP_PORT` (default is `5005`)

Important notes:

- `LAPTOP_IP` should be your hotspot gateway IP (commonly `192.168.137.1` on Windows).
- `#define SIMULATE` is enabled by default.
  - Keep it enabled to test UI without CAN hardware.
  - Comment it out for real CAN bus traffic.

If using real CAN mode, wiring expected by firmware:

- `GPIO5` -> transceiver TXD
- `GPIO4` -> transceiver RXD
- `3V3/GND` -> transceiver VCC/GND

---

## 3) Windows Hotspot + Firewall Setup

1. Open Windows Mobile Hotspot settings.
2. Set hotspot SSID/password to match `WIFI_SSID` and `WIFI_PASSWORD` in firmware.
3. Set hotspot band to **2.4 GHz**.
4. Open Windows Defender Firewall with Advanced Security.
5. Add an **Inbound** UDP rule for port **5005** (or your configured `UDP_PORT`).
6. Enable rule for all profiles (Domain/Private/Public).
7. If needed, restart hotspot (or reboot laptop).

To verify hotspot IP, run:

```powershell
ipconfig
```

Find the adapter for the mobile hotspot and confirm it matches `LAPTOP_IP`.

---

## 4) Build And Flash M5 Firmware

### Option A: PlatformIO (recommended)

From repo root:

```powershell
cd ui/m5_can_bridge
pio run
pio run -t upload
pio device monitor --baud 115200
```

The serial monitor should show Wi-Fi connect status and target UDP endpoint.

### Option B: Arduino IDE

1. Create a new sketch for M5Stamp S3.
2. Copy contents of `m5_can_bridge/src/main.cpp` into the sketch.
3. Install required ESP32/M5 board support package.
4. Select the correct board/port and upload.

---

## 5) Set Up Python UI Environment

From repo root:

```powershell
cd ui
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r ground_station/requirements.txt
```

If PowerShell script execution is blocked, use Command Prompt activation instead:

```bat
.venv\Scripts\activate.bat
```

---

## 6) Run And Verify

Start from `ui/` directory after activating the venv.

Optional smoke test (prints every UDP frame):

```powershell
python ground_station/udp_receiver.py
```

Run UI:

```powershell
python ground_station/ui.py
```

Expected behavior:

- Window title shows `UDP :5005`.
- Node Health tab turns nodes green as frames arrive.
- Frame Log fills with decoded traffic.
- CSV logs are created in `logs/` (relative to your current working directory).

---

## 7) Troubleshooting

### No frames in UI

- Verify M5 serial output says Wi-Fi connected.
- Confirm `LAPTOP_IP` in firmware matches hotspot gateway IP.
- Confirm Windows firewall inbound UDP rule for port `5005`.
- Confirm M5 and laptop are on the same hotspot network.
- Ensure `UDP_PORT` matches in both firmware and `ground_station/ui.py`.

### Python import errors (`ModuleNotFoundError`)

- Confirm venv is active.
- Re-run:

```powershell
pip install -r ground_station/requirements.txt
```

### M5 uploads but no CAN data (real hardware mode)

- Ensure `SIMULATE` is commented out.
- Check transceiver wiring (`GPIO5` TX, `GPIO4` RX).
- Confirm bus bitrate and physical CAN bus health.

### Logs are written in unexpected folder

- `CsvLogger(log_dir="logs")` is relative to where you launch Python.
- If you run from `ui/`, logs go to `ui/logs/`.

---

## 8) Useful Paths

- `ground_station/ui.py`: main Qt UI
- `ground_station/udp_receiver.py`: UDP packet receiver + decoder
- `ground_station/csv_logger.py`: per-node CSV logger
- `ground_station/requirements.txt`: Python deps
- `m5_can_bridge/src/main.cpp`: M5 bridge firmware
- `m5_can_bridge/platformio.ini`: PlatformIO board config
