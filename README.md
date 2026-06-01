# Vehicle Emergency Response System (VERS)
# Zephyr RTOS — Complete Project

[![Zephyr RTOS](https://img.shields.io/badge/Zephyr-3.x-blue?logo=zephyr)](https://zephyrproject.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Python](https://img.shields.io/badge/Backend-FastAPI-009688)](backend/)
[![Platform](https://img.shields.io/badge/Platform-nRF52840%20%7C%20POSIX-orange)](firmware/)

> **Resume-ready description**: Developed a real-time vehicle emergency response system using Zephyr RTOS, integrating accelerometer-based crash detection, heart-rate monitoring, GPS tracking, and GSM communication. Implemented multitasking architecture for sensor acquisition, decision logic, and emergency notification. Designed backend integration for event logging and monitoring.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  Zephyr RTOS Firmware                         │
│                                                              │
│  Thread A: Sensor Acquisition (50ms period)                  │
│    ├─ ADXL345  I²C  — 3-axis acceleration (±16g, 100Hz)     │
│    ├─ MAX30102 I²C  — Heart-rate + SpO₂                     │
│    └─ PA1010D  UART — GPS NMEA 0183 parser                  │
│                                                              │
│  Thread B: Decision Engine (100ms / ISR wakeup)             │
│    ├─ Crash detection: |G| ≥ 30g, SMA filter, 2s confirm    │
│    ├─ Rollover: tilt angle > 45°                            │
│    ├─ Vital alert: BPM < 40 or > 180                       │
│    └─ State machine: NORMAL → CRASH_ALERT → EMERGENCY       │
│                                                              │
│  Thread C: Communication Manager                             │
│    ├─ HTTP POST → Cloud backend (JSON)                      │
│    └─ SMS fallback → SIM800L GSM modem (AT commands)        │
│                                                              │
│  ISR: ADXL345 INT1 GPIO → instant wakeup of Thread B        │
└──────────────────────────────────────────────────────────────┘
             │  HTTP POST / SMS
             ▼
┌──────────────────────────┐
│  FastAPI Cloud Backend   │  SQLite · WebSocket · REST
│  POST /api/event         │
│  GET  /api/events        │
│  GET  /api/stats         │
│  WS   /ws                │
└──────────────────────────┘
             │  WebSocket
             ▼
┌──────────────────────────┐
│  Live Dashboard           │  Leaflet Map · Chart.js
│  dashboard/index.html    │  Crash alerts · Event log
└──────────────────────────┘
```

---

## Project Structure

```
zepher/
├── firmware/                    # Zephyr RTOS C firmware
│   ├── CMakeLists.txt           # Build configuration
│   ├── prj.conf                 # Zephyr Kconfig
│   ├── boards/
│   │   ├── native_posix.overlay         # Simulation (host machine)
│   │   └── nrf52840dk_nrf52840.overlay  # Hardware target
│   ├── include/
│   │   └── vers_types.h         # Shared types, zbus channels, constants
│   └── src/
│       ├── main.c               # Entry point, thread creation, ISR
│       ├── sensors/
│       │   ├── accel.c/h        # ADXL345 driver + SMA filter
│       │   ├── heartrate.c/h    # MAX30102 driver + peak/SpO₂ algo
│       │   └── gps.c/h          # NMEA 0183 parser + UART ISR
│       ├── decision/
│       │   └── engine.c/h       # Crash state machine
│       ├── comm/
│       │   └── gsm.c/h          # SIM800L AT command driver
│       ├── cloud/
│       │   └── http_client.c/h  # Zephyr HTTP POST to backend
│       └── sim/
│           └── sensor_sim.c/h   # native_posix crash scenario
├── backend/                     # Python cloud backend
│   ├── main.py                  # FastAPI app
│   ├── requirements.txt
│   └── test_backend.py          # pytest suite
├── dashboard/
│   └── index.html               # Live monitoring dashboard
├── scripts/
│   └── simulate_firmware.py     # Firmware event simulator
└── docs/
    └── architecture.md          # Detailed system documentation
```

---

## Hardware Components

| Component | Interface | Function |
|-----------|-----------|----------|
| **ADXL345**  | I²C (0x53) | 3-axis accelerometer, crash detection, INT1 interrupt |
| **MAX30102** | I²C (0x57) | Heart-rate + SpO₂ (IR/Red LED photodetector) |
| **PA1010D**  | UART 9600  | GPS NMEA 0183, lat/lon/speed/altitude |
| **SIM800L**  | UART 115200| GSM modem, AT commands, SMS + HTTP |
| **nRF52840** | —          | ARM Cortex-M4 MCU, BLE (optional), Zephyr RTOS |

---

## Quick Start

### Option 1 — Software Only (No Hardware Required)

```bash
# 1. Start the cloud backend
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000 --reload

# 2. Open the dashboard
open dashboard/index.html          # macOS
# or: firefox dashboard/index.html

# 3. Run the firmware simulator
python3 scripts/simulate_firmware.py              # 60s scenario
python3 scripts/simulate_firmware.py --mode continuous --interval 3
```

### Option 2 — Zephyr native_posix Simulation

```bash
# Install Zephyr SDK and west (see https://docs.zephyrproject.org/latest/getting_started)
cd firmware
west build -b native_posix .
./build/zephyr/zephyr.exe

# The firmware connects to backend at 127.0.0.1:8000 (loopback networking)
```

### Option 3 — Real Hardware (nRF52840-DK)

```bash
cd firmware
west build -b nrf52840dk_nrf52840 .
west flash

# Monitor logs
west espressif monitor  # or: minicom -D /dev/ttyACM0 -b 115200
```

---

## API Reference

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/event` | Ingest emergency event (firmware → backend) |
| `GET`  | `/api/events` | List all events (paginated, filterable) |
| `GET`  | `/api/events/{id}` | Get raw event payload |
| `POST` | `/api/events/{id}/ack` | Acknowledge event |
| `GET`  | `/api/stats` | Aggregate statistics |
| `GET`  | `/api/health` | Health check |
| `WS`   | `/ws` | Live WebSocket push to dashboard |

Interactive docs: `http://localhost:8000/docs`

---

## Running Tests

```bash
cd backend
pip install pytest pytest-asyncio httpx
pytest test_backend.py -v
```

---

## Zephyr RTOS Concepts Demonstrated

| Concept | Implementation |
|---------|---------------|
| **k_thread_create** | 3 cooperating threads (sensor/decision/comm) |
| **k_sem_take / k_sem_give** | GPIO ISR wakes decision thread |
| **ZBUS channels** | Type-safe inter-thread message bus |
| **Interrupt-driven UART** | GPS parser, GSM modem RX |
| **I²C sensor API** | `sensor_sample_fetch`, `sensor_channel_get` |
| **GPIO callbacks** | ADXL345 INT1 hardware interrupt |
| **Work queues** | System work queue for deferred processing |
| **Device tree** | Board overlays for hardware abstraction |
| **HTTP client** | Zephyr's built-in `http_client_req` |
| **native_posix** | Simulation target for CI/dev testing |

---

## Algorithm Details

### Crash Detection
1. Raw 3-axis acceleration sampled at 100 Hz
2. Magnitude computed: `|G| = √(x² + y² + z²)`
3. Simple Moving Average (SMA) filter over 8 samples
4. If `|G| ≥ 30g` → enter **CRASH_ALERT** state
5. 2-second confirmation window prevents false positives
6. If sustained → **EMERGENCY** fired, event dispatched

### Heart-Rate Detection (MAX30102)
- IR LED photodetector signal buffered (32 samples)
- Peak-to-peak detection with dynamic threshold
- BPM = peaks × (60 / buffer_duration)
- SpO₂ via ratio-of-ratios: `R = (AC_red/DC_red) / (AC_ir/DC_ir)` → `SpO₂ ≈ 104 − 17R`

### GPS Parsing
- NMEA 0183 sentences: `$GPRMC` (position/speed/time) + `$GPGGA` (altitude/satellites)
- Coordinate conversion: DDDMM.MMMM → decimal degrees
- XOR checksum validated on every sentence

---

## Industrial Relevance

This architecture mirrors production systems used in:
- **Automotive Safety** — eCall (EU mandate), ACAS
- **Fleet Management** — Samsara, Geotab, Verizon Connect
- **Ambulance Automation** — automatic dispatch on vehicle sensor triggers
- **Insurance Telematics** — usage-based insurance black boxes

---

## License

MIT License — © 2026 VERS Project
