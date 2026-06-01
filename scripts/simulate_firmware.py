#!/usr/bin/env python3
"""
=============================================================================
VERS Integration Test — End-to-End Simulation
Simulates the Zephyr firmware sending events to the cloud backend.
Run: python3 scripts/simulate_firmware.py
=============================================================================
"""

import time
import random
import math
import json
import argparse
import urllib.request
import urllib.error
from datetime import datetime

BACKEND = "http://localhost:8000"
DEVICE_ID = "VERS-NRF52840-001"


def post_event(payload: dict) -> bool:
    """POST a JSON payload to /api/event."""
    data = json.dumps(payload).encode()
    req  = urllib.request.Request(
        f"{BACKEND}/api/event",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            result = json.loads(resp.read())
            print(f"  ✓ Event #{result['event_id']} accepted (HTTP 200)")
            return True
    except urllib.error.URLError as e:
        print(f"  ✗ Backend unreachable: {e}")
        return False


def make_event(alert_type: str, event_id: int,
               accel_mag: float, bpm: int, spo2: int,
               lat: float, lon: float, speed: float) -> dict:
    """Build an EmergencyEvent JSON payload."""
    angle = random.uniform(0, 2 * math.pi)
    ratio = random.uniform(0.4, 0.9)
    x_g   = accel_mag * ratio * math.cos(angle)
    y_g   = accel_mag * ratio * math.sin(angle)
    z_g   = math.sqrt(max(0, accel_mag**2 - x_g**2 - y_g**2))

    return {
        "device_id":   DEVICE_ID,
        "event_id":    event_id,
        "alert_type":  alert_type,
        "accel": {
            "x_g": round(x_g, 3),
            "y_g": round(y_g, 3),
            "z_g": round(z_g, 3),
            "magnitude_g": round(accel_mag, 3),
        },
        "heart_rate": {
            "bpm":   bpm,
            "spo2":  spo2,
            "valid": True,
        },
        "gps": {
            "latitude":   round(lat, 6),
            "longitude":  round(lon, 6),
            "speed_kmh":  round(speed, 1),
            "altitude":   15.0,
            "satellites": random.randint(6, 12),
            "fix_valid":  True,
            "utc_time":   datetime.utcnow().strftime("%H:%M:%S"),
            "utc_date":   datetime.utcnow().strftime("%d/%m/%y"),
        },
        "firmware_ts": int(time.time() * 1000),
    }


def run_scenario():
    """
    Simulate 60-second drive scenario:
      0–15s   Normal driving
      15s     Hard braking (4g)
      15–30s  Recovery
      30s     CRASH (45g)
      33s     Post-crash (low HR)
      53s     Auto-recovery
    """
    lat = 37.422160
    lon = -122.084270
    event_id = 1
    speed = 65.0

    print("=" * 60)
    print("  VERS Firmware Simulation — 60s Drive Scenario")
    print("=" * 60)

    # Phase 1: Normal driving (0–15s)
    print("\n[Phase 1] Normal driving (0–15s)...")
    for i in range(15):
        accel = 1.0 + random.gauss(0, 0.05)
        bpm   = int(random.gauss(72, 3))
        lat  += random.gauss(0, 0.00002)
        lon  += random.gauss(0, 0.00001)
        print(f"  t={i:02d}s | |G|={accel:.2f}g  HR={bpm}BPM  spd={speed:.0f}km/h")
        time.sleep(1)

    # Phase 2: Hard braking
    print("\n[Phase 2] Hard braking event (4g spike)...")
    ev = make_event("CRASH", event_id, 4.0, 82, 96, lat, lon, 5.0)
    # Not a real crash — just a spike — in firmware this would be < CRASH_G_THRESHOLD
    ev["alert_type"] = "CRASH"
    ev["accel"]["magnitude_g"] = 4.0
    # (In real firmware 4g < 30g so no alert fires — shown for illustration)
    print("  [FW] 4g spike detected — below threshold (30g), no alert")
    speed = 8.0
    time.sleep(2)

    # Phase 3: Recovery (15–30s)
    print("\n[Phase 3] Recovering (15–30s)...")
    for i in range(13):
        accel = 1.0 + random.gauss(0, 0.04)
        print(f"  t={i+17:02d}s | |G|={accel:.2f}g  HR=70BPM  spd={speed:.0f}km/h")
        speed += 3
        time.sleep(1)

    # Phase 4: CRASH
    print("\n[Phase 4] *** CRASH EVENT at t=30s ***")
    lat += random.gauss(0, 0.0001)
    lon += random.gauss(0, 0.0001)
    ev = make_event("CRASH", event_id, 45.0, 45, 88, lat, lon, 0.0)
    post_event(ev)
    event_id += 1
    time.sleep(3)

    # Phase 5: Post-crash (low HR)
    print("\n[Phase 5] Post-crash — driver unconscious (HR=28)")
    ev = make_event("HR_LOW", event_id, 1.0, 28, 82, lat, lon, 0.0)
    post_event(ev)
    event_id += 1
    time.sleep(5)

    # Phase 6: Rollover
    print("\n[Phase 6] Rollover detection")
    ev = make_event("ROLLOVER", event_id, 44.3, 55, 90, lat, lon, 0.0)
    post_event(ev)
    event_id += 1
    time.sleep(3)

    print("\n✅ Scenario complete — check dashboard for events")


def run_continuous(interval: float = 5.0):
    """Send a random event every `interval` seconds."""
    alert_types  = ["CRASH", "HR_LOW", "HR_HIGH", "ROLLOVER", "MANUAL_SOS"]
    event_id     = 1
    lat, lon     = 37.422160, -122.084270

    print(f"Continuous mode — sending event every {interval}s (Ctrl+C to stop)\n")
    while True:
        alert = random.choice(alert_types)
        lat  += random.gauss(0, 0.0003)
        lon  += random.gauss(0, 0.0003)
        params = {
            "CRASH":      (random.uniform(30, 55), random.randint(35, 80),  88),
            "HR_LOW":     (random.uniform(0.9, 1.1), random.randint(18, 40), 80),
            "HR_HIGH":    (random.uniform(0.9, 1.1), random.randint(180, 210), 96),
            "ROLLOVER":   (random.uniform(35, 50), random.randint(50, 100), 92),
            "MANUAL_SOS": (1.0, random.randint(70, 110), 97),
        }
        mag, bpm, spo2 = params[alert]
        speed = random.uniform(0, 100) if alert in ["CRASH","ROLLOVER"] else random.uniform(10,90)
        ev = make_event(alert, event_id, mag, bpm, spo2, lat, lon, speed)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Sending {alert} event #{event_id}...")
        post_event(ev)
        event_id += 1
        time.sleep(interval)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VERS Firmware Simulator")
    parser.add_argument("--mode",     choices=["scenario","continuous"], default="scenario")
    parser.add_argument("--interval", type=float, default=5.0, help="Seconds between events (continuous mode)")
    parser.add_argument("--backend",  default="http://localhost:8000", help="Backend URL")
    args = parser.parse_args()

    BACKEND = args.backend

    if args.mode == "continuous":
        run_continuous(args.interval)
    else:
        run_scenario()
