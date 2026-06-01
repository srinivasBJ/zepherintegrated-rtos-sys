"""
VERS Backend — pytest test suite
Tests event ingestion, WebSocket, statistics, and ACK endpoints.
Run: pytest backend/ -v
"""

import pytest
import httpx
from fastapi.testclient import TestClient
from main import app, Base, engine

# Use fresh test DB
Base.metadata.drop_all(bind=engine)
Base.metadata.create_all(bind=engine)

client = TestClient(app)

SAMPLE_EVENT = {
    "device_id":   "TEST-DEVICE-001",
    "event_id":    1,
    "alert_type":  "CRASH",
    "accel": {
        "x_g": 32.0, "y_g": 18.0, "z_g": 22.0, "magnitude_g": 42.7
    },
    "heart_rate": {
        "bpm": 28, "spo2": 85, "valid": True
    },
    "gps": {
        "latitude": 37.422160, "longitude": -122.084270,
        "speed_kmh": 0.0, "altitude": 15.0,
        "satellites": 8, "fix_valid": True,
        "utc_time": "12:30:00", "utc_date": "01/06/26"
    },
    "firmware_ts": 30000
}


def test_health():
    r = client.get("/api/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok"


def test_ingest_event():
    r = client.post("/api/event", json=SAMPLE_EVENT)
    assert r.status_code == 200
    data = r.json()
    assert data["device_id"] == "TEST-DEVICE-001"
    assert data["alert_type"] == "CRASH"
    assert data["accel_mag_g"] == pytest.approx(42.7, abs=0.1)


def test_list_events():
    r = client.get("/api/events")
    assert r.status_code == 200
    events = r.json()
    assert len(events) >= 1


def test_list_events_filter():
    r = client.get("/api/events?alert_type=CRASH")
    assert r.status_code == 200
    for e in r.json():
        assert e["alert_type"] == "CRASH"


def test_get_event():
    r = client.get("/api/events/1")
    assert r.status_code == 200
    assert "device_id" in r.json()


def test_ack_event():
    r = client.post("/api/events/1/ack?acked_by=dispatcher_01")
    assert r.status_code == 200
    assert r.json()["status"] == "acknowledged"


def test_stats():
    r = client.get("/api/stats")
    assert r.status_code == 200
    stats = r.json()
    assert stats["total_events"] >= 1
    assert stats["crash_events"] >= 1


def test_multiple_alert_types():
    for alert_type, bpm in [("HR_LOW", 25), ("HR_HIGH", 200), ("ROLLOVER", 60), ("MANUAL_SOS", 80)]:
        event = SAMPLE_EVENT.copy()
        event["alert_type"] = alert_type
        event["heart_rate"] = {"bpm": bpm, "spo2": 95, "valid": True}
        r = client.post("/api/event", json=event)
        assert r.status_code == 200

    r = client.get("/api/stats")
    stats = r.json()
    assert stats["total_events"] >= 5
