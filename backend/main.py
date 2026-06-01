"""
=============================================================================
Vehicle Emergency Response System — Cloud Backend
FastAPI + SQLite + WebSocket

Endpoints:
  POST /api/event          — receive emergency event from firmware
  GET  /api/events         — list all events (paginated)
  GET  /api/events/{id}    — get single event
  GET  /api/health         — health check
  GET  /api/stats          — aggregate statistics
  WS   /ws                 — live push to dashboard clients

Run: uvicorn main:app --host 0.0.0.0 --port 8000 --reload
=============================================================================
"""

from __future__ import annotations

import asyncio
import json
import logging
from datetime import datetime, timezone
from typing import Optional, List

import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, field_validator
from sqlalchemy import (
    create_engine, Column, Integer, String, Float,
    Boolean, DateTime, Enum as SAEnum, Text,
)
from sqlalchemy.orm import declarative_base, Session, sessionmaker
import enum

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
)
logger = logging.getLogger("vers.backend")

# ---------------------------------------------------------------------------
# Database
# ---------------------------------------------------------------------------
DATABASE_URL = "sqlite:///./vers_events.db"
engine = create_engine(DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


class AlertTypeEnum(str, enum.Enum):
    CRASH       = "CRASH"
    HR_LOW      = "HR_LOW"
    HR_HIGH     = "HR_HIGH"
    NO_GPS      = "NO_GPS"
    ROLLOVER    = "ROLLOVER"
    MANUAL_SOS  = "MANUAL_SOS"
    UNKNOWN     = "UNKNOWN"


class EmergencyEventDB(Base):
    __tablename__ = "emergency_events"

    id            = Column(Integer, primary_key=True, index=True)
    device_id     = Column(String(64), index=True)
    event_id      = Column(Integer)
    alert_type    = Column(SAEnum(AlertTypeEnum))
    received_at   = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    firmware_ts   = Column(Integer)          # k_uptime_get() ms

    # Accelerometer
    accel_x_g     = Column(Float)
    accel_y_g     = Column(Float)
    accel_z_g     = Column(Float)
    accel_mag_g   = Column(Float)

    # Heart-rate
    hr_bpm        = Column(Integer)
    hr_spo2       = Column(Integer)
    hr_valid      = Column(Boolean)

    # GPS
    latitude      = Column(Float)
    longitude     = Column(Float)
    speed_kmh     = Column(Float)
    altitude      = Column(Float)
    satellites    = Column(Integer)
    fix_valid     = Column(Boolean)
    utc_time      = Column(String(12))
    utc_date      = Column(String(10))

    # Acknowledgement
    acked         = Column(Boolean, default=False)
    acked_by      = Column(String(64), nullable=True)
    raw_payload   = Column(Text)


Base.metadata.create_all(bind=engine)

# ---------------------------------------------------------------------------
# Pydantic schemas
# ---------------------------------------------------------------------------

class AccelPayload(BaseModel):
    x_g: float = 0.0
    y_g: float = 0.0
    z_g: float = 0.0
    magnitude_g: float = 0.0


class HRPayload(BaseModel):
    bpm: int = 0
    spo2: int = 0
    valid: bool = False


class GPSPayload(BaseModel):
    latitude:   float = 0.0
    longitude:  float = 0.0
    speed_kmh:  float = 0.0
    altitude:   float = 0.0
    satellites: int   = 0
    fix_valid:  bool  = False
    utc_time:   str   = ""
    utc_date:   str   = ""


class EmergencyEventIn(BaseModel):
    device_id:   str = Field(..., max_length=64)
    event_id:    int = Field(..., ge=0)
    alert_type:  AlertTypeEnum
    accel:       AccelPayload
    heart_rate:  HRPayload
    gps:         GPSPayload
    firmware_ts: int = 0


class EmergencyEventOut(BaseModel):
    id:          int
    device_id:   str
    event_id:    int
    alert_type:  str
    received_at: str
    accel_mag_g: float
    hr_bpm:      int
    hr_spo2:     int
    latitude:    float
    longitude:   float
    speed_kmh:   float
    fix_valid:   bool
    acked:       bool

    class Config:
        from_attributes = True


class StatsOut(BaseModel):
    total_events:   int
    crash_events:   int
    hr_alerts:      int
    manual_sos:     int
    unique_devices: int
    avg_bpm:        float
    avg_magnitude:  float

# ---------------------------------------------------------------------------
# WebSocket connection manager
# ---------------------------------------------------------------------------

class ConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)
        logger.info(f"WS client connected — total={len(self.active)}")

    def disconnect(self, ws: WebSocket):
        self.active.remove(ws)
        logger.info(f"WS client disconnected — total={len(self.active)}")

    async def broadcast(self, data: dict):
        payload = json.dumps(data)
        dead = []
        for ws in self.active:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.active.remove(ws)


manager = ConnectionManager()

# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app = FastAPI(
    title="Vehicle Emergency Response System — Cloud Backend",
    description="Real-time emergency event ingestion and monitoring API",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.get("/api/health", tags=["System"])
def health():
    return {
        "status": "ok",
        "service": "VERS Cloud Backend",
        "version": "1.0.0",
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }


@app.post("/api/event", response_model=EmergencyEventOut, tags=["Events"])
async def ingest_event(payload: EmergencyEventIn):
    """
    Receive an emergency event from Zephyr firmware (HTTP POST).
    Persists to SQLite and broadcasts to all WebSocket dashboard clients.
    """
    logger.warning(
        f"🚨 EMERGENCY [{payload.alert_type}] device={payload.device_id} "
        f"event_id={payload.event_id} |G|={payload.accel.magnitude_g:.1f} "
        f"HR={payload.heart_rate.bpm}bpm GPS={payload.gps.latitude:.5f},{payload.gps.longitude:.5f}"
    )

    db: Session = SessionLocal()
    try:
        db_event = EmergencyEventDB(
            device_id    = payload.device_id,
            event_id     = payload.event_id,
            alert_type   = payload.alert_type,
            firmware_ts  = payload.firmware_ts,
            accel_x_g    = payload.accel.x_g,
            accel_y_g    = payload.accel.y_g,
            accel_z_g    = payload.accel.z_g,
            accel_mag_g  = payload.accel.magnitude_g,
            hr_bpm       = payload.heart_rate.bpm,
            hr_spo2      = payload.heart_rate.spo2,
            hr_valid     = payload.heart_rate.valid,
            latitude     = payload.gps.latitude,
            longitude    = payload.gps.longitude,
            speed_kmh    = payload.gps.speed_kmh,
            altitude     = payload.gps.altitude,
            satellites   = payload.gps.satellites,
            fix_valid    = payload.gps.fix_valid,
            utc_time     = payload.gps.utc_time,
            utc_date     = payload.gps.utc_date,
            raw_payload  = payload.model_dump_json(),
        )
        db.add(db_event)
        db.commit()
        db.refresh(db_event)

        # Broadcast to dashboard
        ws_payload = {
            "type": "EMERGENCY_EVENT",
            "event": {
                "id":          db_event.id,
                "device_id":   db_event.device_id,
                "alert_type":  db_event.alert_type.value if db_event.alert_type else "UNKNOWN",
                "received_at": db_event.received_at.isoformat() if db_event.received_at else "",
                "accel_mag_g": db_event.accel_mag_g,
                "hr_bpm":      db_event.hr_bpm,
                "hr_spo2":     db_event.hr_spo2,
                "latitude":    db_event.latitude,
                "longitude":   db_event.longitude,
                "speed_kmh":   db_event.speed_kmh,
                "fix_valid":   db_event.fix_valid,
                "acked":       db_event.acked,
            }
        }
        await manager.broadcast(ws_payload)

        return EmergencyEventOut(
            id          = db_event.id,
            device_id   = db_event.device_id,
            event_id    = db_event.event_id,
            alert_type  = db_event.alert_type.value if db_event.alert_type else "UNKNOWN",
            received_at = db_event.received_at.isoformat() if db_event.received_at else "",
            accel_mag_g = db_event.accel_mag_g or 0.0,
            hr_bpm      = db_event.hr_bpm or 0,
            hr_spo2     = db_event.hr_spo2 or 0,
            latitude    = db_event.latitude or 0.0,
            longitude   = db_event.longitude or 0.0,
            speed_kmh   = db_event.speed_kmh or 0.0,
            fix_valid   = db_event.fix_valid or False,
            acked       = db_event.acked or False,
        )
    finally:
        db.close()


@app.get("/api/events", response_model=List[EmergencyEventOut], tags=["Events"])
def list_events(
    skip: int = Query(0, ge=0),
    limit: int = Query(50, le=200),
    device_id: Optional[str] = None,
    alert_type: Optional[str] = None,
):
    """List all emergency events, optionally filtered."""
    db: Session = SessionLocal()
    try:
        q = db.query(EmergencyEventDB)
        if device_id:
            q = q.filter(EmergencyEventDB.device_id == device_id)
        if alert_type:
            q = q.filter(EmergencyEventDB.alert_type == alert_type)
        events = q.order_by(EmergencyEventDB.received_at.desc()).offset(skip).limit(limit).all()
        return [
            EmergencyEventOut(
                id=e.id, device_id=e.device_id or "", event_id=e.event_id or 0,
                alert_type=e.alert_type.value if e.alert_type else "UNKNOWN",
                received_at=e.received_at.isoformat() if e.received_at else "",
                accel_mag_g=e.accel_mag_g or 0.0, hr_bpm=e.hr_bpm or 0,
                hr_spo2=e.hr_spo2 or 0, latitude=e.latitude or 0.0,
                longitude=e.longitude or 0.0, speed_kmh=e.speed_kmh or 0.0,
                fix_valid=e.fix_valid or False, acked=e.acked or False,
            )
            for e in events
        ]
    finally:
        db.close()


@app.get("/api/events/{event_id}", tags=["Events"])
def get_event(event_id: int):
    """Get a single event by database ID."""
    db: Session = SessionLocal()
    try:
        e = db.query(EmergencyEventDB).filter(EmergencyEventDB.id == event_id).first()
        if not e:
            raise HTTPException(status_code=404, detail="Event not found")
        return json.loads(e.raw_payload or "{}")
    finally:
        db.close()


@app.post("/api/events/{event_id}/ack", tags=["Events"])
async def ack_event(event_id: int, acked_by: str = "dispatcher"):
    """Acknowledge an emergency event."""
    db: Session = SessionLocal()
    try:
        e = db.query(EmergencyEventDB).filter(EmergencyEventDB.id == event_id).first()
        if not e:
            raise HTTPException(status_code=404, detail="Event not found")
        e.acked    = True
        e.acked_by = acked_by
        db.commit()

        await manager.broadcast({"type": "ACK", "event_id": event_id, "acked_by": acked_by})
        return {"status": "acknowledged", "event_id": event_id}
    finally:
        db.close()


@app.get("/api/stats", response_model=StatsOut, tags=["Statistics"])
def get_stats():
    """Aggregate statistics across all events."""
    db: Session = SessionLocal()
    try:
        all_events = db.query(EmergencyEventDB).all()
        if not all_events:
            return StatsOut(
                total_events=0, crash_events=0, hr_alerts=0,
                manual_sos=0, unique_devices=0, avg_bpm=0.0, avg_magnitude=0.0
            )

        crash_types   = {AlertTypeEnum.CRASH, AlertTypeEnum.ROLLOVER, AlertTypeEnum.NO_GPS}
        hr_alert_types = {AlertTypeEnum.HR_LOW, AlertTypeEnum.HR_HIGH}

        return StatsOut(
            total_events   = len(all_events),
            crash_events   = sum(1 for e in all_events if e.alert_type in crash_types),
            hr_alerts      = sum(1 for e in all_events if e.alert_type in hr_alert_types),
            manual_sos     = sum(1 for e in all_events if e.alert_type == AlertTypeEnum.MANUAL_SOS),
            unique_devices = len({e.device_id for e in all_events}),
            avg_bpm        = sum(e.hr_bpm or 0 for e in all_events) / len(all_events),
            avg_magnitude  = sum(e.accel_mag_g or 0 for e in all_events) / len(all_events),
        )
    finally:
        db.close()


# ---------------------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------------------

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Keep alive — echo ping
            data = await websocket.receive_text()
            if data == "ping":
                await websocket.send_text("pong")
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# ---------------------------------------------------------------------------
# Static Files & Dashboard SPA Mounting
# ---------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DASHBOARD_DIR = os.path.join(BASE_DIR, "dashboard")

app.mount("/", StaticFiles(directory=DASHBOARD_DIR, html=True), name="dashboard")
