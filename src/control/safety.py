"""Safety layer: reads the range sensors and limits speed near obstacles."""
from dataclasses import dataclass
from src.control.controller import wrap
from src.utils import config as cfg


@dataclass
class SafetyStatus:
    emergency: bool
    speed_cap: float
    nearest: float


class SafetyMonitor:
    def evaluate(self, readings, heading, speed):
        front = [d for ang, d in readings if abs(wrap(ang - heading)) <= cfg.SAFETY_CONE]
        nearest = min(front) if front else cfg.RAY_MAX

        if nearest < cfg.SAFETY_HALT:
            cap = 0.0
        elif nearest <= cfg.SAFETY_STOP:
            cap = cfg.CREEP_SPEED
        elif nearest >= cfg.SAFETY_SLOW:
            cap = cfg.MAX_SPEED
        else:   # linear ramp between creep speed and cruise speed
            frac = (nearest - cfg.SAFETY_STOP) / (cfg.SAFETY_SLOW - cfg.SAFETY_STOP)
            cap = cfg.CREEP_SPEED + frac * (cfg.CRUISE_SPEED - cfg.CREEP_SPEED)

        emergency = nearest <= cfg.SAFETY_STOP and speed > cfg.CREEP_SPEED + 10
        return SafetyStatus(emergency, cap, nearest)