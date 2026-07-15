from __future__ import annotations

from typing import Any

from core.interfaces import VehicleBackend, VehicleSpec
from core.logging import Log


class DryRunVehicleBackend(VehicleBackend):
    def __init__(self, spec: VehicleSpec):
        """Create an in-memory fake vehicle for scenario validation."""
        super().__init__(spec)
        self.state = {"pos_ned": [0.0, 0.0, 0.0], "armed": False, "landed": True}

    def _log(self, action: str, payload: dict[str, Any] | None = None) -> None:
        """Print a dry-run action with optional structured payload."""
        suffix = f" {payload}" if payload else ""
        Log.info(self.spec.id, f"dry-run {action}{suffix}")

    def initialize(self) -> None:
        """Pretend to initialize a backend without opening sockets."""
        self._log("initialize", {"backend": self.spec.backend, "vehicle_id": self.spec.vehicle_id})

    def shutdown(self) -> None:
        """Pretend to shutdown a backend."""
        self._log("shutdown")

    def arm(self, **_: Any) -> None:
        """Mark the fake vehicle as armed."""
        self.state["armed"] = True
        self._log("arm")

    def disarm(self, **_: Any) -> None:
        """Mark the fake vehicle as disarmed."""
        self.state["armed"] = False
        self._log("disarm")

    def goto(self, pos_ned: list[float], yaw: float = 0.0, **_: Any) -> None:
        """Update the fake vehicle position."""
        self.state["pos_ned"] = [float(v) for v in pos_ned]
        self.state["landed"] = False
        self._log("goto", {"pos_ned": self.state["pos_ned"], "yaw": yaw})

    def velocity(self, vel_ned: list[float], yaw_rate: float = 0.0, **_: Any) -> None:
        """Log a velocity command without integrating dynamics."""
        self._log("velocity", {"vel_ned": vel_ned, "yaw_rate": yaw_rate})

    def hover(self, **_: Any) -> None:
        """Log a hold-position command."""
        self._log("hover", {"pos_ned": self.state["pos_ned"]})

    def land(self, **_: Any) -> None:
        """Mark the fake vehicle as landed."""
        self.state["landed"] = True
        self._log("land")

    def inject_fault(self, mode: int, flag: int, params: list[float] | None = None, **_: Any) -> None:
        """Log a semantic fault injection command."""
        self._log("inject_fault", {"mode": mode, "flag": flag, "params": params or []})

    def clear_fault(self, mode: int = 0, **_: Any) -> None:
        """Log a fault-clear command."""
        self._log("clear_fault", {"mode": mode})

    def mark_phase(self, phase_id: int, *params: Any, **_: Any) -> None:
        """Log an offline-log phase marker."""
        self._log("mark_phase", {"phase_id": phase_id, "params": list(params)})

    def switch_virtual_redundancy(self, source: str = "virtual_imu", enabled: bool = True, **_: Any) -> None:
        """Log a virtual-redundancy source switch."""
        self._log("switch_virtual_redundancy", {"source": source, "enabled": enabled})

    def run_mission(self, trajectory: list[list[float]], period_s: float = 0.1, **_: Any) -> None:
        """Move the fake vehicle directly to the final trajectory point."""
        if trajectory:
            self.state["pos_ned"] = [float(v) for v in trajectory[-1]]
        self._log("run_mission", {"points": len(trajectory), "period_s": period_s})

    def snapshot(self) -> dict[str, Any]:
        """Return fake state for report generation."""
        snap = super().snapshot()
        snap.update(self.state)
        return snap

    def record_snapshot(self, fields: list[str]) -> dict[str, Any]:
        """Return fake values for recorder dry-run validation."""
        data = super().snapshot()
        for field in fields:
            if field == "uavPosNED":
                data[field] = list(self.state["pos_ned"])
            else:
                data[field] = None
        return data
