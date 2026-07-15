from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass
class VehicleSpec:
    id: str
    backend: str
    vehicle_id: int
    role: str = "vehicle"
    params: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "VehicleSpec":
        """Parse one vehicle entry from a run config."""
        return VehicleSpec(
            id=str(data["id"]),
            backend=str(data["backend"]),
            vehicle_id=int(data.get("vehicle_id", data.get("copter_id", 1))),
            role=str(data.get("role", "vehicle")),
            params={k: v for k, v in data.items() if k not in {"id", "backend", "vehicle_id", "copter_id", "role"}},
        )


class VehicleBackend(ABC):
    def __init__(self, spec: VehicleSpec):
        """Store the normalized vehicle specification."""
        self.spec = spec

    @abstractmethod
    def initialize(self) -> None:
        """Connect or start the underlying vehicle backend."""
        raise NotImplementedError

    @abstractmethod
    def shutdown(self) -> None:
        """Release backend resources and stop background loops."""
        raise NotImplementedError

    def arm(self, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support arm")

    def disarm(self, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support disarm")

    def takeoff(self, pos_ned: list[float], **_: Any) -> None:
        self.goto(pos_ned=pos_ned)

    def goto(self, pos_ned: list[float], yaw: float = 0.0, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support goto")

    def velocity(self, vel_ned: list[float], yaw_rate: float = 0.0, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support velocity")

    def hover(self, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support hover")

    def land(self, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support land")

    def inject_fault(self, mode: int, flag: int, params: list[float] | None = None, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support inject_fault")

    def clear_fault(self, mode: int = 0, **_: Any) -> None:
        """Clear any active fault injection on the vehicle."""
        self.inject_fault(mode=mode, flag=0, params=[0.0] * 16)

    def mark_phase(self, phase_id: int, *params: Any, **_: Any) -> None:
        """Optionally write a semantic phase marker into the vehicle log."""
        return None

    def switch_virtual_redundancy(self, source: str = "virtual_imu", enabled: bool = True, **_: Any) -> None:
        raise NotImplementedError(f"{self.spec.id} does not support switch_virtual_redundancy")

    def run_mission(self, trajectory: list[list[float]], period_s: float = 0.1, **_: Any) -> None:
        """Default mission behavior: send each setpoint once."""
        for point in trajectory:
            self.goto(pos_ned=point)

    def snapshot(self) -> dict[str, Any]:
        """Return backend identity fields for the report."""
        return {"id": self.spec.id, "backend": self.spec.backend, "vehicle_id": self.spec.vehicle_id}

    def record_snapshot(self, fields: list[str]) -> dict[str, Any]:
        """Return selected real-time fields for the data recorder."""
        data = self.snapshot()
        for field in fields:
            data.setdefault(field, None)
        return data
