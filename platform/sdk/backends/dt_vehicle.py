from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Any

from core.interfaces import VehicleBackend, VehicleSpec
from core.logging import Log


class RflySimDTBackend(VehicleBackend):
    """Backend for a vehicle running in the full RflySim/SITL toolchain."""

    def __init__(self, spec: VehicleSpec):
        """Store connection options for one full RflySim/SITL vehicle."""
        super().__init__(spec)
        self.mav: Any | None = None
        self.ip = str(spec.params.get("ip", "127.0.0.1"))
        self.udp_mode = int(spec.params.get("udp_mode", 2))
        self.init_true_data = bool(spec.params.get("init_true_data", True))
        self.init_offboard = bool(spec.params.get("init_offboard", True))
        self.startup_delay_s = float(spec.params.get("startup_delay_s", 2.0))
        self.offboard_delay_s = float(spec.params.get("offboard_delay_s", 0.0))

    def initialize(self) -> None:
        """Connect PX4MavCtrlV4 to an already launched CopterSim/PX4 vehicle."""
        utils_path = self.spec.params.get("rfly_utils_path")
        if utils_path:
            sys.path.insert(0, str(Path(utils_path).resolve()))
        try:
            import PX4MavCtrlV4 as PX4MavCtrler
        except ImportError as exc:
            raise ImportError(
                "Cannot import PX4MavCtrlV4. Set vehicle.rfly_utils_path to the RflySim utils folder."
            ) from exc

        Log.info(self.spec.id, f"connecting DT vehicle {self.spec.vehicle_id} at {self.ip}")
        self.mav = PX4MavCtrler.PX4MavCtrler(self.spec.vehicle_id, self.ip)
        self.mav.InitMavLoop(self.udp_mode)
        if self.init_true_data:
            self.mav.InitTrueDataLoop()
        if self.startup_delay_s > 0:
            time.sleep(self.startup_delay_s)
        if self.init_offboard:
            self.mav.initOffboard()
            if self.offboard_delay_s > 0:
                time.sleep(self.offboard_delay_s)
        Log.ok(self.spec.id, "DT backend initialized")

    def shutdown(self) -> None:
        """Stop the MAVLink receive/control loops for this vehicle."""
        if self.mav is not None and hasattr(self.mav, "endMavLoop"):
            self.mav.endMavLoop()
        Log.info(self.spec.id, "DT backend stopped")

    def _require_mav(self) -> Any:
        """Return the active PX4MavCtrlV4 controller or raise a clear error."""
        if self.mav is None:
            raise RuntimeError(f"{self.spec.id} is not initialized")
        return self.mav

    def arm(self, **_: Any) -> None:
        """Arm the vehicle and broadcast a semantic phase marker."""
        mav = self._require_mav()
        mav.SendMavArm(1)
        mav.SendMavCmdLong(183, 2, 1, 0, 0, 0, 0, 666)

    def disarm(self, **_: Any) -> None:
        """Disarm the vehicle."""
        self._require_mav().SendMavArm(0)

    def goto(self, pos_ned: list[float], yaw: float = 0.0, **_: Any) -> None:
        """Send a position setpoint in local NED coordinates."""
        mav = self._require_mav()
        mav.SendPosNED(float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]), float(yaw))
        mav.SendMavCmdLong(183, 2, 4, float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]), float(yaw), 666)

    def takeoff(self, pos_ned: list[float], **_: Any) -> None:
        """Send the initial takeoff position setpoint."""
        mav = self._require_mav()
        mav.SendPosNED(float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]))
        mav.SendMavCmdLong(183, 2, 3, float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]), 0, 666)

    def velocity(self, vel_ned: list[float], yaw_rate: float = 0.0, **_: Any) -> None:
        """Send a velocity setpoint in local NED coordinates."""
        self._require_mav().SendVelNED(float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2]), float(yaw_rate))

    def hover(self, **_: Any) -> None:
        """Hold the current estimated local position."""
        mav = self._require_mav()
        pos = list(getattr(mav, "uavPosNED", [0.0, 0.0, -1.0]))
        mav.SendPosNED(pos[0], pos[1], pos[2])
        mav.SendMavCmdLong(183, 2, 6, 0, 0, 0, 0, 666)

    def land(self, **_: Any) -> None:
        """Command a local-position landing through PX4MavCtrlV4."""
        mav = self._require_mav()
        pos = list(getattr(mav, "uavPosNED", [0.0, 0.0, 0.0]))
        mav.sendMavLand(pos[0], pos[1], 0)
        mav.SendMavCmdLong(183, 2, 7, 0, 0, 0, 0, 666)

    def inject_fault(self, mode: int, flag: int, params: list[float] | None = None, **_: Any) -> None:
        """Send an Rfly HIL control/fault injection message to PX4."""
        mav = self._require_mav()
        ctrls = [0.0] * 16
        for i, value in enumerate(params or []):
            if i >= len(ctrls):
                break
            ctrls[i] = float(value)
        mav.SendHILCtrlMsg(int(mode), int(flag), ctrls)
        mav.SendMavCmdLong(183, 2, 9, int(mode), int(flag), 0, 0, 666)

    def clear_fault(self, mode: int = 0, **_: Any) -> None:
        """Clear the Rfly HIL fault-injection flag."""
        mav = self._require_mav()
        mav.SendHILCtrlMsg(int(mode), 0, [0.0] * 16)
        mav.SendMavCmdLong(183, 2, 10, int(mode), 0, 0, 0, 666)

    def mark_phase(self, phase_id: int, *params: Any, **_: Any) -> None:
        """Write a phase marker to the PX4 log via MAV_CMD_USER_1."""
        padded = list(params[:5]) + [0] * max(0, 5 - len(params))
        self._require_mav().SendMavCmdLong(183, 2, int(phase_id), *padded[:5])

    def switch_virtual_redundancy(self, source: str = "virtual_imu", enabled: bool = True, **_: Any) -> None:
        """Switch a ROS parameter that downstream virtual-redundancy logic can watch."""
        try:
            import rospy
        except ImportError as exc:
            raise ImportError("switch_virtual_redundancy requires rospy in DT mode") from exc
        rospy.set_param("use_imu_source", bool(enabled))
        rospy.set_param("imu_source_name", source)

    def run_mission(self, trajectory: list[list[float]], period_s: float = 0.1, **_: Any) -> None:
        """Stream a list of position setpoints as a trajectory mission."""
        for point in trajectory:
            self.goto(point)
            time.sleep(period_s)

    def snapshot(self) -> dict[str, Any]:
        """Return latest estimated state fields for reports and metrics."""
        snap = super().snapshot()
        if self.mav is not None:
            snap.update({
                "pos_ned": list(getattr(self.mav, "uavPosNED", [])),
                "vel_ned": list(getattr(self.mav, "uavVelNED", [])),
                "acc_b": list(getattr(self.mav, "uavAccB", [])),
            })
        return snap

    def record_snapshot(self, fields: list[str]) -> dict[str, Any]:
        """Read selected PX4MavCtrlV4 state attributes for real-time logging."""
        data = super().snapshot()
        mav = self.mav
        for field in fields:
            value = getattr(mav, field, None) if mav is not None else None
            data[field] = list(value) if isinstance(value, (list, tuple)) else value
        return data
