from __future__ import annotations

from pathlib import Path
import math
import subprocess
import sys
import threading
import time
from typing import Any

from core.interfaces import VehicleBackend, VehicleSpec
from core.logging import Log


class RflySimRealBackend(VehicleBackend):
    """Backend for a real PX4 vehicle controlled through PX4MavCtrlV4 in ROS."""

    def __init__(self, spec: VehicleSpec):
        """Store real-vehicle network and ROS/mocap configuration."""
        super().__init__(spec)
        self.mav: Any | None = None
        self.ip = str(spec.params.get("ip", "127.0.0.1"))
        self.port = int(spec.params.get("port", spec.params.get("Port", 15501)))
        self.mav_id = int(spec.params.get("mav_id", self.port - 15500))
        self.udp_mode = int(spec.params.get("udp_mode", 2))
        self.startup_delay_s = float(spec.params.get("startup_delay_s", 3.0))
        self.yaw_init_delay_s = float(spec.params.get("yaw_init_delay_s", 5.0))
        self.offboard_delay_s = float(spec.params.get("offboard_delay_s", 3.0))

        self.ros_config = dict(spec.params.get("ros", {}))
        self.mocap_config = dict(spec.params.get("mocap", {}))
        self.ros_enabled = bool(self.ros_config.get("enabled", True))
        self.mocap_enabled = bool(self.mocap_config.get("enabled", True))
        self.check_before_init = bool(self.mocap_config.get("check_status", True))
        self.send_vision_enabled = bool(self.mocap_config.get("send_vision", True))
        self.start_twist_thread = bool(self.mocap_config.get("start_twist_thread", False))
        self.pose_timeout_s = float(self.mocap_config.get("pose_timeout_s", 0.2))

        default_prefix = str(self.mocap_config.get("rig_prefix", "droneyee0"))
        default_pose = f"/vrpn_client_node/{default_prefix}{self.mav_id}/pose"
        default_twist = f"/vrpn_client_node/{default_prefix}{self.mav_id}/twist"
        self.pose_topic = str(self.mocap_config.get("pose_topic", default_pose))
        self.twist_topic = str(self.mocap_config.get("twist_topic", default_twist))

        self.vrpn_pos = [0.0, 0.0, 0.0]
        self.vrpn_vel = [0.0, 0.0, 0.0]
        self.vrpn_quat = [1.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.px4_yaw = 0.0
        self.offset_yaw = 0.0
        self.init_px4_yaw = 0.0
        self.ini_yaw_success = False
        self.battery = 0.0
        self.last_fault_mode = int(spec.params.get("default_fault_mode", 0))

        self._stop_event = threading.Event()
        self._pose_thread: threading.Thread | None = None
        self._twist_thread: threading.Thread | None = None
        self._rospy: Any | None = None
        self._PoseStamped: Any | None = None
        self._TwistStamped: Any | None = None
        self._Float32MultiArray: Any | None = None
        self._quat2euler: Any | None = None

    def initialize(self) -> None:
        """Connect MAVLink, initialize ROS/mocap loops, and enter Offboard mode."""
        utils_path = self.spec.params.get("rfly_utils_path")
        if utils_path:
            sys.path.insert(0, str(Path(utils_path).resolve()))
        try:
            import PX4MavCtrlV4 as PX4MavCtrler
        except ImportError as exc:
            raise ImportError(
                "Cannot import PX4MavCtrlV4. Set vehicle.rfly_utils_path to platform/sdk/rflysim_legacy."
            ) from exc

        if self.ros_enabled:
            self._load_ros_dependencies()
            self._init_ros_node()

        Log.info(self.spec.id, f"connecting real vehicle mav_id={self.mav_id} ip={self.ip} port={self.port}")
        self.mav = PX4MavCtrler.PX4MavCtrler(self.mav_id, self.ip, "Direct", self.port)

        if self.mocap_enabled:
            if self.check_before_init and not self.check_status():
                raise RuntimeError(f"{self.spec.id} ROS/mocap status check failed")
            self._start_mocap_threads()

        Log.info(self.spec.id, "starting MAVLink receive loop")
        self.mav.InitMavLoop(UDPMode=self.udp_mode)
        if self.startup_delay_s > 0:
            time.sleep(self.startup_delay_s)

        if self.mocap_enabled:
            self.init_yaw()
            if self.yaw_init_delay_s > 0:
                time.sleep(self.yaw_init_delay_s)

        Log.info(self.spec.id, "initializing Offboard mode")
        self.mav.initOffboard()
        if self.offboard_delay_s > 0:
            time.sleep(self.offboard_delay_s)
        Log.ok(self.spec.id, "real backend initialized")

    def shutdown(self) -> None:
        """Stop ROS helper loops and close the PX4MavCtrlV4 MAVLink loop."""
        self._stop_event.set()
        for thread in (self._pose_thread, self._twist_thread):
            if thread is not None:
                thread.join(timeout=1.0)
        if self.mav is not None and hasattr(self.mav, "endMavLoop"):
            self.mav.endMavLoop()
        Log.info(self.spec.id, "real backend stopped")

    def _load_ros_dependencies(self) -> None:
        """Import ROS-only modules lazily so non-ROS dry-runs stay portable."""
        try:
            import rospy
            from geometry_msgs.msg import PoseStamped, TwistStamped
            from std_msgs.msg import Float32MultiArray
            from transforms3d.euler import quat2euler
        except ImportError as exc:
            raise ImportError(
                "Real vehicle backend requires rospy, geometry_msgs, std_msgs, and transforms3d in ROS mode"
            ) from exc
        self._rospy = rospy
        self._PoseStamped = PoseStamped
        self._TwistStamped = TwistStamped
        self._Float32MultiArray = Float32MultiArray
        self._quat2euler = quat2euler

    def _init_ros_node(self) -> None:
        """Initialize the ROS node once and apply optional ROS parameters."""
        rospy = self._require_rospy()
        node_name = str(self.ros_config.get("node_name", "rflydt"))
        anonymous = bool(self.ros_config.get("anonymous", False))
        try:
            already_initialized = bool(rospy.core.is_initialized())
        except Exception:
            already_initialized = False
        if not already_initialized:
            rospy.init_node(node_name, anonymous=anonymous)

        for key, value in dict(self.ros_config.get("set_params", {})).items():
            rospy.set_param(str(key), value)

    def check_status(self) -> bool:
        """Check that the VRPN pose topic exists and the aircraft IP is reachable."""
        if self._rospy is None:
            return True
        try:
            result = subprocess.run(
                ["rostopic", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            topics = result.stdout.decode("utf-8", errors="ignore")
        except Exception as exc:
            Log.error(self.spec.id, f"failed to list ROS topics: {exc}")
            return False

        if self.pose_topic not in topics:
            Log.error(self.spec.id, f"pose topic not found: {self.pose_topic}")
            return False

        ping_cmd = ["ping", "-c", "1", self.ip]
        try:
            ping = subprocess.run(ping_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        except Exception as exc:
            Log.error(self.spec.id, f"failed to ping {self.ip}: {exc}")
            return False
        if ping.returncode != 0:
            Log.error(self.spec.id, f"{self.ip} unreachable")
            return False
        return True

    def init_yaw(self) -> None:
        """Capture the initial yaw offset between mocap and PX4 estimates."""
        mav = self._require_mav()
        self.px4_yaw = float(getattr(mav, "uavAngEular", [0.0, 0.0, 0.0])[2])
        self.offset_yaw = self.yaw - self.px4_yaw
        self.init_px4_yaw = self.px4_yaw
        self.ini_yaw_success = True

    def update_pose(self) -> None:
        """Receive VRPN pose messages and forward vision-position estimates to PX4."""
        rospy = self._require_rospy()
        PoseStamped = self._PoseStamped
        quat2euler = self._quat2euler
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(self.pose_topic, PoseStamped, timeout=self.pose_timeout_s)
            except Exception:
                continue
            self.vrpn_pos[0] = float(msg.pose.position.x)
            self.vrpn_pos[1] = float(msg.pose.position.y)
            self.vrpn_pos[2] = float(msg.pose.position.z)
            self.vrpn_quat[0] = float(msg.pose.orientation.w)
            self.vrpn_quat[1] = float(msg.pose.orientation.x)
            self.vrpn_quat[2] = float(msg.pose.orientation.y)
            self.vrpn_quat[3] = float(msg.pose.orientation.z)
            _, _, self.yaw = quat2euler(self.vrpn_quat)
            if self.mav is not None:
                self.px4_yaw = float(getattr(self.mav, "uavAngEular", [0.0, 0.0, 0.0])[2])
                self.battery = float(getattr(self.mav, "batInfo", [0.0])[0])
            if self.send_vision_enabled:
                self.send_vision_capture()

    def update_twist(self) -> None:
        """Receive optional VRPN twist messages for velocity monitoring."""
        rospy = self._require_rospy()
        TwistStamped = self._TwistStamped
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(self.twist_topic, TwistStamped, timeout=self.pose_timeout_s)
            except Exception:
                continue
            self.vrpn_vel[0] = float(msg.twist.linear.x)
            self.vrpn_vel[1] = float(msg.twist.linear.y)
            self.vrpn_vel[2] = float(msg.twist.linear.z)

    def send_vision_capture(self) -> None:
        """Transform mocap pose into PX4 vision-position coordinates and send it."""
        if not self.ini_yaw_success or self.mav is None:
            return
        yaw = self._limit_yaw(-self.offset_yaw - self.yaw)
        yaw_err = self.offset_yaw
        x = self.vrpn_pos[0] * math.cos(yaw_err) - self.vrpn_pos[1] * math.sin(yaw_err)
        y = -self.vrpn_pos[1] * math.cos(yaw_err) - self.vrpn_pos[0] * math.sin(yaw_err)
        z = -self.vrpn_pos[2]
        self.mav.send_vision_position(x, y, z, yaw)

    def _start_mocap_threads(self) -> None:
        """Start background ROS subscribers used by the real vehicle."""
        self._stop_event.clear()
        self._pose_thread = threading.Thread(target=self.update_pose, name=f"{self.spec.id}-pose", daemon=True)
        self._pose_thread.start()
        if self.start_twist_thread:
            self._twist_thread = threading.Thread(target=self.update_twist, name=f"{self.spec.id}-twist", daemon=True)
            self._twist_thread.start()

    def _require_mav(self) -> Any:
        """Return the active PX4MavCtrlV4 controller or raise a clear error."""
        if self.mav is None:
            raise RuntimeError(f"{self.spec.id} is not initialized")
        return self.mav

    def _require_rospy(self) -> Any:
        """Return the lazy-loaded rospy module."""
        if self._rospy is None:
            raise RuntimeError(f"{self.spec.id} ROS dependencies are not loaded")
        return self._rospy

    @staticmethod
    def _limit_yaw(yaw: float) -> float:
        """Wrap yaw to [-pi, pi]."""
        if math.fabs(yaw) > math.pi:
            return yaw - 2 * math.pi if yaw > 0 else 2 * math.pi + yaw
        return yaw

    def arm(self, **_: Any) -> None:
        """Arm the real vehicle and mark the arm phase in the PX4 log."""
        mav = self._require_mav()
        mav.SendMavArm(1)
        mav.SendMavCmdLong(183, 2, 1, 0, 0, 0, 0, 666)

    def disarm(self, **_: Any) -> None:
        """Disarm the real vehicle."""
        self._require_mav().SendMavArm(0)

    def goto(self, pos_ned: list[float], yaw: float = 0.0, **_: Any) -> None:
        """Send a position setpoint in local NED coordinates."""
        mav = self._require_mav()
        mav.SendPosNED(float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]), float(yaw))
        mav.SendMavCmdLong(183, 2, 4, float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2]), float(yaw), 666)

    def takeoff(self, pos_ned: list[float], **_: Any) -> None:
        """Send the initial takeoff setpoint."""
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
        """Command a local-position landing."""
        mav = self._require_mav()
        pos = list(getattr(mav, "uavPosNED", [0.0, 0.0, 0.0]))
        mav.sendMavLand(pos[0], pos[1], 0)
        mav.SendMavCmdLong(183, 2, 7, 0, 0, 0, 0, 666)

    def inject_fault(self, mode: int, flag: int, params: list[float] | None = None, **_: Any) -> None:
        """Send a real-aircraft Rfly fault-injection HIL control message."""
        mav = self._require_mav()
        self.last_fault_mode = int(mode)
        ctrls = [0.0] * 16
        for i, value in enumerate(params or []):
            if i >= len(ctrls):
                break
            ctrls[i] = float(value)
        mav.SendHILCtrlMsg(int(mode), int(flag), ctrls)
        mav.SendMavCmdLong(183, 2, 9, int(mode), int(flag), 0, 0, 666)

    def clear_fault(self, mode: int | None = None, **_: Any) -> None:
        """Clear the real-aircraft fault-injection flag and zero all controls."""
        fault_mode = self.last_fault_mode if mode is None else int(mode)
        mav = self._require_mav()
        mav.SendHILCtrlMsg(int(fault_mode), 0, [0.0] * 16)
        mav.SendMavCmdLong(183, 2, 10, int(fault_mode), 0, 0, 0, 666)

    def mark_phase(self, phase_id: int, *params: Any, **_: Any) -> None:
        """Write a semantic phase marker into the PX4 offline log."""
        padded = list(params[:5]) + [0] * max(0, 5 - len(params))
        self._require_mav().SendMavCmdLong(183, 2, int(phase_id), *padded[:5])

    def run_mission(self, trajectory: list[list[float]], period_s: float = 0.1, **_: Any) -> None:
        """Stream a list of position setpoints as a trajectory mission."""
        for point in trajectory:
            self.goto(point)
            time.sleep(period_s)

    def snapshot(self) -> dict[str, Any]:
        """Return latest real-vehicle state for reports."""
        snap = super().snapshot()
        if self.mav is not None:
            snap.update({
                "pos_ned": list(getattr(self.mav, "uavPosNED", [])),
                "vel_ned": list(getattr(self.mav, "uavVelNED", [])),
                "acc_b": list(getattr(self.mav, "uavAccB", [])),
                "vrpn_pos": list(self.vrpn_pos),
                "vrpn_vel": list(self.vrpn_vel),
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
