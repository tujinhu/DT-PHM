from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Callable


SDK_CTRL = Path(r"C:\PX4PSP\RflySimAPIs\RflySimSDK\ctrl")
SDK_UE = Path(r"C:\PX4PSP\RflySimAPIs\RflySimSDK\ue")
DEFAULT_SITL_BAT = Path(r"C:\PX4PSP\RflySimAPIs\SITLRun.bat")


def add_rflysim_paths() -> None:
    """Make the official RflySim Python APIs importable from this standalone test."""
    for path in (SDK_CTRL, SDK_UE, Path.cwd()):
        if path.exists():
            sys.path.insert(0, str(path))


def norm3(values: list[float]) -> float:
    """Return the Euclidean norm of a 3D vector."""
    return float((values[0] ** 2 + values[1] ** 2 + values[2] ** 2) ** 0.5)


def vec_sub(a: list[float], b: list[float]) -> list[float]:
    """Subtract two 3D vectors."""
    return [float(a[i]) - float(b[i]) for i in range(3)]


@dataclass
class VehicleState:
    pos: list[float]
    vel: list[float]
    euler: list[float]
    sim_time: float | None = None


@dataclass
class PhaseResult:
    name: str
    metric: str
    dt_response_s: float | None
    dt_final: dict[str, list[float]]


class DtOnlyResponseTester:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.dt = None
        self.sitl_process: subprocess.Popen[str] | None = None
        self.samples: list[dict[str, float | str | int | None]] = []
        self.results: list[PhaseResult] = []
        self.started_at = time.time()

    def log(self, message: str) -> None:
        """Print one timestamped console line."""
        print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)

    def maybe_start_sitl(self) -> None:
        """Optionally start the full RflySim SITL batch for the DT vehicle."""
        if not self.args.start_sitl:
            self.log("SITL autostart disabled. Ensure SITLRun.bat is already running with VehicleNum=1.")
            return
        bat = Path(self.args.sitl_bat).resolve()
        if not bat.exists():
            raise FileNotFoundError(f"SITL batch not found: {bat}")
        self.log(f"Starting SITL batch: {bat}")
        self.sitl_process = subprocess.Popen(["cmd.exe", "/c", str(bat)], cwd=str(bat.parent))
        self.log(f"Waiting {self.args.startup_wait_s:.1f}s for SITL startup")
        time.sleep(self.args.startup_wait_s)

    def initialize(self) -> None:
        """Create the DT controller connected to the running SITL toolchain."""
        add_rflysim_paths()
        import PX4MavCtrlV4 as PX4MavCtrl

        self.maybe_start_sitl()

        self.log(f"Connecting DT vehicle id={self.args.dt_id} UDPMode={self.args.dt_udp_mode}")
        self.dt = PX4MavCtrl.PX4MavCtrler(self.args.dt_id, self.args.ip)
        self.dt.InitMavLoop(UDPMode=self.args.dt_udp_mode)
        if self.args.dt_true_data:
            self.dt.InitTrueDataLoop()
        time.sleep(self.args.dt_warmup_s)
        self.dt.initOffboard()
        self.dt.SendMavArm(1)
        self.started_at = time.time()
        self.log("DT initialization finished")

    def read_dt(self) -> VehicleState:
        """Read the latest DT/SITL estimated NED state from PX4MavCtrlV4."""
        return VehicleState(
            pos=[float(x) for x in getattr(self.dt, "uavPosNED", [0.0, 0.0, 0.0])],
            vel=[float(x) for x in getattr(self.dt, "uavVelNED", [0.0, 0.0, 0.0])],
            euler=[float(x) for x in getattr(self.dt, "uavAngEular", [0.0, 0.0, 0.0])],
            sim_time=float(getattr(self.dt, "uavTimeStmp", 0.0)),
        )

    def append_sample(self, phase: str, phase_start: float) -> dict[str, float | str | int | None]:
        """Sample the DT vehicle once and store a row for later CSV export."""
        now = time.time()
        state = self.read_dt()
        row: dict[str, float | str | int | None] = {
            "sample_index": len(self.samples) + 1,
            "wall_elapsed_s": now - self.started_at,
            "phase": phase,
            "phase_elapsed_s": now - phase_start,
            "dt_sim_time": state.sim_time,
            "dt_x": state.pos[0],
            "dt_y": state.pos[1],
            "dt_z": state.pos[2],
            "dt_vx": state.vel[0],
            "dt_vy": state.vel[1],
            "dt_vz": state.vel[2],
            "dt_roll": state.euler[0],
            "dt_pitch": state.euler[1],
            "dt_yaw": state.euler[2],
        }
        self.samples.append(row)
        return row

    def first_hit(
        self,
        rows: list[dict[str, float | str | int | None]],
        predicate: Callable[[dict[str, float | str | int | None]], bool],
    ) -> float | None:
        """Return first phase time where the predicate stays true for N consecutive samples."""
        streak = 0
        first_time = None
        for row in rows:
            if predicate(row):
                streak += 1
                if first_time is None:
                    first_time = float(row["phase_elapsed_s"])
                if streak >= self.args.consecutive_hits:
                    return first_time
            else:
                streak = 0
                first_time = None
        return None

    def run_phase(
        self,
        name: str,
        duration_s: float,
        issue_command: Callable[[], dict[str, object]],
        dt_hit: Callable[[dict[str, float | str | int | None]], bool],
        metric: str,
    ) -> None:
        """Issue one DT command, sample response, and calculate response time."""
        self.log(f"Phase {name}: {metric}")
        command_meta = issue_command()
        phase_start = time.time()
        rows: list[dict[str, float | str | int | None]] = []
        interval = 1.0 / self.args.sample_hz
        next_t = time.time()
        while time.time() - phase_start < duration_s:
            next_t += interval
            rows.append(self.append_sample(name, phase_start))
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.time()

        dt_response = self.first_hit(rows, dt_hit)
        last = rows[-1]
        result = PhaseResult(
            name=name,
            metric=metric,
            dt_response_s=dt_response,
            dt_final={
                "pos": [float(last["dt_x"]), float(last["dt_y"]), float(last["dt_z"])],
                "vel": [float(last["dt_vx"]), float(last["dt_vy"]), float(last["dt_vz"])],
            },
        )
        self.results.append(result)
        self.log(f"{name}: dt={dt_response} s, command={command_meta}")

    def run(self) -> None:
        """Run the standard DT response-test control sequence."""
        self.initialize()
        try:
            pos_tol = self.args.pos_tol
            vel_tol = self.args.vel_tol
            hover_speed_tol = self.args.hover_speed_tol

            def takeoff_cmd() -> dict[str, object]:
                target = [0.0, 0.0, self.args.takeoff_z]
                self.dt.SendPosNED(*target, 0.0)
                return {"target": target}

            self.run_phase(
                "takeoff_pos",
                self.args.takeoff_duration_s,
                takeoff_cmd,
                lambda r: abs(float(r["dt_z"]) - self.args.takeoff_z) <= pos_tol,
                f"first |z - {self.args.takeoff_z}| <= {pos_tol}",
            )

            def vel_cmd() -> dict[str, object]:
                target = [self.args.forward_vx, 0.0, 0.0]
                self.dt.SendVelNED(*target, 0.0)
                return {"target": target}

            self.run_phase(
                "forward_vel",
                self.args.velocity_duration_s,
                vel_cmd,
                lambda r: abs(float(r["dt_vx"]) - self.args.forward_vx) <= vel_tol,
                f"first |vx - {self.args.forward_vx}| <= {vel_tol}",
            )

            def hover_cmd() -> dict[str, object]:
                dt_pos = self.read_dt().pos
                self.dt.SendPosNED(dt_pos[0], dt_pos[1], dt_pos[2], 0.0)
                hover_cmd.dt_pos = dt_pos
                return {"dt_hold": dt_pos}

            hover_cmd.dt_pos = [0.0, 0.0, 0.0]
            self.run_phase(
                "hover_pos",
                self.args.hover_duration_s,
                hover_cmd,
                lambda r: norm3([float(r["dt_vx"]), float(r["dt_vy"]), float(r["dt_vz"])]) <= hover_speed_tol
                and norm3(vec_sub([float(r["dt_x"]), float(r["dt_y"]), float(r["dt_z"])], hover_cmd.dt_pos)) <= pos_tol,
                f"first speed <= {hover_speed_tol} and hold position error <= {pos_tol}",
            )

            def land_cmd() -> dict[str, object]:
                dt_pos = self.read_dt().pos
                self.dt.SendPosNED(dt_pos[0], dt_pos[1], 0.0, 0.0)
                return {"dt_target": [dt_pos[0], dt_pos[1], 0.0]}

            self.run_phase(
                "land_pos",
                self.args.land_duration_s,
                land_cmd,
                lambda r: float(r["dt_z"]) >= -pos_tol,
                f"first z >= -{pos_tol}",
            )
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Stop Python-side DT control/listening loops without terminating SITL."""
        self.log("Shutting down DT controller")
        try:
            if self.dt is not None:
                self.dt.SendVelNED(0, 0, 0, 0)
                time.sleep(0.2)
                self.dt.endMavLoop()
        except Exception as exc:
            self.log(f"DT shutdown warning: {exc}")

    def export(self) -> tuple[Path, Path]:
        """Write raw samples and response summary files."""
        out_dir = Path(self.args.output_dir).resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sample_path = out_dir / f"dt_only_response_{stamp}_samples.csv"
        summary_path = out_dir / f"dt_only_response_{stamp}_summary.json"

        if self.samples:
            with sample_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()))
                writer.writeheader()
                writer.writerows(self.samples)
        summary = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "description": "Single SITL/DT response test using the standard response-test control sequence.",
            "args": vars(self.args),
            "results": [
                {
                    "name": r.name,
                    "metric": r.metric,
                    "dt_response_s": r.dt_response_s,
                    "dt_final": r.dt_final,
                }
                for r in self.results
            ],
        }
        with summary_path.open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        return sample_path, summary_path


def build_arg_parser() -> argparse.ArgumentParser:
    """Create CLI arguments for repeatable DT-only response tests."""
    parser = argparse.ArgumentParser(description="Run SITL/DT-only response test.")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--dt-id", type=int, default=1)
    parser.add_argument("--dt-udp-mode", type=int, default=2)
    parser.add_argument("--dt-true-data", action="store_true")
    parser.add_argument("--dt-warmup-s", type=float, default=2.0)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--consecutive-hits", type=int, default=3)
    parser.add_argument("--pos-tol", type=float, default=0.15)
    parser.add_argument("--vel-tol", type=float, default=0.15)
    parser.add_argument("--hover-speed-tol", type=float, default=0.15)
    parser.add_argument("--takeoff-z", type=float, default=-1.0)
    parser.add_argument("--forward-vx", type=float, default=1.0)
    parser.add_argument("--takeoff-duration-s", type=float, default=10.0)
    parser.add_argument("--velocity-duration-s", type=float, default=6.0)
    parser.add_argument("--hover-duration-s", type=float, default=5.0)
    parser.add_argument("--land-duration-s", type=float, default=8.0)
    parser.add_argument("--output-dir", default="log")
    parser.add_argument("--start-sitl", action="store_true", help="Start SITLRun.bat before connecting DT.")
    parser.add_argument("--sitl-bat", default=str(DEFAULT_SITL_BAT))
    parser.add_argument("--startup-wait-s", type=float, default=35.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    tester = DtOnlyResponseTester(args)
    try:
        tester.run()
    finally:
        sample_path, summary_path = tester.export()
        print(f"Samples: {sample_path}")
        print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
