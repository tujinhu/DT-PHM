from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import threading
import time
from typing import Any

from core.interfaces import VehicleBackend, VehicleSpec
from core.logging import Log, RealtimeDataRecorder
from core.scenario import Scenario, load_json
from core.toolchain import ToolchainProcess
from core.trajectory import generate_trajectory


class VerificationRunner:
    def __init__(
        self,
        config: dict[str, Any],
        scenario: Scenario,
        dry_run: bool = False,
        base_dir: str | Path | None = None,
    ):
        """Bind one semantic scenario to one concrete verification topology."""
        self.config = config
        self.scenario = scenario
        self.dry_run = dry_run
        self.base_dir = Path(base_dir or Path.cwd()).resolve()
        self.toolchain = ToolchainProcess(config.get("toolchain"), dry_run=dry_run)
        self.vehicles: dict[str, VehicleBackend] = {}
        self.events: list[dict[str, Any]] = []
        self.recorder: RealtimeDataRecorder | None = None
        self.data_log_path: Path | None = None

    @staticmethod
    def from_config_file(
        config_path: str | Path,
        scenario_path: str | Path | None = None,
        dry_run: bool = False,
    ) -> "VerificationRunner":
        """Create a runner from the config path selected by the entry script."""
        cfg_path = Path(config_path).resolve()
        config = load_json(cfg_path)
        VerificationRunner._resolve_config_paths(config, cfg_path.parent)
        scenario_ref = scenario_path or config.get("scenario")
        if scenario_ref is not None:
            scenario_file = Path(scenario_ref)
            if not scenario_file.is_absolute():
                scenario_file = (cfg_path.parent / scenario_file).resolve()
            scenario = Scenario.load(scenario_file)
        else:
            scenario = Scenario.from_dict(config, source=cfg_path)
        return VerificationRunner(config, scenario, dry_run=dry_run, base_dir=cfg_path.parent)

    @staticmethod
    def _resolve_config_paths(config: dict[str, Any], config_dir: Path) -> None:
        """Normalize path-like config entries relative to the config file."""
        def resolve_in_place(container: dict[str, Any], key: str) -> None:
            value = container.get(key)
            if not isinstance(value, str) or not value:
                return
            path = Path(value)
            if not path.is_absolute():
                container[key] = str((config_dir / path).resolve())

        toolchain = config.get("toolchain")
        if isinstance(toolchain, dict):
            resolve_in_place(toolchain, "bat_path")

        for vehicle in config.get("vehicles", []):
            if not isinstance(vehicle, dict):
                continue
            resolve_in_place(vehicle, "rfly_utils_path")

    def build_vehicles(self) -> None:
        """Instantiate every vehicle backend declared by the run config."""
        for vehicle_data in self.config.get("vehicles", []):
            spec = VehicleSpec.from_dict(vehicle_data)
            self.vehicles[spec.id] = self._create_backend(spec)
        if not self.vehicles:
            raise ValueError("Run config must contain at least one vehicle")

    def _create_backend(self, spec: VehicleSpec) -> VehicleBackend:
        """Map a vehicle spec to the concrete backend implementation."""
        if self.dry_run:
            from backends.dry_run import DryRunVehicleBackend
            return DryRunVehicleBackend(spec)
        if spec.backend == "dt":
            from backends.dt_vehicle import RflySimDTBackend
            return RflySimDTBackend(spec)
        raise ValueError(f"Unsupported vehicle backend: {spec.backend}")

    def run(self) -> Path:
        """Run the full scenario lifecycle and write a verification report."""
        started = datetime.now()
        Log.info("Runner", f"case {self.scenario.case_id}: {self.scenario.name}")
        Log.info("Runner", f"dry_run={self.dry_run}")

        self.build_vehicles()
        self.toolchain.start()

        try:
            self._initialize_vehicles()
            self._start_recording()
            for index, step in enumerate(self.scenario.timeline, start=1):
                self._execute_step(index, step)
        finally:
            self._stop_recording()
            self._shutdown_vehicles()
            self.toolchain.stop()

        return self._write_report(started)

    def _initialize_vehicles(self) -> None:
        """Initialize all configured vehicles before timeline execution."""
        Log.info("Runner", f"initializing {len(self.vehicles)} vehicle backend(s)")
        self._parallel_call(list(self.vehicles.values()), "initialize", {})

    def _shutdown_vehicles(self) -> None:
        """Shutdown all backends even if a scenario step fails."""
        Log.info("Runner", "shutting down vehicle backends")
        self._parallel_call(list(self.vehicles.values()), "shutdown", {}, tolerate_errors=True)

    def _start_recording(self) -> None:
        """Create and start the optional real-time xlsx recorder."""
        self.recorder = RealtimeDataRecorder.from_config(
            self.config.get("recording"),
            vehicles=self.vehicles,
            case_id=self.scenario.case_id,
            base_dir=self.base_dir,
        )
        self.recorder.start()

    def _stop_recording(self) -> None:
        """Stop the optional recorder and keep its xlsx path for the report."""
        if self.recorder is None:
            return
        self.data_log_path = self.recorder.stop()

    def _select_targets(self, step: dict[str, Any]) -> list[VehicleBackend]:
        """Resolve a step's target/targets field into backend objects."""
        targets = step.get("targets", step.get("target", "all"))
        if targets == "all":
            return list(self.vehicles.values())
        if isinstance(targets, str):
            targets = [targets]
        selected = []
        for target in targets:
            if target not in self.vehicles:
                raise KeyError(f"Unknown target vehicle '{target}' in action {step.get('action')}")
            selected.append(self.vehicles[target])
        return selected

    def _execute_step(self, index: int, step: dict[str, Any]) -> None:
        """Dispatch one semantic Scenario action to its target backends."""
        action = str(step["action"])
        started = time.time()
        Log.info("Runner", f"step {index}/{len(self.scenario.timeline)} action={action}")

        if action == "wait":
            seconds = float(step.get("seconds", 0.0))
            if self.dry_run:
                Log.info("Runner", f"dry-run skip wait {seconds:.2f}s")
            else:
                time.sleep(seconds)
            self._record_event(index, step, started)
            return

        targets = self._select_targets(step)
        kwargs = {k: v for k, v in step.items() if k not in {"action", "target", "targets"}}

        if action == "mission":
            trajectory = generate_trajectory(kwargs.pop("trajectory"))
            kwargs["trajectory"] = trajectory
            kwargs.setdefault("period_s", step.get("period_s", 0.1))

        method_name = {
            "arm": "arm",
            "disarm": "disarm",
            "takeoff": "takeoff",
            "goto": "goto",
            "velocity": "velocity",
            "hover": "hover",
            "land": "land",
            "inject_fault": "inject_fault",
            "clear_fault": "clear_fault",
            "switch_virtual_redundancy": "switch_virtual_redundancy",
            "mission": "run_mission",
        }.get(action)
        if method_name is None:
            raise ValueError(f"Unsupported scenario action: {action}")

        self._parallel_call(targets, method_name, kwargs)
        self._record_event(index, step, started)

    def _parallel_call(
        self,
        targets: list[VehicleBackend],
        method_name: str,
        kwargs: dict[str, Any],
        tolerate_errors: bool = False,
    ) -> None:
        """Invoke the same backend method on multiple vehicles concurrently."""
        errors: list[BaseException] = []

        def worker(vehicle: VehicleBackend) -> None:
            try:
                getattr(vehicle, method_name)(**kwargs)
            except BaseException as exc:
                errors.append(exc)
                Log.error(vehicle.spec.id, f"{method_name} failed: {exc}")

        threads = [threading.Thread(target=worker, args=(vehicle,), daemon=True) for vehicle in targets]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        if errors and not tolerate_errors:
            raise errors[0]

    def _record_event(self, index: int, step: dict[str, Any], started: float) -> None:
        """Capture step timing and vehicle snapshots for the report."""
        self.events.append({
            "index": index,
            "action": step["action"],
            "elapsed_s": round(time.time() - started, 6),
            "targets": step.get("targets", step.get("target", "all")),
            "snapshots": {vehicle_id: vehicle.snapshot() for vehicle_id, vehicle in self.vehicles.items()},
        })

    def _write_report(self, started: datetime) -> Path:
        """Persist the scenario execution trace as a JSON report."""
        reports_ref = Path(self.config.get("reports_dir", self.base_dir.parent / "reports"))
        reports_dir = reports_ref if reports_ref.is_absolute() else self.base_dir / reports_ref
        reports_dir = reports_dir.resolve()
        reports_dir.mkdir(parents=True, exist_ok=True)
        report_path = reports_dir / f"{self.scenario.case_id}_{started.strftime('%Y%m%d_%H%M%S')}.json"
        report = {
            "case_id": self.scenario.case_id,
            "name": self.scenario.name,
            "started_at": started.isoformat(timespec="seconds"),
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "dry_run": self.dry_run,
            "vehicle_count": len(self.vehicles),
            "data_log": str(self.data_log_path) if self.data_log_path else None,
            "events": self.events,
        }
        with report_path.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        Log.ok("Runner", f"report written: {report_path}")
        return report_path
