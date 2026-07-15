from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

from core.interfaces import VehicleBackend, VehicleSpec
from core.logging import Log, RealtimeDataRecorder
from core.scenario import Scenario, load_json
from core.toolchain import ToolchainProcess
from core.trajectory import generate_trajectory


PHASE_IDS = {
    "arm": 1,
    "disarm": 2,
    "takeoff": 3,
    "goto": 4,
    "velocity": 5,
    "hover": 6,
    "land": 7,
    "mission": 8,
    "inject_fault": 9,
    "clear_fault": 10,
    "wait": 11,
}


class CollectionRunner:
    def __init__(
        self,
        config: dict[str, Any],
        timelines: list[Scenario],
        dry_run: bool = False,
        base_dir: str | Path | None = None,
    ):
        """Bind one collection topology to one or more semantic timelines."""
        self.config = config
        self.timelines = timelines
        self.dry_run = dry_run
        self.base_dir = Path(base_dir or Path.cwd()).resolve()
        self.toolchain = ToolchainProcess(config.get("toolchain"), dry_run=dry_run)
        self.vehicles: dict[str, VehicleBackend] = {}
        self.recorder: RealtimeDataRecorder | None = None
        self.current_phase: dict[str, Any] = {}
        self.current_phase_started = time.time()
        self.data_logs: list[Path] = []

    @staticmethod
    def from_config_file(config_path: str | Path, dry_run: bool = False) -> "CollectionRunner":
        """Create a collection runner from an explicit JSON config path."""
        cfg_path = Path(config_path).resolve()
        config = load_json(cfg_path)
        CollectionRunner._resolve_config_paths(config, cfg_path.parent)
        timelines = CollectionRunner._load_timelines(config, cfg_path.parent)
        return CollectionRunner(config, timelines, dry_run=dry_run, base_dir=cfg_path.parent)

    @staticmethod
    def _resolve_config_paths(config: dict[str, Any], config_dir: Path) -> None:
        """Normalize path-like config entries relative to the collection config."""
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
            resolve_in_place(toolchain, "sh_path")
            resolve_in_place(toolchain, "script_path")
            scripts = toolchain.get("scripts")
            if isinstance(scripts, dict):
                resolve_in_place(scripts, "windows")
                resolve_in_place(scripts, "linux")

        for vehicle in config.get("vehicles", []):
            if isinstance(vehicle, dict):
                resolve_in_place(vehicle, "rfly_utils_path")

        scenario_ref = config.get("scenario")
        if isinstance(scenario_ref, str):
            scenario_path = Path(scenario_ref)
            if not scenario_path.is_absolute():
                config["scenario"] = str((config_dir / scenario_path).resolve())

    @staticmethod
    def _load_timelines(config: dict[str, Any], config_dir: Path) -> list[Scenario]:
        """Load one Scenario or multiple timeline objects from the run config."""
        if "timelines" in config:
            timelines = config["timelines"]
            if not isinstance(timelines, list):
                raise TypeError("collection config 'timelines' must be a list")
            return [Scenario.from_dict(item) for item in timelines]

        if "scenario" in config:
            return [Scenario.load(config["scenario"])]

        if "timeline" in config:
            return [Scenario.from_dict(config)]

        raise ValueError("collection config must define 'timelines', 'scenario', or root 'timeline'")

    def build_vehicles(self) -> None:
        """Instantiate DT, real, or dry-run backends declared by the config."""
        for vehicle_data in self.config.get("vehicles", []):
            spec = VehicleSpec.from_dict(vehicle_data)
            self.vehicles[spec.id] = self._create_backend(spec)
        if not self.vehicles:
            raise ValueError("Collection config must contain at least one vehicle")

    def _create_backend(self, spec: VehicleSpec) -> VehicleBackend:
        """Map a vehicle spec to the concrete backend implementation."""
        if self.dry_run:
            from backends.dry_run import DryRunVehicleBackend
            return DryRunVehicleBackend(spec)
        if spec.backend == "dt":
            from backends.dt_vehicle import RflySimDTBackend
            return RflySimDTBackend(spec)
        if spec.backend == "real":
            from backends.real_vehicle import RflySimRealBackend
            return RflySimRealBackend(spec)
        raise ValueError(f"Unsupported vehicle backend: {spec.backend}")

    def run(self) -> list[Path]:
        """Run every configured timeline and return the generated xlsx paths."""
        Log.info("Collection", f"timeline_count={len(self.timelines)} dry_run={self.dry_run}")
        self.build_vehicles()
        self.toolchain.start()

        try:
            self._initialize_vehicles()
            for round_index, scenario in enumerate(self.timelines, start=1):
                self._run_one_timeline(round_index, scenario)
        finally:
            self._shutdown_vehicles()
            self.toolchain.stop()

        return self.data_logs

    def _initialize_vehicles(self) -> None:
        """Initialize all configured collection backends."""
        Log.info("Collection", f"initializing {len(self.vehicles)} vehicle backend(s)")
        self._parallel_call(list(self.vehicles.values()), "initialize", {})

    def _shutdown_vehicles(self) -> None:
        """Shutdown all collection backends."""
        Log.info("Collection", "shutting down vehicle backends")
        self._parallel_call(list(self.vehicles.values()), "shutdown", {}, tolerate_errors=True)

    def _run_one_timeline(self, round_index: int, scenario: Scenario) -> None:
        """Execute one flight round and write one online-data workbook."""
        Log.info("Collection", f"round {round_index}/{len(self.timelines)} case={scenario.case_id}: {scenario.name}")
        self.recorder = None
        self._set_phase(round_index, scenario, 0, "not_started", {})

        try:
            for step_index, step in enumerate(scenario.timeline, start=1):
                self._execute_step(round_index, scenario, step_index, step)
        finally:
            path = self._stop_recording()
            if path is not None:
                self.data_logs.append(path)
            self._clear_real_faults()

    def _execute_step(self, round_index: int, scenario: Scenario, step_index: int, step: dict[str, Any]) -> None:
        """Dispatch one timeline action and maintain recorder phase metadata."""
        action = str(step["action"])
        self._set_phase(round_index, scenario, step_index, action, step)
        Log.info("Collection", f"round {round_index} step {step_index}/{len(scenario.timeline)} action={action}")

        if action == "wait":
            seconds = float(step.get("seconds", 0.0))
            if self.dry_run:
                Log.info("Collection", f"dry-run skip wait {seconds:.2f}s")
            else:
                time.sleep(seconds)
            return

        targets = self._select_targets(step, action)
        kwargs = {k: v for k, v in step.items() if k not in {"action", "target", "targets"}}

        if action == "mission":
            trajectory = generate_trajectory(kwargs.pop("trajectory"))
            kwargs["trajectory"] = trajectory
            kwargs.setdefault("period_s", step.get("period_s", 0.1))

        method_name = self._method_for_action(action)
        self._parallel_call(targets, method_name, kwargs)

        if action == "arm" and self.recorder is None:
            self._start_recording(scenario)

    def _select_targets(self, step: dict[str, Any], action: str) -> list[VehicleBackend]:
        """Resolve action targets, defaulting fault injection to real vehicles only."""
        targets = step.get("targets", step.get("target"))
        if targets is None:
            if action in {"inject_fault", "clear_fault"}:
                real_targets = [vehicle for vehicle in self.vehicles.values() if vehicle.spec.backend == "real"]
                return real_targets or list(self.vehicles.values())
            return list(self.vehicles.values())

        if targets == "all":
            return list(self.vehicles.values())
        if isinstance(targets, str):
            targets = [targets]

        selected = []
        for target in targets:
            if target not in self.vehicles:
                raise KeyError(f"Unknown target vehicle '{target}' in action {action}")
            selected.append(self.vehicles[target])
        return selected

    def _method_for_action(self, action: str) -> str:
        """Map a semantic action name to a backend method."""
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
            "mission": "run_mission",
        }.get(action)
        if method_name is None:
            raise ValueError(f"Unsupported collection action: {action}")
        return method_name

    def _start_recording(self, scenario: Scenario) -> None:
        """Start online recording immediately after the arm command succeeds."""
        self.recorder = RealtimeDataRecorder.from_config(
            self.config.get("recording"),
            vehicles=self.vehicles,
            case_id=scenario.case_id,
            base_dir=self.base_dir,
            metadata_provider=self._recording_metadata,
        )
        self.recorder.start()

    def _stop_recording(self) -> Path | None:
        """Stop the active online recorder for the current flight round."""
        if self.recorder is None:
            return None
        path = self.recorder.stop()
        self.recorder = None
        return path

    def _clear_real_faults(self) -> None:
        """Clear real-aircraft fault flags after each timeline."""
        real_targets = [vehicle for vehicle in self.vehicles.values() if vehicle.spec.backend == "real"]
        if not real_targets:
            return
        Log.info("Collection", "clearing real-vehicle fault flags")
        self._parallel_call(real_targets, "clear_fault", {}, tolerate_errors=True)

    def _set_phase(
        self,
        round_index: int,
        scenario: Scenario,
        step_index: int,
        action: str,
        step: dict[str, Any],
    ) -> None:
        """Update metadata attached to every online recorder sample."""
        self.current_phase_started = time.time()
        self.current_phase = {
            "round_index": round_index,
            "timeline_case_id": scenario.case_id,
            "timeline_name": scenario.name,
            "phase_index": step_index,
            "phase_action": action,
            "phase_id": PHASE_IDS.get(action, 0),
            "phase_targets": step.get("targets", step.get("target", "all")),
        }

    def _recording_metadata(self) -> dict[str, Any]:
        """Return current timeline/phase labels for one recorder row."""
        data = dict(self.current_phase)
        data["phase_elapsed_s"] = time.time() - self.current_phase_started
        return data

    def _parallel_call(
        self,
        targets: list[VehicleBackend],
        method_name: str,
        kwargs: dict[str, Any],
        tolerate_errors: bool = False,
    ) -> None:
        """Invoke one backend method on multiple vehicles concurrently."""
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
