from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from datetime import datetime
from pathlib import Path
import platform as host_platform
import re
import shutil
import threading
import time
from typing import Any

import pandas as pd


class Log:
    """Small console logger kept dependency-free for RflySim scripts."""

    @staticmethod
    def _ts() -> str:
        return time.strftime("%H:%M:%S")

    @staticmethod
    def info(tag: str, msg: str) -> None:
        print(f"[{Log._ts()}] [{tag}] {msg}")

    @staticmethod
    def ok(tag: str, msg: str) -> None:
        print(f"[{Log._ts()}] [{tag}] OK {msg}")

    @staticmethod
    def warn(tag: str, msg: str) -> None:
        print(f"[{Log._ts()}] [{tag}] WARN {msg}")

    @staticmethod
    def error(tag: str, msg: str) -> None:
        print(f"[{Log._ts()}] [{tag}] ERROR {msg}")


DEFAULT_RECORD_FIELDS = [
    "uavAngEular",
    "uavAngRate",
    "uavPosNED",
    "uavVelNED",
    "uavAngQuatern",
    "uavAccB",
    "uavGyro",
    "uavMag",
    "uavVibr",
]


class RealtimeDataRecorder:
    """Sample vehicle runtime states and save one workbook per verification run."""

    def __init__(
        self,
        vehicles: Mapping[str, Any],
        fields: Sequence[str] | None = None,
        sample_hz: float = 50.0,
        output_dir: str | Path = "log",
        case_id: str = "case",
        enabled: bool = False,
        metadata_provider: Callable[[], Mapping[str, Any]] | None = None,
        run_stamp: str | None = None,
    ):
        """Configure sampling frequency, fields, output path, and target vehicles."""
        self.vehicles = vehicles
        self.fields = list(fields or DEFAULT_RECORD_FIELDS)
        self.sample_hz = float(sample_hz)
        self.output_dir = Path(output_dir).resolve()
        self.case_id = case_id
        self.enabled = bool(enabled)
        self.metadata_provider = metadata_provider
        self.run_stamp = run_stamp
        self.records: dict[str, list[dict[str, Any]]] = {vehicle_id: [] for vehicle_id in vehicles}
        self.output_path: Path | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_time = 0.0
        self._sample_index = 0

    @staticmethod
    def from_config(
        config: dict[str, Any] | None,
        vehicles: Mapping[str, Any],
        case_id: str,
        base_dir: str | Path,
        metadata_provider: Callable[[], Mapping[str, Any]] | None = None,
        run_stamp: str | None = None,
    ) -> "RealtimeDataRecorder":
        """Create a recorder from the optional run-config recording section."""
        cfg = config or {}
        output_ref = Path(cfg.get("output_dir", "../log"))
        output_dir = output_ref if output_ref.is_absolute() else Path(base_dir).resolve() / output_ref
        return RealtimeDataRecorder(
            vehicles=vehicles,
            fields=cfg.get("fields", DEFAULT_RECORD_FIELDS),
            sample_hz=float(cfg.get("sample_hz", 50.0)),
            output_dir=output_dir,
            case_id=case_id,
            enabled=bool(cfg.get("enabled", False)),
            metadata_provider=metadata_provider,
            run_stamp=run_stamp,
        )

    def start(self) -> None:
        """Start the background sampling thread when recording is enabled."""
        if not self.enabled:
            Log.info("Recorder", "real-time data recording disabled")
            return
        if self._thread is not None:
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._start_time = time.time()
        self._stop_event.clear()
        self._sample_once(self._start_time)
        self._thread = threading.Thread(target=self._run_loop, name="RealtimeDataRecorder", daemon=True)
        self._thread.start()
        Log.ok("Recorder", f"recording {len(self.fields)} field(s) at {self.sample_hz:g} Hz")

    def stop(self) -> Path | None:
        """Stop sampling and write all vehicle sheets into a single xlsx file."""
        if not self.enabled:
            return None
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self.output_path = self._write_xlsx()
        return self.output_path

    def _run_loop(self) -> None:
        """Collect one row per vehicle at the configured fixed sample rate."""
        period_s = 1.0 / max(self.sample_hz, 1e-6)
        next_sample = time.time()
        while not self._stop_event.is_set():
            now = time.time()
            if now < next_sample:
                time.sleep(min(next_sample - now, 0.05))
                continue
            self._sample_once(now)
            next_sample += period_s
            if next_sample < time.time() - period_s:
                next_sample = time.time() + period_s

    def _sample_once(self, now: float) -> None:
        """Read selected fields from each vehicle backend and append rows in memory."""
        self._sample_index += 1
        elapsed_s = now - self._start_time
        wall_time = datetime.now().isoformat(timespec="milliseconds")
        for vehicle_id, vehicle in self.vehicles.items():
            try:
                payload = vehicle.record_snapshot(self.fields)
            except Exception as exc:
                payload = {"record_error": str(exc)}
            row = {
                "sample_index": self._sample_index,
                "wall_time": wall_time,
                "elapsed_s": elapsed_s,
                "vehicle_id": vehicle_id,
            }
            if self.metadata_provider is not None:
                try:
                    row.update(dict(self.metadata_provider()))
                except Exception as exc:
                    row["metadata_error"] = str(exc)
            row.update(self._flatten_payload(payload))
            self.records.setdefault(vehicle_id, []).append(row)

    def _flatten_payload(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Flatten scalar/list/dict values into spreadsheet-friendly columns."""
        flat: dict[str, Any] = {}
        for key, value in payload.items():
            if isinstance(value, Mapping):
                for sub_key, sub_value in value.items():
                    flat[f"{key}_{sub_key}"] = sub_value
            elif isinstance(value, (list, tuple)):
                for index, item in enumerate(value):
                    flat[f"{key}_{index}"] = item
            else:
                flat[key] = value
        return flat

    def _write_xlsx(self) -> Path:
        """Write one xlsx workbook with one sheet per vehicle."""
        stamp = self.run_stamp or datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.output_dir / f"{self.case_id}_{stamp}.xlsx"
        with pd.ExcelWriter(path, engine="openpyxl") as writer:
            wrote_any = False
            for vehicle_id, rows in self.records.items():
                if not rows:
                    continue
                sheet_name = self._safe_sheet_name(vehicle_id)
                pd.DataFrame(rows).to_excel(writer, sheet_name=sheet_name, index=False)
                wrote_any = True
            if not wrote_any:
                pd.DataFrame([{"info": "no samples recorded"}]).to_excel(writer, sheet_name="empty", index=False)
        Log.ok("Recorder", f"xlsx written: {path}")
        return path

    @staticmethod
    def _safe_sheet_name(name: str) -> str:
        """Convert a vehicle id into a valid Excel sheet name."""
        cleaned = re.sub(r"[\[\]\:\*\?\/\\]", "_", str(name))
        return (cleaned or "vehicle")[:31]


class PX4OfflineLogCollector:
    """Archive PX4 ULog files for configured DT vehicle backends."""

    def __init__(
        self,
        vehicles: Mapping[str, Any],
        output_dir: str | Path = "log/offline",
        enabled: bool = False,
        firmware_dir: str | Path | None = None,
        build_dir: str | Path | None = None,
        date_name: str | None = None,
        group_by_run: bool = True,
        settle_s: float = 2.0,
    ):
        """Configure where to find PX4 instance logs and where to copy them."""
        self.vehicles = vehicles
        self.output_dir = Path(output_dir).resolve()
        self.enabled = bool(enabled)
        self.firmware_dir = Path(firmware_dir).resolve() if firmware_dir else None
        self.build_dir = Path(build_dir).resolve() if build_dir else None
        self.date_name = str(date_name) if date_name else None
        self.group_by_run = bool(group_by_run)
        self.settle_s = float(settle_s)

    @staticmethod
    def from_config(
        config: dict[str, Any] | None,
        vehicles: Mapping[str, Any],
        base_dir: str | Path,
    ) -> "PX4OfflineLogCollector":
        """Create an offline ULog collector from the optional config section."""
        cfg = config or {}
        output_ref = Path(cfg.get("output_dir", "../log/offline"))
        output_dir = output_ref if output_ref.is_absolute() else Path(base_dir).resolve() / output_ref
        return PX4OfflineLogCollector(
            vehicles=vehicles,
            output_dir=output_dir,
            enabled=bool(cfg.get("enabled", False)),
            firmware_dir=cfg.get("firmware_dir"),
            build_dir=cfg.get("build_dir"),
            date_name=cfg.get("date"),
            group_by_run=bool(cfg.get("group_by_run", True)),
            settle_s=float(cfg.get("settle_s", 2.0)),
        )

    def collect(self, case_id: str, started: datetime, dry_run: bool = False) -> list[Path]:
        """Copy the newest PX4 .ulg for each DT vehicle and return copied paths."""
        if not self.enabled:
            return []
        if dry_run:
            Log.info("OfflineLog", "dry-run skip PX4 ULog copy")
            return []

        dt_vehicles = {
            vehicle_id: vehicle
            for vehicle_id, vehicle in self.vehicles.items()
            if getattr(getattr(vehicle, "spec", None), "backend", None) == "dt"
        }
        if not dt_vehicles:
            Log.info("OfflineLog", "skip PX4 ULog copy because no DT backend is present")
            return []

        if self.settle_s > 0:
            time.sleep(self.settle_s)

        target_dir = self._resolve_output_dir(case_id, started)
        target_dir.mkdir(parents=True, exist_ok=True)
        build_dir = self._resolve_px4_build_dir()

        copied: list[Path] = []
        for vehicle_id, vehicle in dt_vehicles.items():
            instance_id = int(vehicle.spec.vehicle_id)
            instance_dir = build_dir / f"instance_{instance_id}" / "log"
            source_log = self._find_latest_ulg(instance_dir, self.date_name)
            if source_log is None:
                Log.warn("OfflineLog", f"no .ulg found for {vehicle_id} at {instance_dir}")
                continue
            target_name = f"{case_id}_{vehicle_id}_instance_{instance_id}_{source_log.name}"
            target_path = target_dir / target_name
            shutil.copy2(source_log, target_path)
            copied.append(target_path)
            Log.ok("OfflineLog", f"copied {vehicle_id}: {target_path}")
        return copied

    def _resolve_output_dir(self, case_id: str, started: datetime) -> Path:
        """Resolve the destination folder for one run's copied ULog files."""
        if self.group_by_run:
            return self.output_dir / f"{case_id}_{started.strftime('%Y%m%d_%H%M%S')}"
        return self.output_dir

    def _resolve_px4_build_dir(self) -> Path:
        """Resolve the PX4 build folder containing instance_* log folders."""
        if self.build_dir is not None:
            return self.build_dir
        firmware_dir = self.firmware_dir or self._default_firmware_dir()
        return firmware_dir / "build" / "px4_sitl_default"

    @staticmethod
    def _default_firmware_dir() -> Path:
        """Return the platform default PX4 Firmware directory."""
        if host_platform.system().lower().startswith("win"):
            return Path("C:/PX4PSP/Firmware")
        return Path.home() / "PX4PSP" / "Firmware"

    @staticmethod
    def _find_latest_ulg(instance_log_dir: Path, date_name: str | None = None) -> Path | None:
        """Find the newest .ulg under instance_N/log[/date]."""
        if date_name:
            date_dirs = [instance_log_dir / date_name]
        elif instance_log_dir.exists():
            date_dirs = [path for path in instance_log_dir.iterdir() if path.is_dir()]
            date_dirs.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        else:
            return None

        for date_dir in date_dirs:
            if not date_dir.exists():
                continue
            logs = sorted(date_dir.glob("*.ulg"), key=lambda path: path.stat().st_mtime, reverse=True)
            if logs:
                return logs[0]
        return None
