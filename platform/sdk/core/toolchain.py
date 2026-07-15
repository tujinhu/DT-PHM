from __future__ import annotations

from pathlib import Path
import subprocess
import time
from typing import Any

from core.logging import Log


class ToolchainProcess:
    def __init__(self, config: dict[str, Any] | None, dry_run: bool = False):
        """Store RflySim toolchain launch settings."""
        self.config = config or {}
        self.dry_run = dry_run
        self.process: subprocess.Popen[str] | None = None

    def start(self) -> None:
        """Start the configured RflySim batch file when enabled."""
        if not self.config.get("enabled", False):
            Log.info("Toolchain", "RflySim toolchain launch disabled")
            return

        bat_path = Path(self.config["bat_path"]).resolve()
        wait_s = float(self.config.get("startup_wait_s", 20.0))

        if self.dry_run:
            Log.info("Toolchain", f"dry-run would start: {bat_path}")
            return

        if not bat_path.exists():
            raise FileNotFoundError(f"RflySim batch file not found: {bat_path}")

        runtime_env = {str(k): str(v) for k, v in self.config.get("env", {}).items()}
        vehicle_num = runtime_env.get("RFV_VEHICLE_NUM")
        start_index = runtime_env.get("RFV_START_INDEX", "1")
        if vehicle_num is not None:
            Log.info(
                "Toolchain",
                f"write RFV_VEHICLE_NUM={vehicle_num} RFV_START_INDEX={start_index} to bat",
            )
            self._write_runtime_config_to_bat(bat_path, vehicle_num, start_index)

        Log.info("Toolchain", f"starting RflySim batch: {bat_path}")
        self.process = subprocess.Popen(
            ["cmd.exe", "/c", str(bat_path)],
            cwd=str(bat_path.parent),
            creationflags=subprocess.CREATE_NEW_CONSOLE if hasattr(subprocess, "CREATE_NEW_CONSOLE") else 0,
        )

        if wait_s > 0:
            Log.info("Toolchain", f"waiting {wait_s:.1f}s for RflySim startup")
            time.sleep(wait_s)

    def stop(self) -> None:
        """Optionally terminate the toolchain process started by this runner."""
        stop_on_exit = bool(self.config.get("stop_on_exit", False))
        if self.dry_run or self.process is None:
            return
        if stop_on_exit and self.process.poll() is None:
            Log.warn("Toolchain", "terminating toolchain process")
            self.process.terminate()

    @staticmethod
    def _write_runtime_config_to_bat(bat_path: Path, vehicle_num: str, start_index: str) -> None:
        """Write RFV runtime values directly into the local batch file before launch."""
        begin = "REM BEGIN RFV_RUNTIME_CONFIG"
        end = "REM END RFV_RUNTIME_CONFIG"
        text = bat_path.read_text(encoding="utf-8")
        block = (
            f"{begin}\n"
            f"SET RFV_VEHICLE_NUM={vehicle_num}\n"
            f"SET RFV_START_INDEX={start_index}\n"
            f"{end}"
        )
        if begin not in text or end not in text:
            raise ValueError(f"Missing RFV runtime config markers in {bat_path}")
        prefix, rest = text.split(begin, 1)
        _, suffix = rest.split(end, 1)
        bat_path.write_text(prefix + block + suffix, encoding="utf-8")
