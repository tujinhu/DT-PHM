from __future__ import annotations

import os
from pathlib import Path
import platform
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
        """Start the configured RflySim/PX4 toolchain script when enabled."""
        if not self.config.get("enabled", False):
            Log.info("Toolchain", "RflySim toolchain launch disabled")
            return

        script_path = self._select_script_path()
        os_name = self._current_os_key()
        wait_s = float(self.config.get("startup_wait_s", 20.0))

        if self.dry_run:
            Log.info("Toolchain", f"dry-run would start {os_name} toolchain: {script_path}")
            return

        if not script_path.exists():
            raise FileNotFoundError(f"RflySim toolchain script not found: {script_path}")

        runtime_env = {str(k): str(v) for k, v in self.config.get("env", {}).items()}
        vehicle_num = runtime_env.get("RFV_VEHICLE_NUM")
        start_index = runtime_env.get("RFV_START_INDEX", "1")
        if vehicle_num is not None:
            Log.info(
                "Toolchain",
                f"write RFV_VEHICLE_NUM={vehicle_num} RFV_START_INDEX={start_index} to {script_path.name}",
            )
            self._write_runtime_config(script_path, vehicle_num, start_index)

        Log.info("Toolchain", f"starting {os_name} toolchain: {script_path}")
        self.process = self._open_process(script_path, runtime_env)

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

    def _select_script_path(self) -> Path:
        """Choose the platform-specific toolchain script from the config."""
        scripts = self.config.get("scripts")
        os_key = self._current_os_key()
        script_ref: str | None = None
        if isinstance(scripts, dict):
            script_ref = scripts.get(os_key)
        if script_ref is None:
            script_ref = self.config.get("script_path")
        if script_ref is None:
            script_ref = self.config.get("bat_path") if os_key == "windows" else self.config.get("sh_path")
        if script_ref is None:
            raise ValueError(
                "Toolchain config must provide scripts.windows/scripts.linux, script_path, bat_path, or sh_path"
            )
        return Path(script_ref).resolve()

    def _open_process(self, script_path: Path, runtime_env: dict[str, str]) -> subprocess.Popen[str]:
        """Launch the selected script with the proper shell for the host OS."""
        env = os.environ.copy()
        env.update(runtime_env)
        suffix = script_path.suffix.lower()
        if self._current_os_key() == "windows" or suffix in {".bat", ".cmd"}:
            return subprocess.Popen(
                ["cmd.exe", "/c", str(script_path)],
                cwd=str(script_path.parent),
                env=env,
                creationflags=subprocess.CREATE_NEW_CONSOLE if hasattr(subprocess, "CREATE_NEW_CONSOLE") else 0,
            )
        return subprocess.Popen(
            ["bash", str(script_path)],
            cwd=str(script_path.parent),
            env=env,
        )

    @staticmethod
    def _current_os_key() -> str:
        """Return the config key for the current operating system."""
        return "windows" if platform.system().lower().startswith("win") else "linux"

    @staticmethod
    def _write_runtime_config(script_path: Path, vehicle_num: str, start_index: str) -> None:
        """Write RFV runtime values into a supported script before launch."""
        suffix = script_path.suffix.lower()
        if suffix in {".bat", ".cmd"}:
            ToolchainProcess._write_runtime_config_to_bat(script_path, vehicle_num, start_index)
        elif suffix == ".sh":
            ToolchainProcess._write_runtime_config_to_sh(script_path, vehicle_num, start_index)
        else:
            raise ValueError(f"Unsupported toolchain script suffix: {script_path.suffix}")

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

    @staticmethod
    def _write_runtime_config_to_sh(sh_path: Path, vehicle_num: str, start_index: str) -> None:
        """Write RFV runtime values directly into the local shell script before launch."""
        begin = "# BEGIN RFV_RUNTIME_CONFIG"
        end = "# END RFV_RUNTIME_CONFIG"
        text = sh_path.read_text(encoding="utf-8")
        block = (
            f"{begin}\n"
            f"export RFV_VEHICLE_NUM={vehicle_num}\n"
            f"export RFV_START_INDEX={start_index}\n"
            f"{end}"
        )
        if begin not in text or end not in text:
            raise ValueError(f"Missing RFV runtime config markers in {sh_path}")
        prefix, rest = text.split(begin, 1)
        _, suffix = rest.split(end, 1)
        sh_path.write_text(prefix + block + suffix, encoding="utf-8")
