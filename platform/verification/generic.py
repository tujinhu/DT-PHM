from __future__ import annotations

import argparse
from pathlib import Path
import sys


THIS_DIR = Path(__file__).resolve().parent
SDK_DIR = THIS_DIR.parent / "sdk"
DEFAULT_CONFIG = THIS_DIR / "configs" / "generic_dt.json"
sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(SDK_DIR))

from verification_runner import VerificationRunner


def main() -> int:
    """Run a configurable DT-only verification topology."""
    parser = argparse.ArgumentParser(description="Run semantic Scenario verification on configurable RflySim backends.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Path to a verification run config JSON.")
    parser.add_argument("--scenario", help="Optional scenario JSON override.")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print actions without launching RflySim.")
    args = parser.parse_args()

    runner = VerificationRunner.from_config_file(args.config, scenario_path=args.scenario, dry_run=args.dry_run)
    runner.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
