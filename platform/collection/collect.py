from __future__ import annotations

import argparse
from pathlib import Path
import sys


THIS_DIR = Path(__file__).resolve().parent
SDK_DIR = THIS_DIR.parent / "sdk"
DEFAULT_CONFIG = THIS_DIR / "configs" / "real_dt_collection.json"
sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(SDK_DIR))

from collection_runner import CollectionRunner


def main() -> int:
    """Run real+DT online collection with one semantic collection config."""
    parser = argparse.ArgumentParser(description="Collect online real+DT data from semantic timelines.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Path to a collection config JSON.")
    parser.add_argument("--dry-run", action="store_true", help="Validate actions without importing ROS or RflySim.")
    args = parser.parse_args()

    runner = CollectionRunner.from_config_file(args.config, dry_run=args.dry_run)
    runner.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
