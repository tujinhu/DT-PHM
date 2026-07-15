from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


REQUIRED_SCENARIO_KEYS = {"case_id", "name", "timeline"}


@dataclass(frozen=True)
class Scenario:
    case_id: str
    name: str
    timeline: list[dict[str, Any]]
    raw: dict[str, Any]

    @staticmethod
    def load(path: str | Path) -> "Scenario":
        """Load a Scenario JSON file from disk and validate its structure."""
        scenario_path = Path(path).resolve()
        with scenario_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return Scenario.from_dict(data, source=scenario_path)

    @staticmethod
    def from_dict(data: dict[str, Any], source: Path | None = None) -> "Scenario":
        """Build a Scenario object from parsed JSON data."""
        missing = REQUIRED_SCENARIO_KEYS - set(data)
        if missing:
            where = f" in {source}" if source else ""
            raise ValueError(f"Scenario missing required keys{where}: {sorted(missing)}")

        if not isinstance(data["timeline"], list):
            raise TypeError("Scenario 'timeline' must be a list")

        for index, step in enumerate(data["timeline"]):
            if not isinstance(step, dict):
                raise TypeError(f"Scenario step {index} must be an object")
            if "action" not in step:
                raise ValueError(f"Scenario step {index} missing 'action'")

        return Scenario(
            case_id=str(data["case_id"]),
            name=str(data["name"]),
            timeline=data["timeline"],
            raw=data,
        )


def load_json(path: str | Path) -> dict[str, Any]:
    """Read a UTF-8 JSON file into a dictionary."""
    with Path(path).resolve().open("r", encoding="utf-8") as f:
        return json.load(f)
