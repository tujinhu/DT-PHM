from __future__ import annotations

import math
from typing import Any


def generate_trajectory(spec: dict[str, Any] | list[list[float]]) -> list[list[float]]:
    """Expand a semantic trajectory spec into NED position setpoints."""
    if isinstance(spec, list):
        return [[float(v) for v in point] for point in spec]

    shape = spec.get("type", "circle")
    total_time = float(spec.get("total_time", 10.0))
    time_step = float(spec.get("time_step", 0.1))
    repetitions = float(spec.get("repetitions", 1.0))
    total_duration = total_time * repetitions
    count = max(1, int(total_duration / time_step))
    points: list[list[float]] = []

    if shape == "circle":
        radius = float(spec.get("radius", 1.0))
        cx = float(spec.get("center_x", 0.0))
        cy = float(spec.get("center_y", 0.0))
        cz = float(spec.get("center_z", -1.0))
        for i in range(count):
            t = i * time_step
            angle = 2.0 * math.pi * (t / total_time)
            points.append([cx + radius * math.cos(angle), cy + radius * math.sin(angle), cz])
        return points

    if shape == "figure8":
        radius = float(spec.get("radius", 1.0))
        cx = float(spec.get("center_x", 0.0))
        cy = float(spec.get("center_y", 0.0))
        cz = float(spec.get("center_z", -1.0))
        for i in range(count):
            ratio = (i * time_step) / total_time
            points.append([
                cx + radius * math.sin(2.0 * math.pi * ratio),
                cy + radius * math.sin(4.0 * math.pi * ratio),
                cz,
            ])
        return points

    if shape == "line":
        start = [float(v) for v in spec.get("start", [0.0, 0.0, -1.0])]
        end = [float(v) for v in spec.get("end", [2.0, 0.0, -1.0])]
        for i in range(count):
            alpha = i / max(count - 1, 1)
            points.append([start[j] + alpha * (end[j] - start[j]) for j in range(3)])
        return points

    raise ValueError(f"Unsupported trajectory type: {shape}")
