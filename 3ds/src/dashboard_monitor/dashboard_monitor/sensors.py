"""Read Linux thermal and hwmon interfaces without subprocesses or privileges."""
from __future__ import annotations

import math
import re
from pathlib import Path

# Conservative matching: an arbitrary thermal zone is not necessarily a CPU.
CPU = re.compile(r"cpu|x86_pkg_temp|coretemp|k10temp|zenpower", re.I)
GPU = re.compile(r"gpu|amdgpu|nouveau", re.I)


def read_temperatures(sys_root: Path = Path("/sys")) -> dict:
    """Return all identified CPU/GPU readings, in Celsius; absent means unavailable.

    Jetson exposes CPU-therm/GPU-therm thermal zones. Other Linux machines
    commonly expose coretemp/k10temp/amdgpu hwmon devices. Never infer GPU
    temperature from a board, battery, or NVMe sensor.
    """
    result = {"cpu": [], "gpu": []}
    seen = set()
    candidates = []
    for zone in sorted((sys_root / "class/thermal").glob("thermal_zone*")):
        candidates.append((zone / "temp", zone / "type", None))
    for device in sorted((sys_root / "class/hwmon").glob("hwmon*")):
        for sensor in sorted(device.glob("temp*_input")):
            candidates.append((sensor, device / "name", sensor.with_name(
                sensor.name.replace("_input", "_label"))))
    for sensor, name_path, label_path in candidates:
        try:
            name = name_path.read_text().strip()
            label = label_path.read_text().strip() if label_path and label_path.exists() else ""
            kind = "gpu" if GPU.search(name) else "cpu" if CPU.search(name) else None
            if kind is None:
                continue
            resolved = sensor.resolve()
            if resolved in seen:
                continue
            temperature = float(sensor.read_text().strip()) / 1000.0
            if not math.isfinite(temperature) or not -50 <= temperature <= 200:
                continue
            seen.add(resolved)
            result[kind].append({
                "sensor": f"{name}:{label or sensor.name}", "celsius": temperature,
            })
        except (OSError, ValueError):
            continue
    return result
