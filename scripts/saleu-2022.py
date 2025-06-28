from __future__ import annotations

import argparse
import json
import re
import tempfile
from pathlib import Path
from typing import List, TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        path: Path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path, help="Path to the original CVRP instance")
    args = parser.parse_args(namespace=Namespace())

    x: List[float] = []
    y: List[float] = []
    dronable: List[bool] = []
    demands: List[float] = []

    with args.path.open("r", encoding="utf-8") as reader:
        data = reader.read()
        for match in re.finditer(r"^\s*\d+\s+(-?[\d\.]+)\s+(-?[\d\.]+)\s*$", data, flags=re.MULTILINE):
            _x, _y = map(float, match.groups())
            x.append(_x)
            y.append(_y)
            dronable.append(True)
            demands.append(0)

    # The depot is the first customer in the list
    depot = x.pop(0), y.pop(0)
    dronable = dronable[1:]
    demands = demands[1:]

    # Mapping from instances to fleet sizes
    FLEET_SIZE = {
        "CMT1": 5,
        "CMT2": 10,
        "CMT3": 8,
        "CMT4": 12,
        "CMT5": 17,
    }
    try:
        fleet_size = FLEET_SIZE[args.path.stem]
    except KeyError:
        fleet_size = int(re.fullmatch(r"^[A-Z]-n\d+-k(\d+)$", args.path.stem).group(1))  # type: ignore

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".txt",
        prefix=f"{args.path.stem}_",
        delete=False,
    ) as output:
        output.write(f"trucks_count {(fleet_size + 1) // 2}\n")
        output.write(f"drones_count {fleet_size // 2}\n")
        output.write(f"customers {len(x)}\n")
        output.write(f"depot {depot[0]} {depot[1]}\n")

        output.write("Coordinate X         Coordinate Y         Dronable Demand\n")
        for _x, _y, _dronable, _demand in zip(x, y, dronable, demands, strict=True):
            output.write(f"{_x:20} {_y:20} {int(_dronable)} {_demand:20}\n")

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".json",
        delete=False,
    ) as truck:
        json.dump(
            {
                "V_max (m/s)": 1,
                "M_t (kg)": 1,
            },
            truck,
            indent=4,
            ensure_ascii=False,
        )

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".json",
        delete=False,
    ) as drone:
        common = {
            "capacity [kg]": 1,
            "FixedTime (s)": 31557600,
            "V_max (m/s)": 1,
        }
        json.dump(
            [
                {
                    "speed_type": "low",
                    "range_type": "low",
                    **common,
                },
                {
                    "speed_type": "low",
                    "range_type": "high",
                    **common,
                },
                {
                    "speed_type": "high",
                    "range_type": "low",
                    **common,
                },
                {
                    "speed_type": "high",
                    "range_type": "high",
                    **common,
                },
            ],
            drone,
            indent=4,
            ensure_ascii=False,
        )

    print(
        f"run {output.name} --truck-cfg {truck.name} --drone-cfg {drone.name} -c endurance "
        "--truck-distance manhattan --drone-distance euclidean --waiting-time-limit 31557600 "
        "--single-truck-route --single-drone-route",
    )
