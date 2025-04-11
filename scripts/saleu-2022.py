from __future__ import annotations

import argparse
import json
import re
import tempfile
from pathlib import Path
from typing import List, TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        instance: str


ROOT = Path(__file__).parent.parent.resolve()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("instance", type=str, help="Name of the original CVRP instance")
    args = parser.parse_args(namespace=Namespace())

    x: List[float] = []
    y: List[float] = []
    dronable: List[bool] = []
    demands: List[float] = []

    with ROOT.joinpath(
        "problems",
        "saleu-2022",
        f"{args.instance}.vrp",
    ).open("r", encoding="utf-8") as reader:
        data = reader.read()
        for match in re.finditer(r"^\d+ (-?[\d\.]+) (-?[\d\.]+)$", data, flags=re.MULTILINE):
            _x, _y = map(float, match.groups())
            x.append(_x)
            y.append(_y)

        for match in re.finditer(r"^\d+ (-?[\d\.]+)$", data, flags=re.MULTILINE):
            _demand = float(match.group(1))
            dronable.append(True)
            demands.append(_demand)

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
    fleet_size = FLEET_SIZE[args.instance]

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".txt",
        prefix=f"{args.instance}_",
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
            {
                "1": {
                    "speed_type": "low",
                    "range_type": "low",
                    **common,
                },
                "2": {
                    "speed_type": "low",
                    "range_type": "high",
                    **common,
                },
                "3": {
                    "speed_type": "high",
                    "range_type": "low",
                    **common,
                },
                "4": {
                    "speed_type": "high",
                    "range_type": "high",
                    **common,
                },
            },
            drone,
            indent=4,
            ensure_ascii=False,
        )

    print(
        f"run {output.name} --truck-cfg {truck.name} --drone-cfg {drone.name} -c endurance "
        "--truck-distance manhattan --drone-distance euclidean --waiting-time-limit 31557600 "
        "--single-truck-route --single-drone-route",
    )
