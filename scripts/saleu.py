from __future__ import annotations

import argparse
import json
import re
import tempfile
from math import sqrt
from pathlib import Path
from typing import List, Literal, TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        instance: str
        el: Literal[0, 20, 40, 60, 80, 100]
        sp: Literal[1, 2, 3, 4, 5]
        drones_count: Literal[1, 2, 3, 4, 5]
        dp: Literal[1, 2]


def euc(dx: float, dy: float) -> float:
    return sqrt(dx ** 2 + dy ** 2)


ROOT = Path(__file__).parent.parent.resolve()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("instance", type=str, help="Name of the original TSPLIB instance")
    parser.add_argument("--el", type=int, default=80, choices=[0, 20, 40, 60, 80, 100], help="Percentage of drone-eligible customers")
    parser.add_argument("--sp", type=int, default=2, choices=[1, 2, 3, 4, 5], help="Drone speed compared to truck")
    parser.add_argument("--drones-count", type=int, default=1, choices=[1, 2, 3, 4, 5], help="Number of drones")
    parser.add_argument("--dp", type=int, default=1, choices=[1, 2], help="Depot location type")
    args = parser.parse_args(namespace=Namespace())

    tsp = ROOT.joinpath("problems", "tsplib", f"{args.instance}.tsp").read_text(encoding="utf-8")
    coordinates = re.compile(r"^\s*(\d+)\s+(-?[\d\.]+)\s+(-?[\d\.]+)\s*$", flags=re.MULTILINE)

    x: List[float] = []
    y: List[float] = []
    dronable: List[bool] = []
    demands: List[float] = []
    for match in coordinates.finditer(tsp):
        _index, _x, _y = match.groups()
        index = int(_index)
        x.append(float(_x))
        y.append(float(_y))
        dronable.append(args.el == 100 or index % int(1 / (0.3 * (1 - args.el / 100))) != 0)
        demands.append(0.0)  # Drone-eligibility is decided by `dronable` instead.

    if args.dp == 1:
        # Actually, the depot should be (max + min) / 2, but it seems like the original author
        # made a mistake.
        depot = (max(x) - min(x)) / 2, (max(y) - min(y)) / 2
    else:
        depot = min(x), min(y)

    dronable_index = [i for i, d in enumerate(dronable) if d]
    dronable_index.sort(key=lambda i: euc(x[i] - depot[0], y[i] - depot[1]))
    while len(dronable_index) > len(x) * args.el / 100:
        index = dronable_index.pop()
        dronable[index] = False

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".txt",
        prefix=f"{args.instance}_",
        delete=False,
    ) as output:
        output.write("trucks_count 1\n")
        output.write(f"drones_count {args.drones_count}\n")
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
            "FixedTime (s)": 86400,
            "V_max (m/s)": args.sp,
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
        "--truck-distance manhattan --drone-distance euclidean --waiting-time-limit 86400 "
        "--single-truck-route --single-drone-route",
    )
