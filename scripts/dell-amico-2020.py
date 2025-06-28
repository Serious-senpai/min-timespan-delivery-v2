from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path
from typing import List, Literal, TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        instance: str
        el: Literal[0, 20, 40, 60, 80, 100]
        sp: Literal[1, 2, 3, 4, 5]
        dc: Literal[1, 2, 3, 4, 5]
        dp: Literal[1, 2]


ROOT = Path(__file__).parent.parent.resolve()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("instance", type=str, help="Name of the original TSPLIB instance")
    parser.add_argument("--el", type=int, default=80, choices=[0, 20, 40, 60, 80, 100], help="Percentage of drone-eligible customers")
    parser.add_argument("--sp", type=int, default=2, choices=[1, 2, 3, 4, 5], help="Drone speed compared to truck")
    parser.add_argument("--dc", type=int, default=1, choices=[1, 2, 3, 4, 5], help="Number of drones")
    parser.add_argument("--dp", type=int, default=1, choices=[1, 2], help="Depot location type")
    args = parser.parse_args(namespace=Namespace())

    x: List[float] = []
    y: List[float] = []
    dronable: List[bool] = []
    demands: List[float] = []

    with ROOT.joinpath(
        "problems",
        "dell-amico",
        "TSPLIB_Saleu",
        "Instances",
        f"{args.instance}_{args.dp - 1}_{args.el}.csv",
    ).open("r", encoding="utf-8") as reader:
        for line in reader.readlines():
            _, _x_str, _y_str, _truck_only_str = map(str.strip, line.split(","))
            x.append(float(_x_str))
            y.append(float(_y_str))
            dronable.append(_truck_only_str == "0")
            demands.append(0.0)

    # The depot is the first and last customer in the list
    depot = x[0], y[0]
    x = x[1:-1]
    y = y[1:-1]
    dronable = dronable[1:-1]
    demands = demands[1:-1]

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".txt",
        prefix=f"{args.instance}_{args.el}_{args.sp}_{args.dc}_{args.dp}_",
        delete=False,
    ) as output:
        output.write("trucks_count 1\n")
        output.write(f"drones_count {args.dc}\n")
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
            "V_max (m/s)": args.sp,
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
