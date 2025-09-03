from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: Path
        single_truck_route: bool


ROOT = Path(__file__).parent.parent.resolve()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("problem", type=Path, help="Path to the PTDS-DDSS instance")
    args = parser.parse_args(namespace=Namespace())

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".json",
        delete=False,
    ) as truck:
        json.dump(
            {
                "V_max (m/s)": 15.557,
                "M_t (kg)": 9000,
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
            "capacity [kg]": 9000,
            "FixedTime (s)": 7200,
            "V_max (m/s)": 22.2626,
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
        f"run {args.problem} --truck-cfg {truck.name} --drone-cfg {drone.name} -c endurance "
        "--truck-distance euclidean --drone-distance euclidean --waiting-time-limit 3600 ",
    )
