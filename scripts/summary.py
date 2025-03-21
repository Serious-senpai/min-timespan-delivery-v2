from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        directory: str
        output: str


def wrap(content: Any) -> str:
    return f"\"{content}\""


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(namespace=Namespace())

    directory = Path(args.directory).resolve()
    output = Path(args.output).resolve()
    if output.is_file():
        raise FileExistsError(f"Cannot overwrite existing file at {output}")

    pattern = re.compile(r"^([^-]+?)-\w{8}\.json$")

    with output.open("w", encoding="utf-8") as csv:
        csv.write("sep=,\n")
        headers = [
            "Problem",
            "Customers count",
            "Trucks count",
            "Drones count",
            "Iterations",
            "Tabu size factor",
            "Reset after factor",
            "Tabu size",
            "Reset after",
            "Max elite set size",
            "Energy model",
            "Speed type",
            "Range type",
            "Waiting time limit",
            "Truck maximum speed",
            "Endurance fixed time",
            "Endurance drone speed",
            "Cost",
            "MILP cost",
            "Improved [%]",
            "MILP performance",
            "MILP status",
            "Capacity violation",
            "Energy violation",
            "Waiting time violation",
            "Fixed time violation",
            "Truck paths",
            "Drone paths",
            "Truck working time",
            "Drone working time",
            "Feasible",
            "Last improved",
            "Elapsed [s]",
            "URL",
            "Faster [%]",
            "Weight per truck route",
            "Customers per truck route",
            "Truck route count",
            "Weight per drone route",
            "Customers per drone route",
            "Drone route count",
            "Strategy",
            "Extra",
        ]
        csv.write(",".join(headers))
        csv.write("\n")

        for (dirpath, _, filenames) in directory.walk():
            filenames.sort()
            for filename in filenames:
                if pattern.fullmatch(filename):
                    with open(dirpath / filename, "r", encoding="utf-8") as reader:
                        data = json.load(reader)

                    segments = [
                        wrap(data["problem"]),
                        "",
                        str(data["config"]["trucks_count"]),
                        str(data["config"]["drones_count"]),
                        str(data["iterations"]),
                        str(data["config"]["tabu_size_factor"]),
                        str(data["config"]["reset_after_factor"]),
                        str(data["tabu_size"]),
                        str(data["reset_after"]),
                        str(data["config"]["max_elite_size"]),
                        data["config"]["config"],
                        data["config"]["speed_type"],
                        data["config"]["range_type"],
                        str(data["config"]["waiting_time_limit"]),
                        str(data["config"]["truck"]["V_max (m/s)"]),
                        str(data["config"]["drone"]["_data"].get("FixedTime (s)", -1)),
                        str(data["config"]["drone"]["_data"].get("V_max (m/s)", -1)),
                        str(data["solution"]["working_time"] / 60),
                        "",
                        "",
                        "",
                        "",
                        str(data["solution"]["capacity_violation"]),
                        str(data["solution"]["energy_violation"]),
                        str(data["solution"]["waiting_time_violation"]),
                        str(data["solution"]["fixed_time_violation"]),
                        wrap(data["solution"]["truck_routes"]),
                        wrap(data["solution"]["drone_routes"]),
                        wrap(data["solution"]["truck_working_time"]),
                        wrap(data["solution"]["drone_working_time"]),
                        str(int(data["solution"]["feasible"])),
                        str(data["last_improved"]),
                        str(data["elapsed"]),
                        wrap(data["extra"]),
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        data["config"]["strategy"],
                        data["config"]["extra"],
                    ]
                    csv.write(",".join(segments) + "\n")
