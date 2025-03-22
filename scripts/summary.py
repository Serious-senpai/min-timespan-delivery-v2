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
            "Endurance fixed time [s]",
            "Endurance drone speed [m/s]",
            "Cost [minute]",
            "MILP cost [minute]",
            "Improved [%]",
            "MILP performance [s]",
            "MILP status",
            "Capacity violation [kg]",
            "Energy violation [J]",
            "Waiting time violation [s]",
            "Fixed time violation [s]",
            "Truck paths",
            "Drone paths",
            "Truck working time",
            "Drone working time",
            "Feasible",
            "Last improved",
            "Elapsed [s]",
            "Extra",
            "Faster [%]",
            "Weight per truck route [kg]",
            "Customers per truck route",
            "Truck route count",
            "Weight per drone route [kg]",
            "Customers per drone route",
            "Drone route count",
            "Strategy",
        ]
        csv.write(",".join(headers))
        csv.write("\n")

        for (dirpath, _, filenames) in directory.walk():
            filenames.sort()
            for filename in filenames:
                if pattern.fullmatch(filename):
                    with open(dirpath / filename, "r", encoding="utf-8") as reader:
                        data = json.load(reader)

                    truck_routes = data["solution"]["truck_routes"]
                    drone_routes = data["solution"]["drone_routes"]

                    truck_route_count = sum(len(routes) for routes in truck_routes)
                    drone_route_count = sum(len(routes) for routes in drone_routes)

                    config = data["config"]
                    truck_weight = sum(sum(sum(config["demands"][c] for c in route) for route in routes) for routes in truck_routes)
                    drone_weight = sum(sum(sum(config["demands"][c] for c in route) for route in routes) for routes in drone_routes)
                    truck_customers = sum(sum(len(route) - 2 for route in routes) for routes in truck_routes)
                    drone_customers = sum(sum(len(route) - 2 for route in routes) for routes in drone_routes)

                    segments = [
                        wrap(data["problem"]),
                        "",
                        str(config["trucks_count"]),
                        str(config["drones_count"]),
                        str(data["iterations"]),
                        str(config["tabu_size_factor"]),
                        str(config["reset_after_factor"]),
                        str(data["tabu_size"]),
                        str(data["reset_after"]),
                        str(config["max_elite_size"]),
                        config["config"],
                        config["speed_type"],
                        config["range_type"],
                        str(config["waiting_time_limit"]),
                        str(config["truck"]["V_max (m/s)"]),
                        str(config["drone"]["_data"].get("FixedTime (s)", -1)),
                        str(config["drone"]["_data"].get("V_max (m/s)", -1)),
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
                        wrap(config["extra"]),
                        "",
                        str(truck_weight / truck_route_count if truck_route_count > 0 else 0),
                        str(truck_customers / truck_route_count if truck_route_count > 0 else 0),
                        str(truck_route_count),
                        str(drone_weight / drone_route_count if drone_route_count > 0 else 0),
                        str(drone_customers / drone_route_count if drone_route_count > 0 else 0),
                        str(drone_route_count),
                        config["strategy"],
                    ]
                    csv.write(",".join(segments) + "\n")
