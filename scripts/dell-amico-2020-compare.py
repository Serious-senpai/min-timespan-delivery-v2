from __future__ import annotations

import argparse
import itertools
import json
import re
import sqlite3
from pathlib import Path
from typing import Any, Dict, NamedTuple, TYPE_CHECKING


class DellAmicoKey(NamedTuple):
    problem: str
    el: str
    sp: str
    dc: str
    dp: str


class DellAmicoResult(NamedTuple):
    best: float
    fast: float
    rrls: float


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        against: str
        directory: str


def wrap(content: Any) -> str:
    return f"\"{content}\""


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--against", type=str, default="problems/dell-amico/tsplib-results.csv")
    parser.add_argument("--directory", type=str, default="outputs/")
    args = parser.parse_args(namespace=Namespace())

    against = Path(args.against).resolve()
    directory = Path(args.directory).resolve()
    output_csv = directory / "summary.csv"
    output_db = directory / "summary.db"

    pattern = re.compile(r"^.+?-\w{8}(?<!solution)\.json$")

    with output_csv.open("w", encoding="utf-8") as csv:
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
            "Penalty exponent",
            "Energy model",
            "Speed type",
            "Range type",
            "Waiting time limit",
            "Truck maximum speed",
            "Endurance fixed time [s]",
            "Endurance drone speed [m/s]",
            "Cost [s]",
            "[Fast] cost [s]",
            "[RRLS] cost [s]",
            "Improved to [Fast] [%]",
            "Improved to [RRLS] [%]",
            "[Fast] performance [s]",
            "[RRLS] performance [s]",
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
            "Faster than [Fast] [%]",
            "Faster than [RRLS] [%]",
            "Weight per truck route [kg]",
            "Customers per truck route",
            "Truck routes count",
            "Weight per drone route [kg]",
            "Customers per drone route",
            "Drone routes count",
            "Strategy",
        ]
        csv.write(",".join(headers))
        csv.write("\n")

        if output_db.is_file():
            output_db.unlink()

        with sqlite3.connect(output_db) as connection:
            cursor = connection.cursor()

            columns = [
                "problem TEXT NOT NULL",
                "customers_count INTEGER NOT NULL",
                "trucks_count INTEGER NOT NULL",
                "drones_count INTEGER NOT NULL",
                "iterations INTEGER NOT NULL",
                "tabu_size_factor REAL NOT NULL",
                "reset_after_factor INTEGER NOT NULL",
                "tabu_size INTEGER NOT NULL",
                "reset_after INTEGER NOT NULL",
                "max_elite_set_size INTEGER NOT NULL",
                "penalty_exponent REAL NOT NULL",
                "energy_model TEXT NOT NULL",
                "speed_type TEXT NOT NULL",
                "range_type TEXT NOT NULL",
                "waiting_time_limit INTEGER NOT NULL",
                "truck_maximum_speed REAL",
                "endurance_fixed_time REAL",
                "endurance_drone_speed REAL",
                "cost REAL NOT NULL",
                "capacity_violation REAL NOT NULL",
                "energy_violation REAL NOT NULL",
                "waiting_time_violation REAL NOT NULL",
                "fixed_time_violation REAL NOT NULL",
                "truck_paths TEXT NOT NULL",
                "drone_paths TEXT NOT NULL",
                "truck_working_time TEXT NOT NULL",
                "drone_working_time TEXT NOT NULL",
                "feasible INTEGER NOT NULL",
                "last_improved INTEGER NOT NULL",
                "elapsed REAL NOT NULL",
                "url TEXT",
                "weight_per_truck_route REAL NOT NULL",
                "customers_per_truck_route REAL NOT NULL",
                "truck_route_count INTEGER NOT NULL",
                "weight_per_drone_route REAL NOT NULL",
                "customers_per_drone_route REAL NOT NULL",
                "drone_route_count INTEGER NOT NULL",
                "strategy TEXT NOT NULL",
                "el INTEGER NOT NULL",
                "sp INTEGER NOT NULL",
                "dc INTEGER NOT NULL",
                "dp INTEGER NOT NULL",
            ]

            query = "CREATE TABLE summary(" + ", ".join(columns) + ")"
            cursor.execute(query)

            query = "INSERT INTO summary VALUES (" + ", ".join(itertools.repeat("?", len(columns))) + ")"

            dell_amico: Dict[DellAmicoKey, DellAmicoResult] = {}
            with against.open("r", encoding="utf-8") as amico:
                for line in amico.readlines()[2:]:
                    problem, el, sp, dc, dp, best, fast, rrls = map(str.strip, line.split(","))
                    key = DellAmicoKey(problem, el, sp, dc, dp)
                    dell_amico[key] = DellAmicoResult(float(best), float(fast), float(rrls))

            row = 2
            for (dirpath, _, filenames) in directory.walk():
                filenames.sort()
                for filename in filenames:
                    if not pattern.fullmatch(filename):
                        continue

                    with open(dirpath / filename, "r", encoding="utf-8") as reader:
                        data = json.load(reader)

                    problem, el, sp, dc, dp, *_ = data["problem"].split("_")

                    truck_routes = data["solution"]["truck_routes"]
                    drone_routes = data["solution"]["drone_routes"]

                    truck_route_count = sum(len(routes) for routes in truck_routes)
                    drone_route_count = sum(len(routes) for routes in drone_routes)

                    config = data["config"]
                    truck_weight = sum(sum(sum(config["demands"][c] for c in route) for route in routes) for routes in truck_routes)
                    drone_weight = sum(sum(sum(config["demands"][c] for c in route) for route in routes) for routes in drone_routes)
                    truck_customers = sum(sum(len(route) - 2 for route in routes) for routes in truck_routes)
                    drone_customers = sum(sum(len(route) - 2 for route in routes) for routes in drone_routes)

                    compare = dell_amico[DellAmicoKey(problem, el, sp, dc, dp)]
                    segments = [
                        wrap(f"{problem}_{el}_{sp}_{dc}_{dp}"),
                        str(truck_customers + drone_customers),
                        str(config["trucks_count"]),
                        str(config["drones_count"]),
                        str(data["iterations"]),
                        str(config["tabu_size_factor"]),
                        str(config["reset_after_factor"]),
                        str(data["tabu_size"]),
                        str(data["reset_after"]),
                        str(config["max_elite_size"]),
                        str(config["penalty_exponent"]),
                        config["config"],
                        config["speed_type"],
                        config["range_type"],
                        str(config["waiting_time_limit"]),
                        str(config["truck"]["V_max (m/s)"]),
                        str(config["drone"]["_data"].get("FixedTime (s)", -1)),
                        str(config["drone"]["_data"].get("V_max (m/s)", -1)),
                        str(data["solution"]["working_time"]),
                        str(compare.fast),
                        str(compare.rrls),
                        wrap(f"=ROUND(100 * (T{row} - S{row}) / ABS(T{row}), 2)"),
                        wrap(f"=ROUND(100 * (U{row} - S{row}) / ABS(U{row}), 2)"),
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
                        wrap(f"=ROUND(100 * (X{row} - AJ{row}) / ABS(X{row}), 2)"),
                        wrap(f"=ROUND(100 * (Y{row} - AJ{row}) / ABS(Y{row}), 2)"),
                        str(truck_weight / truck_route_count if truck_route_count > 0 else 0),
                        str(truck_customers / truck_route_count if truck_route_count > 0 else 0),
                        str(truck_route_count),
                        str(drone_weight / drone_route_count if drone_route_count > 0 else 0),
                        str(drone_customers / drone_route_count if drone_route_count > 0 else 0),
                        str(drone_route_count),
                        config["strategy"],
                    ]
                    csv.write(",".join(segments) + "\n")
                    row += 1

                    cursor.execute(
                        query,
                        (
                            problem,
                            truck_customers + drone_customers,
                            config["trucks_count"],
                            config["drones_count"],
                            data["iterations"],
                            config["tabu_size_factor"],
                            config["reset_after_factor"],
                            data["tabu_size"],
                            data["reset_after"],
                            config["max_elite_size"],
                            config["penalty_exponent"],
                            config["config"],
                            config["speed_type"],
                            config["range_type"],
                            config["waiting_time_limit"],
                            config["truck"]["V_max (m/s)"],
                            config["drone"]["_data"].get("FixedTime (s)", -1),
                            config["drone"]["_data"].get("V_max (m/s)", -1),
                            data["solution"]["working_time"] / 60,
                            data["solution"]["capacity_violation"],
                            data["solution"]["energy_violation"],
                            data["solution"]["waiting_time_violation"],
                            data["solution"]["fixed_time_violation"],
                            str(data["solution"]["truck_routes"]),
                            str(data["solution"]["drone_routes"]),
                            str(data["solution"]["truck_working_time"]),
                            str(data["solution"]["drone_working_time"]),
                            int(data["solution"]["feasible"]),
                            data["last_improved"],
                            data["elapsed"],
                            config["extra"],
                            truck_weight / truck_route_count if truck_route_count > 0 else 0,
                            truck_customers / truck_route_count if truck_route_count > 0 else 0,
                            truck_route_count,
                            drone_weight / drone_route_count if drone_route_count > 0 else 0,
                            drone_customers / drone_route_count if drone_route_count > 0 else 0,
                            drone_route_count,
                            config["strategy"],
                            el,
                            sp,
                            dc,
                            dp,
                        )
                    )
