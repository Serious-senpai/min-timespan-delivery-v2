from __future__ import annotations

import argparse
import itertools
import json
import re
import sqlite3
from pathlib import Path
from typing import Any, TYPE_CHECKING


compare = {
    "100.10.1": 22.31298872,
    "100.10.2": 23.54298188,
    "100.10.3": 22.71699385,
    "100.10.4": 20.8940764,
    "100.20.1": 43.31821568,
    "100.20.2": 44.43574084,
    "100.20.3": 42.32780332,
    "100.20.4": 51.55685171,
    "50.10.1": 23.17082725,
    "50.10.2": 21.94129384,
    "50.10.3": 21.89158729,
    "50.10.4": 21.51980016,
    "50.20.1": 40.17998894,
    "50.20.2": 39.61689848,
    "50.20.3": 44.80287273,
    "50.20.4": 44.24140392,
    "50.30.1": 67.77205972,
    "50.30.2": 57.73853584,
    "50.30.3": 68.32947239,
    "50.30.4": 65.76833781,
    "50.40.1": 77.73020875,
    "50.40.2": 71.64303362,
    "50.40.3": 74.17431791,
    "50.40.4": 74.29395893,
    "100.40.1": 76.55174599,
    "100.40.2": 73.31744929,
    "100.40.3": 75.98348396,
    "100.40.4": 74.53894884,
    "100.30.1": 60.96531647,
    "100.30.2": 71.06449648,
    "100.30.3": 68.56574676,
    "100.30.4": 69.97970497,
    "12.5.4": 14.1259409,
    "12.5.3": 15.95365163,
    "12.5.2": 12.76825485,
    "12.5.1": 11.55691994,
    "12.20.4": 56.8747253,
    "12.20.3": 46.53667832,
    "12.20.2": 64.05994714,
    "12.20.1": 62.86540994,
    "12.10.4": 29.30635618,
    "12.10.3": 24.43233875,
    "12.10.2": 26.78965929,
    "12.10.1": 27.38099396,
    "10.10.1": 29.01788937,
    "10.10.2": 24.91170315,
    "10.10.3": 27.20634104,
    "10.10.4": 23.78547733,
    "10.20.1": 45.52591438,
    "10.20.2": 49.93467305,
    "10.20.3": 48.15341278,
    "10.20.4": 47.26370097,
    "10.5.1": 12.59377765,
    "10.5.2": 12.80391271,
    "10.5.3": 13.55135845,
    "10.5.4": 13.82523797,
    "6.5.4": 10.95280956,
    "6.5.3": 11.53104063,
    "6.5.2": 9.831135248,
    "6.5.1": 10.96656168,
    "6.20.4": 46.32968456,
    "6.20.3": 46.39377808,
    "6.20.2": 39.02185823,
    "6.20.1": 37.33445961,
    "6.10.4": 16.7263406,
    "6.10.3": 21.92761867,
    "6.10.2": 20.66085034,
    "6.10.1": 17.75020497,
    "20.5.4": 9.332257646,
    "20.5.3": 8.647846578,
    "20.5.2": 10.61026306,
    "20.5.1": 10.49338509,
    "20.20.4": 40.16791962,
    "20.20.3": 42.38956653,
    "20.20.2": 36.90456454,
    "20.20.1": 39.40074399,
    "20.10.4": 21.78908972,
    "20.10.3": 20.79406438,
    "20.10.2": 20.08803647,
    "20.10.1": 21.19339045,
}


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        directory: Path


def wrap(content: Any) -> str:
    return f"\"{content}\""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--directory", type=Path, default="outputs/")
    namespace = parser.parse_args(namespace=Namespace())

    directory = namespace.directory
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
            "Adaptive segments",
            "Total adaptive segments",
            "Adaptive iterations",
            "Actual adaptive iterations",
            "Tabu size",
            "Reset after",
            "Max elite set size",
            "Penalty exponent",
            "Ejection-chain iterations",
            "Destroy rate",
            "Energy model",
            "Speed type",
            "Range type",
            "Waiting time limit",
            "Truck maximum speed",
            "Endurance fixed time [s]",
            "Endurance drone speed [m/s]",
            "Cost [minute]",
            "PTDS-DDS cost",
            "Improved [%]",
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
            "Weight per truck route [kg]",
            "Customers per truck route",
            "Truck routes count",
            "Weight per drone route [kg]",
            "Customers per drone route",
            "Drone routes count",
            "Strategy",
            "Post-optimization [minute]",
            "Post-optimization elapsed [s]",
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
                "adaptive_segments INTEGER NOT NULL",
                "total_adaptive_segments INTEGER NOT NULL",
                "adaptive_iterations INTEGER NOT NULL",
                "actual_adaptive_iterations INTEGER NOT NULL",
                "tabu_size INTEGER NOT NULL",
                "reset_after INTEGER NOT NULL",
                "max_elite_set_size INTEGER NOT NULL",
                "penalty_exponent REAL NOT NULL",
                "ejection_chain_iterations INTEGER NOT NULL",
                "destroy_rate REAL NOT NULL",
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
            ]

            query = "CREATE TABLE summary(" + ", ".join(columns) + ")"
            cursor.execute(query)

            query = "INSERT INTO summary VALUES (" + ", ".join(itertools.repeat("?", len(columns))) + ")"

            row = 2
            for (dirpath, _, filenames) in directory.walk():
                filenames.sort()
                for filename in filenames:
                    if not pattern.fullmatch(filename):
                        continue

                    with open(dirpath / filename, "r", encoding="utf-8") as reader:
                        data = json.load(reader)

                    problem = data["problem"]

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
                        wrap(problem),
                        str(truck_customers + drone_customers),
                        str(config["trucks_count"]),
                        str(config["drones_count"]),
                        str(data["iterations"]),
                        str(config["tabu_size_factor"]),
                        str(config["reset_after_factor"]),
                        str(config["adaptive_segments"]),
                        str(data["total_adaptive_segments"]),
                        str(config["adaptive_iterations"]),
                        str(data["actual_adaptive_iterations"]),
                        str(data["tabu_size"]),
                        str(data["reset_after"]),
                        str(config["max_elite_size"]),
                        str(config["penalty_exponent"]),
                        str(config["ejection_chain_iterations"]),
                        str(config["destroy_rate"]),
                        config["config"],
                        config["speed_type"],
                        config["range_type"],
                        str(config["waiting_time_limit"]),
                        str(config["truck"]["V_max (m/s)"]),
                        str(config["drone"]["_data"].get("FixedTime (s)", -1)),
                        str(config["drone"]["_data"].get("V_max (m/s)", -1)),
                        str(data["solution"]["working_time"] / 60),
                        str(compare[problem]),
                        wrap(f"=ROUND(100 * (Z{row} - Y{row}) / ABS(Z{row}), 2)"),
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
                        str(truck_weight / truck_route_count if truck_route_count > 0 else 0),
                        str(truck_customers / truck_route_count if truck_route_count > 0 else 0),
                        str(truck_route_count),
                        str(drone_weight / drone_route_count if drone_route_count > 0 else 0),
                        str(drone_customers / drone_route_count if drone_route_count > 0 else 0),
                        str(drone_route_count),
                        config["strategy"],
                        str(data["post_optimization"] / 60),
                        str(data["post_optimization_elapsed"]),
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
                            config["adaptive_segments"],
                            data["total_adaptive_segments"],
                            config["adaptive_iterations"],
                            data["actual_adaptive_iterations"],
                            data["tabu_size"],
                            data["reset_after"],
                            config["max_elite_size"],
                            config["penalty_exponent"],
                            config["ejection_chain_iterations"],
                            config["destroy_rate"],
                            config["config"],
                            config["speed_type"],
                            config["range_type"],
                            config["waiting_time_limit"],
                            config["truck"]["V_max (m/s)"],
                            config["drone"]["_data"].get("FixedTime (s)", -1),
                            config["drone"]["_data"].get("V_max (m/s)", -1),
                            data["solution"]["working_time"],
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
                        )
                    )
