use std::fs;

use clap::Parser;
use colored::Colorize;
use routes::Route;

mod cli;
mod clusterize;
mod config;
mod errors;
mod logger;
mod neighborhoods;
mod routes;
mod solutions;

fn main() {
    let mut logger = logger::Logger::new().unwrap();

    let solution = match cli::Arguments::parse().command {
        cli::Commands::Evaluate { solution, .. } => {
            let data = fs::read_to_string(solution).unwrap();

            // Note: Solution `s` here contains attributes calculated using its old config.
            // In order to evaluate `s` with the new config, we construct a new solution.
            let s = serde_json::from_str::<solutions::Solution>(&data).unwrap();

            let mut truck_routes = vec![vec![]; s.truck_routes.len()];
            for (truck, routes) in s.truck_routes.into_iter().enumerate() {
                for route in routes {
                    let new = routes::TruckRoute::new(route.data().customers.clone());
                    truck_routes[truck].push(new);
                }
            }

            let mut drone_routes = vec![vec![]; s.drone_routes.len()];
            for (drone, routes) in s.drone_routes.into_iter().enumerate() {
                for route in routes {
                    let new = routes::DroneRoute::new(route.data().customers.clone());
                    drone_routes[drone].push(new);
                }
            }

            let s = solutions::Solution::new(truck_routes, drone_routes);
            logger.finalize(&s, 0, 0, 0, 0, 0, 0.0).unwrap();
            s
        }
        cli::Commands::Run { .. } => {
            let root = solutions::Solution::initialize();
            solutions::Solution::tabu_search(root, &mut logger)
        }
    };

    eprintln!("{}", format!("Result = {}", solution.working_time).red());
    solution.verify();
}
