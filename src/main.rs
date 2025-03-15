use std::fs;
use std::rc::Rc;

use clap::Parser;

use crate::routes::Route;

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
            serde_json::from_str::<solutions::Solution>(&data).unwrap()
        }
        cli::Commands::Run { .. } => {
            let root = solutions::Solution::initialize();
            solutions::Solution::tabu_search(root, &mut logger)
        }
    };

    for routes in &solution.truck_routes {
        for route in routes {
            for neighborhood in [
                neighborhoods::Neighborhood::Move10,
                neighborhoods::Neighborhood::Move11,
                neighborhoods::Neighborhood::Move20,
                neighborhoods::Neighborhood::Move21,
                neighborhoods::Neighborhood::Move22,
                neighborhoods::Neighborhood::TwoOpt,
            ] {
                route.intra_route(neighborhood);
            }
        }
    }

    logger.finalize(&solution).unwrap();
}
