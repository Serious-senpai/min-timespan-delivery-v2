use std::fs;
use std::rc::Rc;

use clap::Parser;

mod cli;
mod clusterize;
mod config;
mod errors;
mod logger;
mod neighborhoods;
mod routes;
mod solutions;

fn main() {
    let logger = logger::Logger::new();

    let solution = match cli::Arguments::parse().command {
        cli::Commands::Evaluate { solution, .. } => {
            let data = fs::read_to_string(solution).unwrap();
            Rc::new(serde_json::from_str::<solutions::Solution>(&data).unwrap())
        }
        cli::Commands::Run { .. } => {
            let root = solutions::Solution::initialize();
            solutions::Solution::tabu_search(root)
        }
    };

    logger.finalize(&solution).unwrap();
}
