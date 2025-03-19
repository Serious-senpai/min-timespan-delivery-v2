use std::fs;

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
    let mut logger = logger::Logger::new().unwrap();

    match cli::Arguments::parse().command {
        cli::Commands::Evaluate { solution, .. } => {
            let data = fs::read_to_string(solution).unwrap();
            serde_json::from_str::<solutions::Solution>(&data).unwrap()
        }
        cli::Commands::Run { .. } => {
            let root = solutions::Solution::initialize();
            solutions::Solution::tabu_search(root, &mut logger)
        }
    };
}
