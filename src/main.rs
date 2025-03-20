use std::fs;

use clap::Parser;
use colored::Colorize;

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
            let s = serde_json::from_str::<solutions::Solution>(&data).unwrap();
            logger
                .finalize(&s, usize::MAX, usize::MAX, usize::MAX)
                .unwrap();
            s
        }
        cli::Commands::Run { .. } => {
            let root = solutions::Solution::initialize();
            solutions::Solution::tabu_search(root, &mut logger)
        }
    };

    println!("{}", format!("Result = {}", solution.working_time).red());
}
