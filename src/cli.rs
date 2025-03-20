use std::fmt;

use clap::{Parser, Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Deserialize, Serialize)]
pub enum EnergyModel {
    #[serde(rename = "linear")]
    Linear = 0,
    #[serde(rename = "non-linear")]
    NonLinear = 1,
    #[serde(rename = "endurance")]
    Endurance = 2,
    #[serde(rename = "unlimited")]
    Unlimited = 3,
}

impl fmt::Display for EnergyModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::Linear => "linear",
                Self::NonLinear => "non-linear",
                Self::Endurance => "endurance",
                Self::Unlimited => "unlimited",
            }
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Deserialize, Serialize)]
pub enum ConfigType {
    #[serde(rename = "low")]
    Low,
    #[serde(rename = "high")]
    High,
}

impl fmt::Display for ConfigType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::Low => "low",
                Self::High => "high",
            }
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Deserialize, Serialize)]
pub enum Strategy {
    #[serde(rename = "random")]
    Random,
    #[serde(rename = "cyclic")]
    Cyclic,
    #[serde(rename = "vns")]
    Vns,
}

impl fmt::Display for Strategy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::Random => "random",
                Self::Cyclic => "cyclic",
                Self::Vns => "vns",
            }
        )
    }
}

#[derive(Debug, Parser)]
#[command(
    long_about = "The min-timespan parallel technician-and-drone scheduling in door-to-door sampling service system",
    propagate_version = true,
    version
)]
pub struct Arguments {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Debug, Subcommand)]
pub enum Commands {
    /// Evaluate an existing solution
    Evaluate {
        /// Path to the solution JSON file
        solution: String,

        /// Path to the config JSON file
        config: String,
    },

    /// Run the algorithm
    Run {
        /// Path to the coordinate file
        problem: String,

        /// The energy consumption model to use.
        #[arg(short, long, default_value_t = EnergyModel::Endurance)]
        config: EnergyModel,

        /// Tabu size of each neighborhood, final value = a1 * base
        #[arg(long, default_value_t = 1.0)]
        tabu_size_factor: f64,

        /// Speed type of drones.
        #[arg(long, default_value_t = ConfigType::High)]
        speed_type: ConfigType,

        /// Range type of drones.
        #[arg(long, default_value_t = ConfigType::High)]
        range_type: ConfigType,

        /// The number of trucks to override. Otherwise, use the default value.
        #[arg(long)]
        trucks_count: Option<usize>,

        /// The number of drones to override. Otherwise, use the default value.
        #[arg(long)]
        drones_count: Option<usize>,

        /// The waiting time limit for each customer (in seconds).
        #[arg(long, default_value_t = 3600.0)]
        waiting_time_limit: f64,

        /// Tabu search neighborhood selection strategy.
        #[arg(long, default_value_t = Strategy::Random)]
        strategy: Strategy,

        /// Fix the number of iterations and disable elite set extraction. Otherwise, run until the elite set is exhausted.
        #[arg(long)]
        fix_iteration: Option<usize>,

        /// The number of non-improved iterations before resetting the current solution = a2 * base
        #[arg(long, default_value_t = 30.0)]
        reset_after_factor: f64,

        /// The maximum size of the elite set = a3
        #[arg(long, default_value_t = 10)]
        max_elite_size: usize,

        /// The verbose mode
        #[arg(short, long)]
        verbose: bool,

        /// The directory to store results
        #[arg(long, default_value_t = String::from("outputs/"))]
        outputs: String,

        /// Extra data to store in the output JSON
        #[arg(long, default_value_t = String::new())]
        extra: String,
    },
}
