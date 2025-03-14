use std::fs;
use std::sync::LazyLock;

use clap::Parser;
use regex::{Regex, RegexBuilder};
use serde::{Deserialize, Serialize};

use crate::cli;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TruckConfig {
    #[serde(rename = "V_max (m/s)")]
    pub speed: f64,

    #[serde(rename = "M_t (kg)")]
    pub capacity: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct _LinearJSON {
    #[serde(rename = "takeoffSpeed [m/s]")]
    takeoff_speed: f64,

    #[serde(rename = "cruiseSpeed [m/s]")]
    cruise_speed: f64,

    #[serde(rename = "landingSpeed [m/s]")]
    landing_speed: f64,

    #[serde(rename = "cruiseAlt [m]")]
    altitude: f64,

    #[serde(rename = "capacity [kg]")]
    capacity: f64,

    #[serde(rename = "batteryPower [Joule]")]
    battery: f64,

    speed_type: cli::ConfigType,
    range_type: cli::ConfigType,

    #[serde(rename = "beta(w/kg)")]
    beta: f64,

    #[serde(rename = "gamma(w)")]
    gamma: f64,
}

#[derive(Debug, Deserialize)]
struct _LinearFileJSON {
    #[serde(rename = "1")]
    item1: _LinearJSON,

    #[serde(rename = "2")]
    item2: _LinearJSON,

    #[serde(rename = "3")]
    item3: _LinearJSON,

    #[serde(rename = "4")]
    item4: _LinearJSON,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct _NonLinearJSON {
    #[serde(rename = "takeoffSpeed [m/s]")]
    takeoff_speed: f64,

    #[serde(rename = "cruiseSpeed [m/s]")]
    cruise_speed: f64,

    #[serde(rename = "landingSpeed [m/s]")]
    landing_speed: f64,

    #[serde(rename = "cruiseAlt [m]")]
    altitude: f64,

    #[serde(rename = "capacity [kg]")]
    capacity: f64,

    #[serde(rename = "batteryPower [Joule]")]
    battery: f64,

    speed_type: cli::ConfigType,
    range_type: cli::ConfigType,
}

#[derive(Debug, Deserialize)]
struct _NonLinearFileJSON {
    #[serde(rename = "1")]
    item1: _NonLinearJSON,

    #[serde(rename = "2")]
    item2: _NonLinearJSON,

    #[serde(rename = "3")]
    item3: _NonLinearJSON,

    #[serde(rename = "4")]
    item4: _NonLinearJSON,

    k1: f64,

    #[serde(rename = "k2 (sqrt(kg/m))")]
    k2: f64,

    #[serde(rename = "c1 (sqrt(m/kg))")]
    c1: f64,

    #[serde(rename = "c2 (sqrt(m/kg))")]
    c2: f64,

    #[serde(rename = "c4 (kg/m)")]
    c4: f64,

    #[serde(rename = "c5 (Ns/m)")]
    c5: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct _EnduranceJSON {
    speed_type: cli::ConfigType,
    range_type: cli::ConfigType,

    #[serde(rename = "capacity [kg]")]
    capacity: f64,

    #[serde(rename = "FixedTime (s)")]
    fixed_time: f64,

    #[serde(rename = "V_max (m/s)")]
    speed: f64,
}

#[derive(Debug, Deserialize)]
struct _EnduranceFileJSON {
    #[serde(rename = "1")]
    item1: _EnduranceJSON,

    #[serde(rename = "2")]
    item2: _EnduranceJSON,

    #[serde(rename = "3")]
    item3: _EnduranceJSON,

    #[serde(rename = "4")]
    item4: _EnduranceJSON,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "config")]
pub enum DroneConfig {
    Linear {
        _data: _LinearJSON,
        _takeoff_time: f64,
        _landing_time: f64,
    },
    NonLinear {
        _data: _NonLinearJSON,
        _vert_k1: f64,
        _vert_k2: f64,
        _vert_c2: f64,
        _vert_half_takeoff: f64,
        _vert_half_landing: f64,
        _vert_half_takeoff_2: f64,
        _vert_half_landing_2: f64,
        _hori_c12: f64,
        _hori_c4v3: f64,
        _hori_c42v4: f64,
        _hori_c5: f64,
        _takeoff_time: f64,
        _landing_time: f64,
    },
    Endurance {
        _data: _EnduranceJSON,
    },
}

impl DroneConfig {
    const W: f64 = 1.5;
    const G: f64 = 9.8;

    fn new(
        config: cli::EnergyModel,
        speed_type: cli::ConfigType,
        range_type: cli::ConfigType,
    ) -> DroneConfig {
        match config {
            cli::EnergyModel::Linear => {
                let data = serde_json::from_str::<_LinearFileJSON>(include_str!(
                    "../problems/config_parameter/drone_linear_config.json"
                ))
                .unwrap();

                for config in [data.item1, data.item2, data.item3, data.item4] {
                    if config.speed_type == speed_type && config.range_type == range_type {
                        let _takeoff_time = config.altitude / config.takeoff_speed;
                        let _landing_time = config.altitude / config.landing_speed;
                        return Self::Linear {
                            _data: config,
                            _takeoff_time,
                            _landing_time,
                        };
                    }
                }

                panic!("No matching linear config")
            }
            cli::EnergyModel::NonLinear => {
                let data = serde_json::from_str::<_NonLinearFileJSON>(include_str!(
                    "../problems/config_parameter/drone_nonlinear_config.json"
                ))
                .unwrap();

                for config in [data.item1, data.item2, data.item3, data.item4] {
                    if config.speed_type == speed_type && config.range_type == range_type {
                        let _vert_k1 = data.k1 * Self::G;
                        let _vert_k2 = Self::G / (data.k2 * data.k2);
                        let _vert_c2 = data.c2 * Self::G.powf(1.5);
                        let _vert_half_takeoff: f64 = config.takeoff_speed / 2.0;
                        let _vert_half_landing = config.landing_speed / 2.0;
                        let _vert_half_takeoff_2 = _vert_half_takeoff * _vert_half_takeoff;
                        let _vert_half_landing_2 = _vert_half_landing * _vert_half_landing;
                        let _hori_c12 = data.c1 + data.c2;
                        let _hori_c4v3 = data.c4
                            * config.cruise_speed
                            * config.cruise_speed
                            * config.cruise_speed;
                        let _hori_c42v4 = data.c4
                            * data.c4
                            * config.cruise_speed
                            * config.cruise_speed
                            * config.cruise_speed
                            * config.cruise_speed;

                        let deg_10 = std::f64::consts::PI / 18.0;
                        let _hori_c5 = data.c5 * (config.cruise_speed * deg_10.cos()).powi(2);

                        let _takeoff_time = config.altitude / config.takeoff_speed;
                        let _landing_time = config.altitude / config.landing_speed;

                        return Self::NonLinear {
                            _data: config,
                            _vert_k1,
                            _vert_k2,
                            _vert_c2,
                            _vert_half_takeoff,
                            _vert_half_landing,
                            _vert_half_takeoff_2,
                            _vert_half_landing_2,
                            _hori_c12,
                            _hori_c4v3,
                            _hori_c42v4,
                            _hori_c5,
                            _takeoff_time,
                            _landing_time,
                        };
                    }
                }

                panic!("No matching non-linear config")
            }
            cli::EnergyModel::Endurance => {
                let data = serde_json::from_str::<_EnduranceFileJSON>(include_str!(
                    "../problems/config_parameter/drone_endurance_config.json"
                ))
                .unwrap();

                for config in [data.item1, data.item2, data.item3, data.item4] {
                    if config.speed_type == speed_type && config.range_type == range_type {
                        return Self::Endurance { _data: config };
                    }
                }

                panic!("No matching endurance config")
            }
            cli::EnergyModel::Unlimited => Self::Endurance {
                _data: _EnduranceJSON {
                    speed_type: cli::ConfigType::High,
                    range_type: cli::ConfigType::High,
                    capacity: std::f64::INFINITY,
                    fixed_time: std::f64::INFINITY,
                    speed: 1.0,
                },
            },
        }
    }

    pub fn capacity(&self) -> f64 {
        match self {
            Self::Linear { _data, .. } => _data.capacity,
            Self::NonLinear { _data, .. } => _data.capacity,
            Self::Endurance { _data, .. } => _data.capacity,
        }
    }

    pub fn battery(&self) -> f64 {
        match self {
            Self::Linear { _data, .. } => _data.battery,
            Self::NonLinear { _data, .. } => _data.battery,
            Self::Endurance { .. } => 1.0,
        }
    }

    pub fn fixed_time(&self) -> f64 {
        match self {
            Self::Linear { .. } => std::f64::INFINITY,
            Self::NonLinear { .. } => std::f64::INFINITY,
            Self::Endurance { _data, .. } => _data.fixed_time,
        }
    }

    pub fn takeoff_power(&self, weight: f64) -> f64 {
        match self {
            Self::Linear { _data, .. } => _data.beta * weight + _data.gamma,
            Self::NonLinear {
                _vert_k1,
                _vert_k2,
                _vert_c2,
                _vert_half_takeoff,
                _vert_half_takeoff_2,
                ..
            } => {
                let w = Self::W + weight;
                _vert_k1 * w * (_vert_half_takeoff + (_vert_half_takeoff_2 + _vert_k2 * w).sqrt())
                    + _vert_c2 * w.powf(1.5)
            }
            Self::Endurance { .. } => 0.0,
        }
    }

    pub fn landing_power(&self, weight: f64) -> f64 {
        match self {
            Self::Linear { _data, .. } => _data.beta * weight + _data.gamma,
            Self::NonLinear {
                _vert_k1,
                _vert_k2,
                _vert_c2,
                _vert_half_landing,
                _vert_half_landing_2,
                ..
            } => {
                let w = Self::W + weight;
                _vert_k1 * w * (_vert_half_landing + (_vert_half_landing_2 + _vert_k2 * w).sqrt())
                    + _vert_c2 * w.powf(1.5)
            }
            Self::Endurance { .. } => 0.0,
        }
    }

    pub fn cruise_power(&self, weight: f64) -> f64 {
        match self {
            Self::Linear { _data, .. } => _data.beta * weight + _data.gamma,
            Self::NonLinear {
                _hori_c12,
                _hori_c4v3,
                _hori_c42v4,
                _hori_c5,
                ..
            } => {
                let temp = (Self::W + weight) * Self::G - _hori_c5;
                _hori_c12 * (temp * temp + _hori_c42v4).powf(0.75) + _hori_c4v3
            }
            Self::Endurance { .. } => 0.0,
        }
    }

    pub fn takeoff_time(&self) -> f64 {
        match self {
            Self::Linear { _takeoff_time, .. } => *_takeoff_time,
            Self::NonLinear { _takeoff_time, .. } => *_takeoff_time,
            Self::Endurance { .. } => 0.0,
        }
    }

    pub fn landing_time(&self) -> f64 {
        match self {
            Self::Linear { _landing_time, .. } => *_landing_time,
            Self::NonLinear { _landing_time, .. } => *_landing_time,
            Self::Endurance { .. } => 0.0,
        }
    }

    pub fn cruise_time(&self, distance: f64) -> f64 {
        match self {
            Self::Linear { _data, .. } => distance / _data.cruise_speed,
            Self::NonLinear { _data, .. } => distance / _data.cruise_speed,
            Self::Endurance { _data, .. } => distance / _data.speed,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Config {
    pub customers_count: usize,
    pub trucks_count: usize,
    pub drones_count: usize,

    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub demands: Vec<f64>,
    pub dronable: Vec<bool>,
    pub distances: Vec<Vec<f64>>,

    pub truck: TruckConfig,
    pub drone: DroneConfig,

    pub problem: String,
    pub config: cli::EnergyModel,
    pub tabu_size_factor: f64,
    pub speed_type: cli::ConfigType,
    pub range_type: cli::ConfigType,
    pub waiting_time_limit: f64,
    pub strategy: cli::Strategy,
    pub fix_iteration: Option<usize>,
    pub reset_after_factor: usize,
    pub max_elite_size: usize,
    pub verbose: bool,
    pub outputs: String,
}

pub static CONFIG: LazyLock<Config> = LazyLock::new(|| {
    let arguments = cli::Arguments::parse();
    match arguments.command {
        cli::Commands::Evaluate { config, .. } => {
            let data = fs::read_to_string(config).unwrap();
            serde_json::from_str::<Config>(&data).unwrap()
        }
        cli::Commands::Run {
            problem,
            config,
            tabu_size_factor,
            speed_type,
            range_type,
            trucks_count,
            drones_count,
            waiting_time_limit,
            strategy,
            fix_iteration,
            reset_after_factor,
            max_elite_size,
            verbose,
            outputs,
        } => {
            let trucks_count_regex = Regex::new(r"number_truck (\d+)").unwrap();
            let drones_count_regex = Regex::new(r"number_drone (\d+)").unwrap();
            let customers_regex = RegexBuilder::new(r"^(-?[\d\.]+)\s+(-?[\d\.]+)\s+([\d\.]+)$")
                .multi_line(true)
                .build()
                .unwrap();

            let data = fs::read_to_string(&problem).unwrap();

            let trucks_count = trucks_count
                .or_else(|| {
                    trucks_count_regex
                        .captures(&data)
                        .and_then(|caps| caps.get(1))
                        .and_then(|m| m.as_str().parse::<usize>().ok())
                })
                .expect("Missing trucks count");
            let drones_count = drones_count
                .or_else(|| {
                    drones_count_regex
                        .captures(&data)
                        .and_then(|caps| caps.get(1))
                        .and_then(|m| m.as_str().parse::<usize>().ok())
                })
                .expect("Missing drones count");

            let mut customers_count = 0;
            let mut x = vec![0.0];
            let mut y = vec![0.0];
            let mut demands = vec![0.0];
            let mut dronable = vec![true];
            for c in customers_regex.captures_iter(&data) {
                customers_count += 1;

                let (_, [_x, _y, _demand]) = c.extract::<3>();
                x.push(_x.parse::<f64>().unwrap());
                y.push(_y.parse::<f64>().unwrap());
                demands.push(_demand.parse::<f64>().unwrap());
                dronable.push(true);
            }

            let mut distances = vec![vec![0.0; customers_count + 1]; customers_count + 1];
            for i in 0..customers_count + 1 {
                for j in 0..customers_count + 1 {
                    distances[i][j] = ((x[i] - x[j]).powi(2) + (y[i] - y[j]).powi(2)).sqrt();
                }
            }

            let truck = serde_json::from_str::<TruckConfig>(include_str!(
                "../problems/config_parameter/truck_config.json"
            ))
            .unwrap();
            let drone = DroneConfig::new(config, speed_type, range_type);

            Config {
                customers_count,
                trucks_count,
                drones_count,
                x,
                y,
                demands,
                dronable,
                distances,
                truck,
                drone,
                problem,
                config,
                tabu_size_factor,
                speed_type,
                range_type,
                waiting_time_limit,
                strategy,
                fix_iteration,
                reset_after_factor,
                max_elite_size,
                verbose,
                outputs,
            }
        }
    }
});
