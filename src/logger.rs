use std::error::Error;
use std::fs::{self, File};
use std::io;
use std::io::Write;
use std::path::Path;
use std::rc::Rc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rand::distr::Alphanumeric;
use rand::Rng;

use crate::config::{SerializedConfig, CONFIG};
use crate::errors::ExpectedValue;
use crate::neighborhoods::Neighborhood;
use crate::routes::Route;
use crate::solutions::{penalty_coeff, Solution};

#[derive(serde::Serialize)]
struct RunJSON<'a> {
    problem: String,
    tabu_size: usize,
    reset_after: usize,
    iterations: usize,
    actual_adaptive_iterations: usize,
    adaptive_segments: usize,
    solution: &'a Solution,
    config: &'a SerializedConfig,
    last_improved: usize,
    elapsed: f64,
}

pub struct Logger<'a> {
    _iteration: usize,
    _time_offset: Duration,

    _outputs: &'a Path,
    _problem: String,
    _id: String,
    _writer: Option<File>,
}

impl Logger<'_> {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let outputs = Path::new(&CONFIG.outputs);
        if !outputs.is_dir() {
            fs::create_dir_all(outputs)?;
        }

        let problem = ExpectedValue::cast(
            Path::new(&CONFIG.problem)
                .file_stem()
                .and_then(|f| f.to_os_string().into_string().ok()),
        )?;
        let id = rand::rng()
            .sample_iter(&Alphanumeric)
            .take(8)
            .map(char::from)
            .collect::<String>();

        let mut writer = if CONFIG.disable_logging {
            None
        } else {
            Some(File::create(outputs.join(format!("{}-{}.csv", problem, id)))?)
        };

        if let Some(ref mut writer) = writer {
            eprintln!("Logging iterations to {:?}", writer);

            let columns = vec![
                "Iteration",
                "Cost",
                "Working time",
                "Feasible",
                "p0",
                "Energy violation",
                "p1",
                "Capacity violation",
                "p2",
                "Waiting time violation",
                "p3",
                "Fixed time violation",
                "Truck routes",
                "Drone routes",
                "Truck routes count",
                "Drone routes count",
                "Neighborhood",
                "Tabu list",
            ]
            .join(",");
            writeln!(writer, "sep=,\n{}", columns)?;
        }

        Ok(Logger {
            _iteration: 0,
            _time_offset: SystemTime::now().duration_since(UNIX_EPOCH).unwrap(),
            _outputs: outputs,
            _id: id,
            _problem: problem,
            _writer: writer,
        })
    }

    pub fn log(
        &mut self,
        solution: &Solution,
        neighbor: Neighborhood,
        tabu_list: &Vec<Vec<usize>>,
    ) -> Result<(), io::Error> {
        fn _wrap(content: &String) -> String {
            format!("\"{}\"", content)
        }

        fn _expand_routes<T>(routes: &[Vec<Rc<T>>]) -> Vec<Vec<&Vec<usize>>>
        where
            T: Route,
        {
            routes
                .iter()
                .map(|r| r.iter().map(|x| &x.data().customers).collect())
                .collect()
        }

        self._iteration += 1;
        if let Some(ref mut writer) = self._writer {
            writeln!(
                writer,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                self._iteration,
                solution.cost(),
                solution.working_time,
                i32::from(solution.feasible),
                penalty_coeff::<0>(),
                solution.energy_violation,
                penalty_coeff::<1>(),
                solution.capacity_violation,
                penalty_coeff::<2>(),
                solution.waiting_time_violation,
                penalty_coeff::<3>(),
                solution.fixed_time_violation,
                _wrap(&format!("{:?}", _expand_routes(&solution.truck_routes))),
                _wrap(&format!("{:?}", _expand_routes(&solution.drone_routes))),
                solution.truck_routes.iter().map(|r| r.len()).sum::<usize>(),
                solution.drone_routes.iter().map(|r| r.len()).sum::<usize>(),
                _wrap(&neighbor.to_string()),
                _wrap(&format!("{:?}", tabu_list)),
            )?;
        }

        Ok(())
    }

    pub fn finalize(
        &self,
        result: &Solution,
        tabu_size: usize,
        reset_after: usize,
        actual_adaptive_iterations: usize,
        adaptive_segments: usize,
        last_improved: usize,
    ) -> Result<(), Box<dyn Error>> {
        let elapsed = SystemTime::now().duration_since(UNIX_EPOCH).unwrap() - self._time_offset;
        let serialized_config = SerializedConfig::from(CONFIG.clone());

        let json_path = self._outputs.join(format!("{}-{}.json", self._problem, self._id));
        let mut json = File::create(&json_path)?;
        println!("{}", json_path.display());
        json.write_all(
            serde_json::to_string(&RunJSON {
                problem: self._problem.clone(),
                tabu_size,
                reset_after,
                iterations: self._iteration,
                actual_adaptive_iterations,
                adaptive_segments,
                solution: result,
                config: &serialized_config,
                last_improved,
                elapsed: elapsed.as_micros() as f64 / 1e6,
            })?
            .as_bytes(),
        )?;

        let json_path = self
            ._outputs
            .join(format!("{}-{}-solution.json", self._problem, self._id));
        let mut json = File::create(&json_path)?;
        println!("{}", json_path.display());
        json.write_all(serde_json::to_string(&result)?.as_bytes())?;

        let json_path = self
            ._outputs
            .join(format!("{}-{}-config.json", self._problem, self._id));
        let mut json = File::create(&json_path)?;
        println!("{}", json_path.display());
        json.write_all(serde_json::to_string(&serialized_config)?.as_bytes())?;

        Ok(())
    }
}
