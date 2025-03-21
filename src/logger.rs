use std::error::Error;
use std::fs::File;
use std::io;
use std::io::Write;
use std::path::Path;
use std::rc::Rc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rand::distr::Alphanumeric;
use rand::Rng;

use crate::config::{Config, CONFIG};
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
    solution: &'a Solution,
    config: &'a Config,
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
            std::fs::create_dir_all(outputs)?;
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
            Some(File::create(
                outputs.join(format!("{}-{}.csv", problem, id)),
            )?)
        };

        if let Some(ref mut writer) = writer {
            writer.write_all(
                "sep=,\nIteration,p0,p1,p2,p3,Truck routes,Drone routes,Neighborhood\n".as_bytes(),
            )?;
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

    pub fn log(&mut self, solution: &Solution, neighbor: Neighborhood) -> Result<(), io::Error> {
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
            writer.write_all(
                format!(
                    "{},{},{},{},{},{},{},{}\n",
                    self._iteration,
                    penalty_coeff::<0>(),
                    penalty_coeff::<1>(),
                    penalty_coeff::<2>(),
                    penalty_coeff::<3>(),
                    _wrap(&format!("{:?}", _expand_routes(&solution.truck_routes))),
                    _wrap(&format!("{:?}", _expand_routes(&solution.drone_routes))),
                    _wrap(&neighbor.to_string()),
                )
                .as_bytes(),
            )?;
        }

        Ok(())
    }

    pub fn finalize(
        &self,
        result: &Solution,
        tabu_size: usize,
        reset_after: usize,
        last_improved: usize,
    ) -> Result<(), Box<dyn Error>> {
        let elapsed = SystemTime::now().duration_since(UNIX_EPOCH).unwrap() - self._time_offset;

        let mut json = File::create(
            self._outputs
                .join(format!("{}-{}.json", self._problem, self._id)),
        )?;
        println!("Writing summary to {:?}", json);
        json.write_all(
            serde_json::to_string(&RunJSON {
                problem: self._problem.clone(),
                tabu_size,
                reset_after,
                iterations: self._iteration,
                solution: result,
                config: &CONFIG,
                last_improved,
                elapsed: elapsed.as_micros() as f64 / 1e6,
            })?
            .as_bytes(),
        )?;

        let mut json = File::create(
            self._outputs
                .join(format!("{}-{}-solution.json", self._problem, self._id)),
        )?;
        println!("Writing solution to {:?}", json);
        json.write_all(serde_json::to_string(&result)?.as_bytes())?;

        let mut json = File::create(
            self._outputs
                .join(format!("{}-{}-config.json", self._problem, self._id)),
        )?;
        println!("Writing config to {:?}", json);
        json.write_all(serde_json::to_string(&*CONFIG)?.as_bytes())?;

        Ok(())
    }
}
