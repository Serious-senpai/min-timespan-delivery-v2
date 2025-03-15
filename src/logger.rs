use std::error::Error;
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;

use rand::distr::Alphanumeric;
use rand::Rng;

use crate::config::CONFIG;
use crate::errors::ExpectedValue;
use crate::neighborhoods::Neighborhood;
use crate::solutions::{penalty_coeff, Solution};

pub struct Logger<'a> {
    _iteration: usize,

    _outputs: &'a Path,
    _problem: String,
    _id: String,
    _writer: File,
}

impl Logger<'_> {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let outputs = Path::new(&CONFIG.outputs);
        if !outputs.is_dir() {
            std::fs::create_dir_all(outputs).expect("Cannot create output directory");
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

        let mut writer = File::create(outputs.join(format!("{}-{}.csv", problem, id)))
            .expect("Cannot create logging CSV file");
        writer.write("sep=,\nIteration,p0,p1,p2,p3,Neighborhood\n".as_bytes())?;

        Ok(Logger {
            _iteration: 0,
            _outputs: outputs,
            _id: id,
            _problem: problem,
            _writer: writer,
        })
    }

    pub fn log(&mut self, solution: &Solution, neighbor: Neighborhood) -> Result<usize, io::Error> {
        fn _wrap(content: &String) -> String {
            format!("\"{}\"", content)
        }

        self._iteration += 1;
        self._writer.write(
            format!(
                "{},{},{},{},{},{}\n",
                self._iteration,
                penalty_coeff::<0>(),
                penalty_coeff::<1>(),
                penalty_coeff::<2>(),
                penalty_coeff::<3>(),
                _wrap(&neighbor.to_string()),
            )
            .as_bytes(),
        )
    }

    pub fn finalize(&self, result: &Solution) -> Result<(), Box<dyn Error>> {
        let mut json = File::create(
            self._outputs
                .join(format!("{}-{}.json", self._problem, self._id)),
        )?;
        println!("Writing solution to {:?}", json);
        json.write(serde_json::to_string(&result)?.as_bytes())?;

        let mut json = File::create(
            self._outputs
                .join(format!("{}-{}-config.json", self._problem, self._id)),
        )?;
        println!("Writing config to {:?}", json);
        json.write(serde_json::to_string(&*CONFIG)?.as_bytes())?;

        Ok(())
    }
}
