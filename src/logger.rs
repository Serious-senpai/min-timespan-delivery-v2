use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::Path;

use rand::distr::Alphanumeric;
use rand::Rng;

use crate::config::CONFIG;
use crate::errors::ExpectedValue;
use crate::solutions::Solution;

pub struct Logger {
    pub iterations: usize,
}

impl Logger {
    pub fn new() -> Self {
        Logger { iterations: 0 }
    }

    pub fn finalize(&self, result: &Solution) -> Result<(), Box<dyn Error>> {
        let problem = ExpectedValue::cast(
            Path::new(&CONFIG.problem)
                .file_stem()
                .and_then(|f| f.to_os_string().into_string().ok()),
        )?;
        let outputs = Path::new(&CONFIG.outputs);
        if !outputs.is_dir() {
            std::fs::create_dir_all(outputs)?;
        }

        let id = rand::rng()
            .sample_iter(&Alphanumeric)
            .take(8)
            .map(char::from)
            .collect::<String>();

        let mut json = File::create(outputs.join(format!("{}-{}.json", problem, id)))?;
        println!("Writing solution to {:?}", json);
        json.write(serde_json::to_string(&result)?.as_bytes())?;

        let mut json = File::create(outputs.join(format!("{}-{}-config.json", problem, id)))?;
        println!("Writing config to {:?}", json);
        json.write(serde_json::to_string(&*CONFIG)?.as_bytes())?;

        Ok(())
    }
}
