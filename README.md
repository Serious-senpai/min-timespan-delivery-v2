# min-timespan-delivery-v2

This repository contains the original implementation of the paper **"The Min-Makespan Vehicle Routing Problem with Drones under Multiple Trips and Visits"**, which is packed with a bunch of undocumented optimizations that somehow make it fast, though there is still plenty of room to make it faster (and cleaner).

All experimental results in the paper come straight from the automated [workflow runs](https://github.com/Serious-senpai/min-timespan-delivery-v2/actions) of this repository. Thanks to [GitHub Actions](https://github.com/features/actions), experiments run themselves, results are reproducible, and nobody has to trust "it worked on my machine".

## Build and run

### Using devcontainer

The *highly recommended* way to setup development environment is to use [GitHub Codespaces](https://github.com/features/codespaces). This repository already contains a [`.devcontainer`](/.devcontainer) directory to setup a workspace. Full VSCode support, work directly in your browser, no extra setup required, and no disk memory wasted.

### Without devcontainer

If you already speak fluent GitHub Actions, just read the [`run.yml`](.github/workflows/run.yml) workflow and copy whatever dark magic it uses to set up the build environment.

For everyone else, the short version:
- Install [`cargo`](https://doc.rust-lang.org/cargo). The workflow uses `cargo` **v1.89**. Other versions will *probably* work too, but no promises.
- Clone the repository, then run `cargo build --release` to build the executable (usually at `target/release/min-timespan-delivery`, unless a future `cargo` release decides to surprise us).
- Invoke the executable with `--help` to see what it can do and how to use it.
