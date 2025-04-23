from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Dict, Set, TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        directory: Path
        executable: str
        linear: str
        linear_dir: str
        non_linear: str
        non_linear_dir: str


parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("--directory", type=Path, default="outputs/")
parser.add_argument("--executable", type=str, default="./min-timespan-delivery")
parser.add_argument("--linear", type=str, default="problems/config_parameter/drone_linear_config.json")
parser.add_argument("--linear-dir", type=str, default="outputs/linear/")
parser.add_argument("--non-linear", type=str, default="problems/config_parameter/drone_nonlinear_config.json")
parser.add_argument("--non-linear-dir", type=str, default="outputs/non-linear/")

namespace = parser.parse_args(namespace=Namespace())
print(namespace)


problems: Set[str] = set()
for file in namespace.directory.glob("*-config.json"):
    with file.open("r", encoding="utf-8") as f:
        data = json.load(f)

    problems.add(data["problem"])


linear: Dict[str, str] = {}
for problem in problems:
    process = subprocess.Popen(
        [
            namespace.executable, "run",
            problem,
            "--drone-cfg", namespace.linear,
            "--config", "linear",
            "--outputs", "/tmp/linear",
            "--disable-logging",
            "--dry-run",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        print(f"Standard error when generating linear config for {problem}:\n{stderr}")
        continue

    _, _, config = stdout.splitlines()
    with open(config, "r", encoding="utf-8") as f:
        data = json.load(f)

    with open(config, "w", encoding="utf-8") as f:
        data["outputs"] = namespace.linear_dir
        json.dump(data, f)

    linear[Path(problem).stem.split("-")[0]] = config


non_linear: Dict[str, str] = {}
for problem in problems:
    process = subprocess.Popen(
        [
            namespace.executable, "run",
            problem,
            "--drone-cfg", namespace.non_linear,
            "--config", "non-linear",
            "--outputs", "/tmp/non-linear",
            "--disable-logging",
            "--dry-run",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        print(f"Standard error when generating non-linear config for {problem}:\n{stderr}")
        continue

    _, _, config = stdout.splitlines()
    with open(config, "r", encoding="utf-8") as f:
        data = json.load(f)

    with open(config, "w", encoding="utf-8") as f:
        data["outputs"] = namespace.non_linear_dir
        json.dump(data, f)

    non_linear[Path(problem).stem.split("-")[0]] = config


print(linear)
print(non_linear)


for file in namespace.directory.glob("*-solution.json"):
    problem = file.stem.split("-")[0]
    subprocess.run(
        [
            namespace.executable, "evaluate",
            str(file),
            linear[problem],
        ]
    )
    subprocess.run(
        [
            namespace.executable, "evaluate",
            str(file),
            non_linear[problem],
        ]
    )
