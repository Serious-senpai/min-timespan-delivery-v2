from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        result: Path


parser = argparse.ArgumentParser()
parser.add_argument("result", type=Path, help="Path to the result file")
namespace = parser.parse_args(namespace=Namespace())


template_html = Path(__file__).parent / "map.html"
html = template_html.read_text(encoding="utf-8")

with namespace.result.open("r", encoding="utf-8") as f:
    data = json.load(f)


html = html.replace(
    "const x = undefined; // DATA SECTION",
    f"const x = {data['config']['x']};",
)
html = html.replace(
    "const y = undefined; // DATA SECTION",
    f"const y = {data['config']['y']};",
)
html = html.replace(
    "const dronable = undefined; // DATA SECTION",
    f"const dronable = {json.dumps(data['config']['dronable'])};",
)


drone = data["config"]["drone"]
if drone["config"] == "Endurance":
    html = html.replace(
        "const enduranceDroneRange = undefined; // DATA SECTION",
        f"const enduranceDroneRange = {drone['_data']['FixedTime (s)'] * drone['_data']['V_max (m/s)'] / 2};",
    )


html = html.replace(
    "const truck_routes = undefined; // DATA SECTION",
    f"const truck_routes = {data['solution']['truck_routes']};",
)
html = html.replace(
    "const drone_routes = undefined; // DATA SECTION",
    f"const drone_routes = {data['solution']['drone_routes']};",
)


distance_func = {
    "manhattan": "function (dx, dy) { return Math.abs(dx) + Math.abs(dy); }",
    "euclidean": "function (dx, dy) { return Math.sqrt(dx * dx + dy * dy); }",
}
html = html.replace(
    "const truck_distance = undefined; // DATA SECTION",
    f"const truck_distance = {distance_func[data['config']['truck_distance']]};",
)
html = html.replace(
    "const drone_distance = undefined; // DATA SECTION",
    f"const drone_distance = {distance_func[data['config']['drone_distance']]};",
)


with tempfile.NamedTemporaryFile(
    "w",
    encoding="utf-8",
    suffix=".html",
    prefix=namespace.result.stem,
    delete=False,
) as writer:
    writer.write(html)


print(writer.name)
# webbrowser.open(writer.name)
