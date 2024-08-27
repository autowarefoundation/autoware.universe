import copy

import yaml

object_type_names = [
    "pointcloud",
    "unknown",
    "car",
    "truck",
    "bus",
    "trailer",
    "motorcycle",
    "bicycle",
    "pedestrian",
]

with open("common_parameters.yaml") as f:
    common_params: dict = yaml.safe_load(f)
with open("misc_parameters.yaml") as f:
    misc_params = yaml.safe_load(f)

params = {}
for type_name in object_type_names:
    params |= {type_name: copy.deepcopy(common_params)}
    params[type_name]["enable_check"][
        "description"
    ] = f"Enable to check surrounding {type_name}"
params |= misc_params
params["pointcloud"]["enable_check"]["default_value"] = False
params = {"surround_obstacle_checker_node": params}

with open("surround_obstacle_checker_node_parameters.yaml", "w") as f:
    yaml.safe_dump(params, f, sort_keys=False)
