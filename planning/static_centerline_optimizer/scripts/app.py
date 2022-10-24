#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from flask import Flask
from flask import abort
from flask import request
from flask import session
import rclpy
from rclpy.node import Node
from static_centerline_optimizer.srv import LoadMap
from static_centerline_optimizer.srv import PlanPath
from static_centerline_optimizer.srv import PlanRoute

rclpy.init()
node = Node("app")

app = Flask(__name__)
app.secret_key = "tmp_secret_key"


def create_client(service_type, server_name):
    # create client
    cli = node.create_client(service_type, server_name)
    while not cli.wait_for_service(timeout_sec=1.0):
        print("{} service not available, waiting again...".format(server_name))
    return cli


@app.route("/load_map", methods=["POST"])
def load_map_post():
    data = request.get_json()
    session["map_id"] = 1

    print(data["map"])

    # create client
    cli = create_client(LoadMap, "load_map")

    # request map loading
    req = LoadMap.Request(map=data["map"])
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    # TODO(murooka)
    error = False
    if error:
        abort(500, "error_message")

    # TODO(murooka) generate uuid
    return {"mapId": "1"}


@app.route("/plan_route", methods=["POST"])
def plan_route_post():
    data = request.get_json()

    # create client
    cli = create_client(PlanRoute, "plan_route")

    # request route planning
    req = PlanRoute.Request(start_lane_id=data["start_lane_id"], end_lane_id=data["end_lane_id"])
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    # TODO(murooka)
    error = False
    if error:
        abort(500, "error_message")

    return {"lane_ids": list(res.lane_ids)}


@app.route("/plan_path", methods=["POST"])
def plan_path_post():
    data = request.get_json()

    # create client
    cli = create_client(PlanPath, "plan_path")

    # request path planning
    req = PlanPath.Request(start_lane_id=data["start_lane_id"])
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    return {"po": "po"}


if __name__ == "__main__":
    app.debug = True
    app.secret_key = "anyrandomstring"
    app.run(host="localhost")
