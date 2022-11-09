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

from IPython import embed
import json
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


@app.route("/post_map", methods=["POST"])
def load_map_post():
    data = request.get_json()
    session["map_id"] = 1

    print(data["map"])

    # create client
    cli = create_client(LoadMap, "/planning/static_centerline_optimizer/load_map")

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


@app.route("/get_planned_route", methods=["GET"])
def plan_route_post():
    # data = request.get_json()
    args = request.args.to_dict()

    # create client
    cli = create_client(PlanRoute, "/planning/static_centerline_optimizer/plan_route")

    # request route planning
    req = PlanRoute.Request(start_lane_id=int(args.get("start_lane_id")), end_lane_id=int(args.get("end_lane_id")))
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    # error handling
    if res.message != "":
        if res.message == "route_has_not_been_planned":
            abort(500, "error_message")
        else:
            # invalid
            pass


    return {"lane_ids": list(res.lane_ids)}


@app.route("/get_planned_path", methods=["GET"])
def plan_path_post():
    # data = request.get_json()
    args = request.args.to_dict()

    # create client
    cli = create_client(PlanPath, "/planning/static_centerline_optimizer/plan_path")

    # request path planning
    req = PlanPath.Request(start_lane_id=int(args.get("start_lane_id")))
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    # error handling
    if res.message != "":
        if True:
            pass
        else:
            # invalid
            pass



    # create output json
    result_json = []
    for points_with_lane_id in res.points_with_lane_ids:
        current_lane_points = []
        for geom_point in points_with_lane_id.points:
            point = {"x" : geom_point.x, "y" : geom_point.y, "z" : geom_point.z}
            current_lane_points.append(point)

        current_result_json = {}
        current_result_json["lane_id"] = int(points_with_lane_id.lane_id)
        current_result_json["points"] = current_lane_points

        result_json.append(current_result_json)

    return json.dumps(result_json)


if __name__ == "__main__":
    app.debug = True
    app.secret_key = "anyrandomstring"
    app.run(host="localhost")
