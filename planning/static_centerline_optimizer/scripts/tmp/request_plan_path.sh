#!/bin/bash

# curl -H "Content-type: application/json" -X POST -d '{"start_lane_id": 125}' localhost:4010/get_planned_path
curl -X GET -sS 'localhost:4010/planned_path?start_lane_id=125'
