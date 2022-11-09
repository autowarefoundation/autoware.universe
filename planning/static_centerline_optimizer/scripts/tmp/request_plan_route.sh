#!/bin/bash

#curl -H "Content-type: application/json" -X POST -d '{"start_lane_id": 125, "end_lane_id": 132}' localhost:4010/get_planned_route
curl -X GET -sS 'localhost:4010/planned_route?start_lane_id=125&end_lane_id=132'
