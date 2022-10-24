#!/bin/bash

curl -H "Content-type: application/json" -X POST -d '{"start_lane_id": 125, "end_lane_id": 132}' localhost:5000/plan_route
