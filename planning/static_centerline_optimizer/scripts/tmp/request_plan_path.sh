#!/bin/bash

curl -H "Content-type: application/json" -X POST -d '{"start_lane_id": 125}' localhost:5000/plan_path
