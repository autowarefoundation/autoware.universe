#!/bin/bash

map_content=$(cat lanelet2_map.osm | tr -d '\n')
tmp="{\"map\": \"${map_content}\"}"
echo $tmp | curl -s -H "Content-type: application/json" -X POST -d @- localhost:4010/map
