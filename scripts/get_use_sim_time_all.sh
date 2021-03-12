#!/bin/bash
( for node in $(ros2 run topic_tools node_list |& grep \/ | perl -pe "~s/^[^\/]\//\//")
#for node in $(ros2 param list|grep :)
do
	#n=$(dirname $node)/$(basename $node :)
	#echo "$n: use_sim_time: $(ros2 param get $n use_sim_time)"
	echo "$node: use_sim_time: $(ros2 param get $node use_sim_time)"
done) |& grep \/
