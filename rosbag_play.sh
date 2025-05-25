#!/bin/bash

source /opt/ros/noetic/setup.bash
sleep 2
rostopic pub -r 4 /clock rosgraph_msgs/Clock "{clock: {secs: 0, nsecs: 0}}"