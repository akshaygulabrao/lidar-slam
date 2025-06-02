#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

rostopic echo -b /bags/recorded_data.bag -p /tf > "/bags/a.csv"