#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

function check() {
    local path_var_name=$1
    local path="${!path_var_name}"  # Indirect variable reference
    
    if [[ ! -f "$path" ]]; then
        echo "Can't find '$path_var_name' (value: $path)"
        exit 1
    fi
}

LAUNCH_FILE=pointLIO_NewerCollege.launch
check LAUNCH_FILE

if [[ $1 == "cloudwise" ]]; then
    roslaunch $LAUNCH_FILE cloudwise:=true
fi

