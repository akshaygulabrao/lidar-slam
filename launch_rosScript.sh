#!/bin/bash

function check() {
    local path_var_name=$1
    local path="${!path_var_name}"  # Indirect variable reference
    
    if [[ ! -f "$path" ]]; then
        echo "Can't find \"$path_var_name\" (value: $path)"
        exit 1
    fi
}

WORKDIR=$PWD

# Default values
DEFAULT_RUN_NAME="default-nc-quad-easy"
DEFAULT_INPUT_PATH="/Volumes/hd1/NewerCollegeDataset/2021-07-01-10-37-38-quad-easy.bag"
DEFAULT_GROUND_TRUTH="/Volumes/hd1/NewerCollegeDataset/collection 1 - newer college/ground_truth/tum_format/gt-nc-quad-easy.csv"

check DEFAULT_INPUT_PATH
check DEFAULT_GROUND_TRUTH

# Handle command-line arguments
if [ $# -eq 0 ]; then
    RUN_NAME=$DEFAULT_RUN_NAME
    INPUT_PATH=$DEFAULT_INPUT_PATH
    GROUND_TRUTH_FILE="$DEFAULT_GROUND_TRUTH"
elif [ $# -eq 3 ]; then
    RUN_NAME="$1"
    INPUT_PATH="$2"
    GROUND_TRUTH_FILE="$3"
else
    echo "Usage: $0 [run_name bag_path ground_truth_file]"
    echo "If no arguments are given, default values are used."
    exit 1
fi

# Docker variables
DOCKER_IMAGE="rosdemo"
DOCKER_NAME="pointLIO"
DOCKER_NETWORK="rosnet"
DOCKER_LAUNCH_FILE="pointLIO_foxglove.launch"
DOCKER_SETUP_SCRIPT="setup_ros.sh"

check DOCKER_LAUNCH_FILE

[[ -f $DOCKER_LAUNCH_FILE ]] || {
    echo "DEFAULT_LAUNCH_FILE does not exist"
    exit 1 
}

# Output paths
OUTPUT_TUM_FILE="results/tums/${RUN_NAME}.tum"
RESULTS_ZIP="results/${RUN_NAME}.zip"

# Create output directories if they don't exist
mkdir -p results/tums

# Build and run Docker container
docker build . -t $DOCKER_IMAGE && \
docker run --rm -it \
  -v $INPUT_PATH:/test.bag \
  -v $WORKDIR/catkin_ws/:/catkin_ws/ \
  -v $WORKDIR/$DOCKER_LAUNCH_FILE:/$DOCKER_LAUNCH_FILE \
  -v $WORKDIR/bags:/bags \
  -v $WORKDIR/$DOCKER_SETUP_SCRIPT:/$DOCKER_SETUP_SCRIPT \
  -v $WORKDIR/ouster128-newercollege.yaml:/ouster128-newercollege.yaml \
  -p 8765:8765 \
  --name $DOCKER_NAME \
  $DOCKER_IMAGE \
  /bin/bash $DOCKER_SETUP_SCRIPT

# Process trajectory data
uv run evo_traj bag bags/recorded_data.bag /aft_mapped_to_init --save_as_tum
mv aft_mapped_to_init.tum $OUTPUT_TUM_FILE

# Run evaluation metrics
uv run evo_traj tum $OUTPUT_TUM_FILE --ref "$GROUND_TRUTH_FILE" --align --no_warnings
uv run evo_ape tum "$GROUND_TRUTH_FILE" $OUTPUT_TUM_FILE -a --save_results $RESULTS_ZIP --no_warnings