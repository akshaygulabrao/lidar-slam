WORKDIR="/Users/trading/workspace/lidar-slam"
INPUT_PATH="/Users/trading/workspace/collection1-newercollege/2021-07-01-10-37-38-quad-easy.bag"
GROUND_TRUTH_DIR="../collection1-newercollege/ground_truth/tum_format"
GT_FILE="gt-nc-quad-easy.csv"

# Extract filename components
filename="${INPUT_PATH##*/}"        # Get filename with extension
basename="${filename%.*}"           # Remove extension
OUTPUT_TUM_FILE="${basename}.tum"  # Output TUM file name

# Docker variables
DOCKER_IMAGE="rosdemo"
DOCKER_NAME="pointLIO"
DOCKER_NETWORK="rosnet"
DOCKER_LAUNCH_FILE="pointLIO_foxglove.launch"
DOCKER_SETUP_SCRIPT="setup_ros.sh"

# Build and run Docker container
docker build . -t $DOCKER_IMAGE && \
docker run --rm -it \
  --network $DOCKER_NETWORK \
  -v $INPUT_PATH:/test.bag \
  -v $WORKDIR/catkin_ws/:/catkin_ws/ \
  -v $WORKDIR/$DOCKER_LAUNCH_FILE:/$DOCKER_LAUNCH_FILE \
  -v $WORKDIR/bags:/bags \
  -v $WORKDIR/$DOCKER_SETUP_SCRIPT:/$DOCKER_SETUP_SCRIPT \
  -p 8765:8765 \
  --name $DOCKER_NAME \
  $DOCKER_IMAGE \
  /bin/bash $DOCKER_SETUP_SCRIPT

# Process trajectory data
uv run evo_traj bag bags/recorded_data.bag /aft_mapped_to_init --save_as_tum
mv aft_mapped_to_init.tum $OUTPUT_TUM_FILE
uv run evo_traj tum $OUTPUT_TUM_FILE --ref $GROUND_TRUTH_DIR/$GT_FILE -p --plot_mode=xy --align
uv run evo_rpe tum $GROUND_TRUTH_DIR/$GT_FILE $OUTPUT_TUM_FILE -a -p --plot_mode=xyz