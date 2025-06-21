#!/usr/bin/env python3
from dataclasses import dataclass
import draccus
import os
import subprocess
import sys
from typing import List, Tuple

# Constants
DOCKER_IMAGE = "rosdemo"
CONTAINER_NAME = "pointLIO"
PORT_MAPPING = "8765:8765"
DOCKER_LAUNCH_FILE = "pointLIO_NewerCollege.launch"
SETUP_SCRIPT = "setup_ros.sh"
LIDAR_CONFIG = "ouster128-newercollege.yaml"


@dataclass
class Config:
    bash: bool = False
    point_cloud: bool = False
    scan: bool = False
    run_name: str = "default-stairs"
    path_bag: str = "/Volumes/hd1/NewerCollegeDataset/2021-07-01-10-40-50_0-stairs-001.bag"
    path_truth: str = "/Volumes/hd1/NewerCollegeDataset/collection 1 - newer college/ground_truth/tum_format/gt-nc-stairs.csv"


def main():

    # should eventually check whether it exists and then cancel it
    subprocess.run("docker rm -f pointLIO".split())
    config = draccus.parse(Config)

    # Verify required files
    for file_path, description in [
        (config.path_bag, "bag file"),
        (config.path_truth, "ground truth file"),
        (wd_path(DOCKER_LAUNCH_FILE), "launch file"),
        (wd_path(SETUP_SCRIPT), "setup script"),
        (wd_path(LIDAR_CONFIG), "lidar config"),
    ]:
        if not os.path.isfile(file_path):
            sys.exit(f"Can't find '{description}' at: {file_path}")

    # Setup output directories
    os.makedirs("results/tums", exist_ok=True)
    output_tum = f"results/tums/{config.run_name}.tum"
    results_zip = f"results/{config.run_name}.zip"

    # Docker operations
    if subprocess.run(f"docker build . -t {DOCKER_IMAGE}".split()).returncode != 0:
        sys.exit("Docker build failed")

    mounts = [
        f"-v{os.getcwd()}/catkin_ws/:/catkin_ws/",
        f"-v{config.path_bag}:/test.bag",
        f"-v{wd_path(DOCKER_LAUNCH_FILE)}:/{DOCKER_LAUNCH_FILE}",
        f"-v{os.getcwd()}/bags:/bags",
        f"-v{wd_path(SETUP_SCRIPT)}:/{SETUP_SCRIPT}",
        f"-v{wd_path(LIDAR_CONFIG)}:/{LIDAR_CONFIG}",
    ]

    docker_cmd = [
        "docker",
        "run",
        "--rm",
        "-it",
        *mounts,
        "-p",
        PORT_MAPPING,
        "--name",
        CONTAINER_NAME,
        DOCKER_IMAGE,
    ]
    docker_cmd += ["/bin/bash", SETUP_SCRIPT]
    if config.point_cloud:
        docker_cmd.append("cloudwise")

    if subprocess.run(docker_cmd).returncode != 0:
        sys.exit("Docker run failed")

    # Process results
    for cmd, error in [
        (
            [
                "uv",
                "run",
                "evo_traj",
                "bag",
                "bags/recorded_data.bag",
                "/aft_mapped_to_init",
                "--save_as_tum",
            ],
            "Trajectory conversion failed",
        ),
        (
            [
                "uv",
                "run",
                "evo_traj",
                "tum",
                output_tum,
                "--ref",
                config.path_truth,
                "--align",
                "--no_warnings",
            ],
            "Trajectory evaluation failed",
        ),
        (
            [
                "uv",
                "run",
                "evo_ape",
                "tum",
                config.path_truth,
                output_tum,
                "-a",
                "--save_results",
                results_zip,
                "--no_warnings",
            ],
            "APE evaluation failed",
        ),
    ]:
        if subprocess.run(cmd).returncode != 0:
            sys.exit(error)

    if os.path.exists("aft_mapped_to_init.tum"):
        os.rename("aft_mapped_to_init.tum", output_tum)
    else:
        print("Warning: Trajectory output file not found")

    print(f"Processing complete. Results saved to {results_zip}")


def wd_path(filename: str) -> str:
    return os.path.join(os.getcwd(), filename)


if __name__ == "__main__":
    main()
