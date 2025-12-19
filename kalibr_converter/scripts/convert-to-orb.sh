#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Define the output directory
OUTPUT_DIR="/data/orb_config"

# Create the directory inside the container if it doesn't exist
mkdir -p $OUTPUT_DIR

echo "--- Running Kalibr to ORB-SLAM3 conversion script ---"

python3 /scripts/kalibr_to_orb.py \
    --kalibr_file /data/kalibr_mono_imu_capture-camchain-imucam.yaml \
    --imu_file /data/kalibr_imu.yaml \
    --output_dir $OUTPUT_DIR

echo "--- Conversion complete. Output file is '$OUTPUT_DIR' ---"