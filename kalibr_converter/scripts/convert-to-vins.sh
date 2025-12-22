#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
DATA_DIR="/data"
OUTPUT_DIR="$DATA_DIR/vins_config"

# Defaults: Use the first argument ($1) if provided, otherwise use the hardcoded string
KALIBR_FILENAME=${1:-"kalibr_mono_imu_capture-camchain-imucam.yaml"}
IMU_FILENAME=${2:-"kalibr_imu.yaml"}

# Construct full paths
KALIBR_FILEPATH="$DATA_DIR/$KALIBR_FILENAME"
IMU_FILEPATH="$DATA_DIR/$IMU_FILENAME"

# --- Logic ---

# Create the directory inside the container if it doesn't exist
mkdir -p "$OUTPUT_DIR"

echo "--- Checking for input files ---"
if [ ! -f "$KALIBR_FILEPATH" ]; then
    echo "Error: Kalibr file not found at $KALIBR_FILEPATH"
    echo "Usage: $0 [kalibr_filename] [imu_filename]"
    exit 1
fi

if [ ! -f "$IMU_FILEPATH" ]; then
    echo "Error: IMU file not found at $IMU_FILEPATH"
    exit 1
fi

echo "Using Kalibr file: $KALIBR_FILEPATH"
echo "Using IMU file:    $IMU_FILEPATH"

echo "--- Running Kalibr to VINS-Fusion conversion script ---"

python3 /scripts/kalibr_to_vins.py \
    --kalibr_file "$KALIBR_FILEPATH" \
    --imu_file "$IMU_FILEPATH" \
    --output_dir "$OUTPUT_DIR"

echo "--- Conversion complete. Output files are in '$OUTPUT_DIR' ---"