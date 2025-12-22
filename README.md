
## Step 1: Target
Download and print the calibration target

[AprilGrid 6x6 0.8m](https://github.com/ethz-asl/kalibr/wiki/downloads)<br>

## Step 2: Camera Calibration

### 2.1 Record Camera Calibration Data
>Reminder 1: An SSH Session with X11 Display forwarding is required to live view the video stream from the raspberry pi 5 during calibration. 

```bash
ssh -Y raspberrypi@10.42.0.1
```

>Reminder 2: Ensure the ros node is publishing camera image topics, e.g. rostopic info /mono_camera/image_raw
```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rostopic info /mono_camera/image_raw
```

**Setup:** Keep the AprilGrid **stationary** throughout this recording.

Launch the image viewer to monitor your camera feed:
```bash
rosrun image_view image_view image:=/mono_camera/image_raw
```
Start recording:
```bash
rosbag record -O camera_calibration_data.bag /mono_camera/image_raw
```

**Procedure:** Move and rotate the **camera** around the stationary AprilGrid, ensuring the target remains visible in the frame at all times. Capture various angles, distances, and orientations.

### 2.2 Run Camera Calibration
```bash
rosrun kalibr kalibr_calibrate_cameras \
  --target target.yaml \
  --bag camera_calibration_data.bag \
  --models pinhole-radtan \
  --topics /mono_camera/image_raw
```

**Output:** `camchain.yaml` (camera intrinsics and distortion parameters)

---

## Step 3: Camera-IMU Calibration

### 3.1 Record Camera-IMU Calibration Data
**Setup:** Keep the AprilGrid **stationary** throughout this recording.

Launch the image viewer:
```bash
rosrun image_view image_view image:=/mono_camera/image_raw
```

Start recording:
```bash
rosbag record -O camera_imu_calibration_data.bag /mono_camera/image_raw /mavros/imu/data_raw
```

**Procedure:** Rotate and move the **camera** (and IMU) while keeping the AprilGrid stationary and in view. Excite all IMU axes (pitch, roll, yaw, and translation in all directions) to capture the spatial and temporal relationship between camera and IMU.

### 3.2 Run Camera-IMU Calibration
```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --target target.yaml \
  --cam camchain.yaml \
  --imu imu.yaml \
  --bag camera_imu_calibration_data.bag \
  --time-calibration
```

**Output:** `camchain-imucam.yaml` containing:
- Camera intrinsics and distortion coefficients
- IMU noise characteristics
- Camera-IMU extrinsic transformation (rotation and translation)
- Time offset between camera and IMU

---
## Step 4: Generate SLAM Configuration Files

The calibration results from `camchain-imucam.yaml` are used to generate configuration files for VINS-Fusion and ORB-SLAM3.

> ⚠️ **Important:** See `config/calibration_results.yaml` for example formatting of the resulting matrices used in the SLAM nodes.

**Key Parameters for SLAM Configuration:**
- Camera intrinsics (fx, fy, cx, cy)
- Distortion coefficients (k1, k2, p1, p2)
- Camera-IMU extrinsic rotation and translation
- IMU noise parameters

Refer to your SLAM framework's documentation for specific configuration file format requirements.

### Launch the kalibr_converter container
```bash
docker compose down
docker compose run --rm kalibr_converter
```
The ros environment should launch and place you in the /scripts directory.

### Create the VINS-Fusion configuration files:
```bash
./convert-to-vins.sh kalibr_mono_imu_capture-camchain-imucam.yaml kalibr_imu.yaml
```
The file is located in /kalibr_converter/data/vins_config

### Create the ORB-SLAM3 configuration file:
```bash
./convert-to-orb.sh kalibr_mono_imu_capture-camchain-imucam.yaml kalibr_imu.yaml
```
The file is located in /kalibr_converter/data/orb_config
