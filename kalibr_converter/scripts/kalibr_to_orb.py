import yaml
import numpy as np
import argparse
import os
from collections import OrderedDict

def parse_kalibr_yaml(kalibr_file_path):
    """Parses the Kalibr camchain-imucam.yaml file."""
    with open(kalibr_file_path, 'r') as f:
        data = yaml.safe_load(f)
        # Get the first camera (e.g., 'cam0')
        cam_name = list(data.keys())[0] 
        return data[cam_name]

def parse_imu_yaml(imu_file_path):
    """Parses the Kalibr-style imu.yaml to get noise parameters."""
    with open(imu_file_path, 'r') as f:
        return yaml.safe_load(f)

def format_opencv_matrix(matrix_name, matrix):
    """Formats a numpy array into the !!opencv-matrix string format."""
    rows, cols = matrix.shape
    
    matrix_str_rows = []
    for row in matrix:
        # Format each number with a specific precision
        matrix_str_rows.append(" ".join([f"{val:12.8f}," for val in row]))
    
    # Remove the final comma from the last line
    last_row = matrix_str_rows[-1]
    matrix_str_rows[-1] = last_row[:last_row.rfind(',')]
    
    formatted_data = "\n          ".join(matrix_str_rows)
    
    return (
        f"{matrix_name}: !!opencv-matrix\n"
        f"   rows: {rows}\n"
        f"   cols: {cols}\n"
        f"   dt: f\n"
        f"   data: [{formatted_data}]"
    )

def generate_orb_slam3_config(kalibr_data, imu_data, output_dir):
    """Generates a YAML config file for ORB-SLAM3 inside a specified directory."""
    
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Define the final file path
    output_path = os.path.join(output_dir, "orb_config.yaml")

    # Kalibr provides T_cam_imu (IMU -> Camera)
    T_cam_imu = np.array(kalibr_data['T_cam_imu'])
    
    # ORB-SLAM3 standard is T_b_c (Camera -> Body/IMU), which is the inverse.
    T_b_c_inverse = np.linalg.inv(T_cam_imu)

    # Using an OrderedDict to maintain the desired key order
    orb_config = OrderedDict()

    # Camera Parameters
    orb_config['Camera.type'] = "\"PinHole\""
    orb_config['Camera.fx'] = kalibr_data['intrinsics'][0]
    orb_config['Camera.fy'] = kalibr_data['intrinsics'][1]
    orb_config['Camera.cx'] = kalibr_data['intrinsics'][2]
    orb_config['Camera.cy'] = kalibr_data['intrinsics'][3]
    orb_config['Camera.k1'] = kalibr_data['distortion_coeffs'][0]
    orb_config['Camera.k2'] = kalibr_data['distortion_coeffs'][1]
    orb_config['Camera.p1'] = kalibr_data['distortion_coeffs'][2]
    orb_config['Camera.p2'] = kalibr_data['distortion_coeffs'][3]
    orb_config['Camera.width'] = kalibr_data['resolution'][0]
    orb_config['Camera.height'] = kalibr_data['resolution'][1]
    orb_config['Camera.fps'] = 29.5
    orb_config['Camera.RGB'] = 0

    # IMU Parameters
    orb_config['IMU.NoiseGyro'] = imu_data['gyroscope_noise_density']
    orb_config['IMU.NoiseAcc'] = imu_data['accelerometer_noise_density']
    orb_config['IMU.GyroWalk'] = imu_data['gyroscope_random_walk']
    orb_config['IMU.AccWalk'] = imu_data['accelerometer_random_walk']
    orb_config['IMU.Frequency'] = imu_data['update_rate']

    # ORB Extractor and Viewer Parameters
    orb_config['ORBextractor.nFeatures'] = 1000
    orb_config['ORBextractor.scaleFactor'] = 1.2
    orb_config['ORBextractor.nLevels'] = 8
    orb_config['ORBextractor.iniThFAST'] = 20
    orb_config['ORBextractor.minThFAST'] = 7
    orb_config['Viewer.KeyFrameSize'] = 0.05
    orb_config['Viewer.KeyFrameLineWidth'] = 1.0
    orb_config['Viewer.GraphLineWidth'] = 0.9
    orb_config['Viewer.PointSize'] = 2.0
    orb_config['Viewer.CameraSize'] = 0.08
    orb_config['Viewer.CameraLineWidth'] = 3.0
    orb_config['Viewer.ViewpointX'] = 0.0
    orb_config['Viewer.ViewpointY'] = -0.7
    orb_config['Viewer.ViewpointZ'] = -3.5
    orb_config['Viewer.ViewpointF'] = 500.0

    # Write to file
    with open(output_path, 'w') as f:
        f.write("%YAML:1.0\n\n")
        
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write("# Camera Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        for key, value in list(orb_config.items())[:12]:
            f.write(f"{key}: {value}\n")
        
        f.write("\n#--------------------------------------------------------------------------------------------\n")
        f.write("# IMU Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")

        # Commented Kalibr matrix for reference
        f.write("# NON-STANDARD (direct Kalibr output, T_cam_imu): Transformation from IMU to Camera frame.\n")
        tbc_direct_str = format_opencv_matrix("Tbc", T_cam_imu)
        commented_tbc_str = '\n'.join([f'# {line}' for line in tbc_direct_str.split('\n')])
        f.write(commented_tbc_str + "\n\n")

        # Active standard inverse matrix
        f.write("# STANDARD ORB-SLAM3 (inverse of Kalibr, T_b_c): Transformation from Camera to IMU frame.\n")
        tbc_inverse_str = format_opencv_matrix("Tbc", T_b_c_inverse)
        f.write(tbc_inverse_str + "\n")

        for key, value in list(orb_config.items())[12:17]:
             f.write(f"{key}: {value}\n")

        f.write("\n#--------------------------------------------------------------------------------------------\n")
        f.write("# ORB Extractor Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        for key, value in list(orb_config.items())[17:22]:
            f.write(f"{key}: {value}\n")
            
        f.write("\n#--------------------------------------------------------------------------------------------\n")
        f.write("# Viewer Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        for key, value in list(orb_config.items())[22:]:
            f.write(f"{key}: {value}\n")

    print(f"Successfully generated ORB-SLAM3 config at: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Kalibr IMU-Camera calibration to a standard ORB-SLAM3 format."
    )
    parser.add_argument(
        "--kalibr_file", type=str, required=True,
        help="Path to the Kalibr 'camchain-imucam.yaml' file."
    )
    parser.add_argument(
        "--imu_file", type=str, required=True,
        help="Path to the Kalibr 'imu.yaml' file containing noise parameters."
    )
    parser.add_argument(
        "--output_dir", type=str, required=True,
        help="Directory to save the ORB-SLAM3 YAML file."
    )
    args = parser.parse_args()

    # These helper functions are now defined above
    kalibr_data = parse_kalibr_yaml(args.kalibr_file)
    imu_data = parse_imu_yaml(args.imu_file)

    generate_orb_slam3_config(kalibr_data, imu_data, args.output_dir)

if __name__ == "__main__":
    main()