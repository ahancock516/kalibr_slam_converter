import yaml
import numpy as np
import argparse
import os
from collections import OrderedDict

# --- Helper to prevent curly braces in YAML output ---
class NoAliasDumper(yaml.Dumper):
    def ignore_aliases(self, data):
        return True

def represent_ordereddict(dumper, data):
    value = []
    for item_key, item_value in data.items():
        node_key = dumper.represent_data(item_key)
        node_value = dumper.represent_data(item_value)
        value.append((node_key, node_value))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', value)

yaml.add_representer(OrderedDict, represent_ordereddict, Dumper=NoAliasDumper)


def parse_kalibr_yaml(kalibr_file_path):
    """Parses the Kalibr camchain-imucam.yaml file."""
    with open(kalibr_file_path, 'r') as f:
        data = yaml.safe_load(f)
        return data[list(data.keys())[0]]

def parse_imu_yaml(imu_file_path):
    """Parses the Kalibr-style imu.yaml file."""
    with open(imu_file_path, 'r') as f:
        return yaml.safe_load(f)

def format_opencv_matrix(matrix):
    """Formats a numpy array into the !!opencv-matrix string format for VINS."""
    rows, cols = matrix.shape
    data_str_rows = []
    for row in matrix:
        data_str_rows.append(" ".join([f"{val:11.8f}," for val in row]))
    
    last_row = data_str_rows[-1]
    data_str_rows[-1] = last_row[:last_row.rfind(',')] + " ".ljust(len(last_row) - last_row.rfind(','))
    
    formatted_data = "\n          ".join(data_str_rows)

    return (
        "!!opencv-matrix\n"
        f"  rows: {rows}\n"
        f"  cols: {cols}\n"
        "  dt: d\n"
        f"  data: [{formatted_data}]"
    )

def generate_files(kalibr_data, imu_data, output_dir):
    """Generates both the main config and the intrinsics config for VINS-Fusion."""

    # --- Create Intrinsics File (camera_config.yaml) ---
    intrinsics_config = OrderedDict()
    intrinsics_config['model_type'] = 'PINHOLE'
    intrinsics_config['camera_name'] = 'camera'
    intrinsics_config['image_width'] = kalibr_data['resolution'][0]
    intrinsics_config['image_height'] = kalibr_data['resolution'][1]
    
    dist_params = OrderedDict()
    dist_params['k1'] = kalibr_data['distortion_coeffs'][0]
    dist_params['k2'] = kalibr_data['distortion_coeffs'][1]
    dist_params['p1'] = kalibr_data['distortion_coeffs'][2]
    dist_params['p2'] = kalibr_data['distortion_coeffs'][3]
    intrinsics_config['distortion_parameters'] = dist_params

    proj_params = OrderedDict()
    proj_params['fx'] = kalibr_data['intrinsics'][0]
    proj_params['fy'] = kalibr_data['intrinsics'][1]
    proj_params['cx'] = kalibr_data['intrinsics'][2]
    proj_params['cy'] = kalibr_data['intrinsics'][3]
    intrinsics_config['projection_parameters'] = proj_params

    intrinsics_path = os.path.join(output_dir, "camera_config.yaml")
    with open(intrinsics_path, 'w') as f:
        f.write("%YAML:1.0\n---\n")
        f.write("# ===================================================================================\n")
        f.write("# VINS-Fusion Camera Intrinsic File\n")
        f.write("# ===================================================================================\n")
        # Use the custom Dumper to avoid curly braces
        yaml.dump(intrinsics_config, f, Dumper=NoAliasDumper, sort_keys=False, default_flow_style=False, width=1000)

    # --- Create Main Config File (vins_mono_imu_config.yaml) ---
    # (The logic for this file remains the same as it was manually written)
    main_config = OrderedDict()
    main_config['imu'] = 1
    main_config['num_of_cam'] = 1
    main_config['imu_topic'] = imu_data['rostopic']
    main_config['image0_topic'] = kalibr_data['rostopic']
    main_config['cam0_calib'] = "camera_config.yaml"
    main_config['image_width'] = kalibr_data['resolution'][0]
    main_config['image_height'] = kalibr_data['resolution'][1]
    
    T_cam_imu = np.array(kalibr_data['T_cam_imu'])
    T_imu_cam_inverse = np.linalg.inv(T_cam_imu)
    
    main_config['estimate_extrinsic'] = 0
    main_config['body_T_cam0'] = format_opencv_matrix(T_imu_cam_inverse)
    main_config['multiple_thread'] = 1
    main_config['max_cnt'] = 150
    main_config['min_dist'] = 30
    main_config['freq'] = 10
    main_config['F_threshold'] = 1.0
    main_config['show_track'] = 1
    main_config['flow_back'] = 1
    main_config['max_solver_time'] = 0.04
    main_config['max_num_iterations'] = 8
    main_config['keyframe_parallax'] = 10.0
    main_config['acc_n'] = imu_data['accelerometer_noise_density']
    main_config['gyr_n'] = imu_data['gyroscope_noise_density']
    main_config['acc_w'] = imu_data['accelerometer_random_walk']
    main_config['gyr_w'] = imu_data['gyroscope_random_walk']
    main_config['g_norm'] = 9.81
    main_config['estimate_td'] = 0
    main_config['td'] = kalibr_data['timeshift_cam_imu']
    main_config['load_previous_pose_graph'] = 0
    main_config['pose_graph_save_path'] = "/root/output/pose_graph/"
    main_config['save_image'] = 1

    main_path = os.path.join(output_dir, "vins_mono_imu_config.yaml")
    with open(main_path, 'w') as f:
        f.write("%YAML:1.0\n")
        f.write("# ===================================================================================\n")
        f.write("# VINS-Fusion Monocular-Inertial Configuration File\n")
        f.write("# ===================================================================================\n")
        for key, value in main_config.items():
            if key == 'body_T_cam0':
                f.write(f"body_T_cam0: {value}\n")
            else:
                yaml.dump({key: value}, f, Dumper=NoAliasDumper, sort_keys=False, default_flow_style=False)

    print(f"Successfully generated VINS-Fusion configs in: {output_dir}")

def main():
    parser = argparse.ArgumentParser(description="Convert Kalibr to a two-file VINS-Fusion format.")
    parser.add_argument("--kalibr_file", type=str, required=True, help="Path to 'camchain-imucam.yaml'.")
    parser.add_argument("--imu_file", type=str, required=True, help="Path to 'imu.yaml'.")
    parser.add_argument("--output_dir", type=str, required=True, help="Directory to save the two config files.")
    args = parser.parse_args()

    kalibr_data = parse_kalibr_yaml(args.kalibr_file)
    imu_data = parse_imu_yaml(args.imu_file)
    generate_files(kalibr_data, imu_data, args.output_dir)

if __name__ == "__main__":
    main()