import yaml

# Read the YAML file
with open('cam_lidar_calibration.yaml', 'r') as input_file:
    yaml_data = yaml.safe_load(input_file)

# Modify the CameraExtrinsicMat.data
new_data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
yaml_data['CameraExtrinsicMat']['data'] = new_data

# Write the modified data to a new YAML file
with open('output.yaml', 'w') as output_file:
    yaml.safe_dump(yaml_data, output_file, default_flow_style=None)
