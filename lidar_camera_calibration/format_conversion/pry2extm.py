# -*- coding: utf-8 -*-
"""
This script reads data from xxx.txt files, calculates extrinsic matrices,
and writes the results to corresponding xxx.yaml files.
"""
# Author: Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import re
import numpy as np
import os
import yaml

def read_values_from_txt1(file_path):
    """Read and parse values from a text file.

    Args:
        file_path (str): The path to the input text file.

    Returns:
        dict: A dictionary containing parsed values.
    """
    values = {}
    with open(file_path, 'r') as file:
        for line in file:
            # this code only works for the format: 'self.variable=value'
            match = re.match(r'self\.(\w+)=(.*)', line)
            if match:
                # Extract the variable name and its corresponding value
                variable_name = match.group(1)
                variable_value = float(match.group(2))
                values[variable_name] = variable_value
    return values

def read_values_from_txt(file_path):
    """Read and parse values from a text file.

    Args:
        file_path (str): The path to the input text file.

    Returns:
        dict: A dictionary containing parsed values.
    """
    values = {}
    with open(file_path, 'r') as file:
        for line in file:
            # Match both formats: 'self.variable=value' and 'variable = value'
            match = re.match(r'(self\.)?(\w+)\s*=\s*(.*)', line)
            if match:
                # Extract the variable name and its corresponding value
                variable_name = match.group(2)
                variable_value = float(match.group(3))
                values[variable_name] = variable_value
    return values

def calculate_extrinsic_matrix(values):
    """Calculate the extrinsic matrix based on pitch, roll, yaw, and translation values.

    Args:
        values (dict): A dictionary containing pitch, roll, yaw, x, y, and z values.

    Returns:
        numpy.ndarray: The calculated extrinsic matrix.
    """
    # Convert pitch, roll, and yaw values from degrees to radians
    pitch_degrees, roll_degrees, yaw_degrees = values['pitch'], values['roll'], values['yaw']
    pitch, roll, yaw = np.radians([values['pitch'], values['roll'], values['yaw']])
    
    # Extract translation values
    x, y, z = values['x'], values['y'], values['z']

    # print pitch, roll, yaw in degree/radians and x,y,z
    print(f"Pitch (degrees): {pitch_degrees}, Pitch (radians): {pitch}")
    print(f"Roll (degrees): {roll_degrees}, Roll (radians): {roll}")
    print(f"Yaw (degrees): {yaw_degrees}, Yaw (radians): {yaw}")
    print(f"Translation (x, y, z): ({x}, {y}, {z})")

    # Calculate rotation matrices for X, Y, and Z axes
    # R_x = np.array([[1, 0, 0],
    #                 [0, np.cos(pitch), -np.sin(pitch)],
    #                 [0, np.sin(pitch), np.cos(pitch)]])
    
    # R_y = np.array([[np.cos(roll), 0, np.sin(roll)],
    #                 [0, 1, 0],
    #                 [-np.sin(roll), 0, np.cos(roll)]])
    
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine the rotation matrices to get the overall rotation matrix R
    R = R_z @ R_y @ R_x

    # Create the translation vector T
    T = np.array([[x], [y], [z]])

    # Combine R and T to form the extrinsic matrix
    extrinsic_matrix = np.hstack((R, T))
    extrinsic_matrix = np.vstack((extrinsic_matrix, [0, 0, 0, 1]))

    return extrinsic_matrix

def write_to_yaml1(matrix, output_path):
    """Write the extrinsic matrix to a YAML file.

    Args:
        matrix (numpy.ndarray): The extrinsic matrix to be written.
        output_path (str): The path to the output YAML file.
    """
    # If output_path doesn't exist, create an output_path
    if not os.path.exists(os.path.dirname(output_path)):
        os.makedirs(os.path.dirname(output_path))
    
    item =str(matrix.flatten().tolist())
    data = {
        'CameraExtrinsicMat': {
            'cols': 4,
            'data': matrix.flatten().tolist()  # Convert to Python list
        },
        'dt': 'd',  # Add data type information
        'rows': 4   # Add rows information
    }
    # Write the 'data' dictionary to the specified 'output_path' YAML file
    # with the given formatting options (default_flow_style=False, default_style='plain').
    # with open(output_path, 'w') as file:
    #     yaml.dump(data, file, default_flow_style=False, default_style='plain')
    # with open(output_path, 'w') as file:
    #     yaml.dump(data, file, default_flow_style=False)
    # with open(output_path, 'w') as file:
    #     yaml.dump(data, file, default_flow_style=False)
    with open(output_path, 'w') as file:
        # yaml.dump(data, file, default_style='', indent=2)
        # yaml.dump(data, file, default_style='', default_flow_style=False, indent=2)
        yaml.dump(data, file, default_style='', default_flow_style=False, Dumper=yaml.Dumper, indent=2)

def write_to_yaml2(matrix, output_path):
    """Write the extrinsic matrix to a YAML file.

    Args:
        matrix (numpy.ndarray): The extrinsic matrix to be written.
        output_path (str): The path to the output YAML file.
    """
    # If output_path doesn't exist, create an output_path
    if not os.path.exists(os.path.dirname(output_path)):
        os.makedirs(os.path.dirname(output_path))

    # Convert matrix data to a single-line string
    data_str = '[' + ', '.join(f'{num:.8f}' for num in matrix.flatten()) + ']'

    data = {
        'CameraExtrinsicMat': {
            'cols': 4,
            'data': data_str,  # Use the formatted string here
            'dt': 'd',  # Add data type information
            'rows': 4   # Add rows information
        }
    }
    # Write the 'data' dictionary to the specified 'output_path' YAML file
    with open(output_path, 'w') as file:
        yaml.dump(data, file, default_flow_style=False, Dumper=yaml.SafeDumper, width=float("inf"))

def write_to_yaml(matrix, output_path):
    """Write the extrinsic matrix to a YAML file with custom formatting.

    Args:
        matrix (numpy.ndarray): The extrinsic matrix to be written.
        output_path (str): The path to the output YAML file.
    """
    # If output_path doesn't exist, create an output_path
    if not os.path.exists(os.path.dirname(output_path)):
        os.makedirs(os.path.dirname(output_path))

    matrix_data = matrix.flatten()
    data_str = ', '.join(f'{num:.8f}' for num in matrix_data)

    # camera_mat_data = "1069.994384765625, 0.0, 921.9642944335938, 0.0, 1069.994384765625, 516.6058959960938, 0.0, 0.0, 1.0"
    camera_mat_data = "[ 1.2216198999999999e+03, 0., 9.6175905999999998e+02, 0.,1.2230425100000000e+03, 5.5460495000000003e+02, 0., 0., 1. ]"
    with open(output_path, 'w') as file:
        file.write("CameraExtrinsicMat:\n")
        file.write("  cols: 4\n")
        file.write(f"  data: [{data_str}]\n")
        file.write("  dt: d\n")
        file.write("  rows: 4\n")
        # ----
        # CameraMat data
        file.write("CameraMat:\n")
        file.write("  cols: 3\n")
        
        file.write(f"  data: [{camera_mat_data}]\n")
        file.write("  dt: d\n")
        file.write("  rows: 3\n")

        # DistCoeff data
        file.write("DistCoeff:\n")
        file.write("  cols: 5\n")
        dist_coeff_data = "0.0, 0.0, 0.0, 0.0, 0.0"
        file.write(f"  data: [{dist_coeff_data}]\n")
        file.write("  dt: d\n")
        file.write("  rows: 1\n")

        # Other entries
        file.write("DistModel: plumb_bob\n")
        file.write("ImageSize: [1920, 1080]\n")
        file.write("ReprojectionError: 0\n")



def batch_txt2yaml(dir_path, output_path):
    """This function reads all the xxx.txt files in the dir_path, processes
    them, and outputs them as xxx.yaml files in the output_path.

    Args:
        dir_path (str): The directory containing xxx.txt files.
        output_path (str): The directory where xxx.yaml files will be saved.
    """
    # Ensure that the output directory exists; create it if not
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # Get a list of all txt files in the specified directory
    txt_files = [f for f in os.listdir(dir_path) if f.endswith('.txt')]

    for txt_file in txt_files:
        # Construct the full path to the input xxx.txt file
        txt_file_path = os.path.join(dir_path, txt_file)

        # Extract values from the txt file
        values = read_values_from_txt(txt_file_path)
        # print("----")
        # print("file name:",txt_file_path)
        # print(values)

        # Calculate the extrinsic matrix
        matrix = calculate_extrinsic_matrix(values)

        # Construct the full path to the output xxx.yaml file
        yaml_file_path = os.path.join(output_path, txt_file.replace('.txt', '.yaml'))

        # Write the extrinsic matrix to the corresponding yaml file
        write_to_yaml(matrix, yaml_file_path)

        # print out some information to know if one conversion is done
        print(f'Converted {txt_file} to {yaml_file_path}')

if __name__ == "__main__":
    
    dir_path = "/home/zhaoliang/zzl/calib_valid/infra_ext"
    output_path = "/home/zhaoliang/zzl/calib_valid/infra_ext/yaml"

    # dir_path = "/home/zhaoliang/zzl/zhz03_github/calibration/zzl_xx_calib_results-20231115T052725Z-001/zzl_xx_calib_results"
    # output_path = "/home/zhaoliang/zzl/zhz03_github/calibration/zzl_xx_calib_results-20231115T052725Z-001/zzl_xx_calib_results/output"
    batch_txt2yaml(dir_path, output_path)