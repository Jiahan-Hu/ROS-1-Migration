# -*- coding: utf-8 -*-
"""
Code description.
"""
# Author: Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import yaml
import os
import glob

def modify_yaml_files(dir_path,output_path):
    """AI is creating summary for batch_yaml2intrinsic

    Args:
        dir_path ([type]): [description]
        output_path ([type]): [description]
    """
    # Ensure that the output directory exists; create it if not
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # Define the additional data to be added
    additional_data = {
        'CameraMat': {
            'cols': 3,
            'data': [1069.994384765625, 0.0, 921.9642944335938, 0.0, 1069.994384765625, 516.6058959960938, 0.0, 0.0, 1.0],
            'dt': 'd',
            'rows': 3
        },
        'DistCoeff': {
            'cols': 5,
            'data': [0.0, 0.0, 0.0, 0.0, 0.0],
            'dt': 'd',
            'rows': 1
        },
        'DistModel': 'plumb_bob',
        'ImageSize': [1920, 1080],
        'ReprojectionError': 0
    }

    # Read each YAML file in the input folder, modify, and save to output folder
    for file_path in glob.glob(os.path.join(dir_path, '*.yaml')):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

            # Modify the data
            data.update(additional_data)

            # Write the modified data to the output folder
            output_file_path = os.path.join(output_path, os.path.basename(file_path))
            with open(output_file_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)

        # print out some information to know if one conversion is done  
        print(f'Converted {file_path} to {output_file_path}')
    

if __name__ == "__main__":
    dir_path = "/home/zhaoliang/zzl/zhz03_github/calibration/zzl_xx_calib_results-20231115T052725Z-001/zzl_xx_calib_results/output"
    output_path = "/home/zhaoliang/zzl/zhz03_github/calibration/zzl_xx_calib_results-20231115T052725Z-001/zzl_xx_calib_results/output/results"
    modify_yaml_files(dir_path, output_path)

