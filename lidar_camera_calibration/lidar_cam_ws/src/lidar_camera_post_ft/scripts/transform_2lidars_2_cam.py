import os
import numpy as np
import yaml
import math
from datetime import datetime
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(angles):
    return R.from_euler('ZYX', angles, degrees=True).as_matrix()

def rotation_matrix_to_euler(matrix):
    return R.from_matrix(matrix).as_euler('ZYX', degrees=True)

def compose_rotations(rotation1, rotation2):
    return np.matmul(rotation1, rotation2)

def calculate_relative_angles(rotation_A_to_B, rotation_B_to_C):
    rotation_A_to_C = compose_rotations(rotation_A_to_B, rotation_B_to_C)
    return rotation_matrix_to_euler(rotation_A_to_C)

def transform_point(rotation, point):
    return np.matmul(rotation, point)

def read_txt2pryxyz(txt_file):
    with open(txt_file, 'r') as f:
        lines = f.readlines()
    # Extract the pitch, roll, yaw, x, y, and z values from the lines
    pitch = float(lines[0].split('=')[1])
    roll = float(lines[1].split('=')[1])
    yaw = float(lines[2].split('=')[1])
    x = float(lines[3].split('=')[1])
    y = float(lines[4].split('=')[1])
    z = float(lines[5].split('=')[1])
    # printout the pitch, roll, yaw, x, y, and z values
    print("-----------------------------------")
    print('Pitch: {}'.format(pitch))
    print('Roll: {}'.format(roll))
    print('Yaw: {}'.format(yaw))
    print('x: {}'.format(x))
    print('y: {}'.format(y))
    print('z: {}'.format(z))

    return pitch,roll,yaw,x,y,z

def transform_matrix(pitch, roll, yaw, x, y, z):
    # convert pitch, roll, and yaw to radians
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    yaw = math.radians(yaw)

    # calculate the rotation matrix
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(pitch), -math.sin(pitch)],
                   [0, math.sin(pitch), math.cos(pitch)]])

    Ry = np.array([[math.cos(roll), 0, math.sin(roll)],
                   [0, 1, 0],
                   [-math.sin(roll), 0, math.cos(roll)]])

    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0],
                   [0, 0, 1]])

    R = np.dot(Rz, np.dot(Ry, Rx))

    # calculate the translation vector
    T = np.array([x, y, z])

    # create the transformation matrix
    M = np.zeros((4, 4))
    M[0:3, 0:3] = R
    M[0:3, 3] = T
    M[3, 3] = 1

    return M

def read_yaml_and_change_M(input_file,M,output_file):

    # Load the data from the YAML file
    with open(input_file, 'r') as file:
        # data = yaml.load(file, Loader=yaml.FullLoader)
        yaml_data = yaml.safe_load(file)
    # Modify the 'data' field of the 'CameraExtrinsicMat' dictionary
    yaml_data['CameraExtrinsicMat']['data'] = M.flatten().tolist()
    # Print the modified data
    print(yaml_data)
    # Save the modified data to the YAML file
    with open(output_file, 'w') as file:
        yaml.safe_dump(yaml_data, file, default_flow_style=None)
    
    print("file saved to '{}'".format(output_file))

def save_parameters_to_txt(pitch,roll,yaw,x,y,z):
    current_dir = os.getcwd()
    # get the path of the parent directory
    parent_dir = os.path.dirname(current_dir)
    directory_path = parent_dir + '/parameters'
    # if the directory path doesn't exist, create one
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
    filename = "finalParameters_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".txt"
    full_path = os.path.join(directory_path, filename)

    with open(full_path, 'w') as f:
        f.write('self.pitch={}\n'.format(pitch))
        f.write('self.roll={}\n'.format(roll))
        f.write('self.yaw={}\n'.format(yaw))
        f.write('self.x={}\n'.format(x))
        f.write('self.y={}\n'.format(y))
        f.write('self.z={}\n'.format(z)) 
    print("Txt has been successfully saved!")

if __name__ == "__main__":
    """
    The position of A,B,C is described as in follows:
    A is camera, B is a Lidar and C is also a Lidar.
    
    From the top view:
      A*****
      *    *
      *    *
    B *    *
      C*****

    From the side view:
    A********C
    *        *
    *        B
    **********
    """
    ################# Get the parameters #################   
    # Get the AB txt file
    ABtxt_path = "/home/zhaoliang/zzl/zhz03_github/lidar_camera_calibration/lidar_cam_ws/src/lidar_transformer/parameters"
    ABtxt_name = "corner2side_lidar_extrinsics.txt"
    ABtxt_file = os.path.join(ABtxt_path,ABtxt_name)
    # Pitch, roll, and yaw of object B relative to object A (in degrees)
    # Position of object B relative to object A
    pitch_B_to_A, roll_B_to_A, yaw_B_to_A,x_B_to_A, y_B_to_A, z_B_to_A = read_txt2pryxyz(ABtxt_file)
    rotation_A_to_B = euler_to_rotation_matrix([yaw_B_to_A, pitch_B_to_A, roll_B_to_A])
    # Get the BC txt file
    BCtxt_path = "/home/zhaoliang/zzl/zhz03_github/lidar_camera_calibration/lidar_cam_ws/src/lidar_camera_calibration_gui/parameters/white_left"
    BCtxt_name = "parameters_2023-05-01_19-41-09.txt"
    BCtxt_file = os.path.join(BCtxt_path,BCtxt_name)
    # Pitch, roll, and yaw of object C relative to object B (in degrees)
    # Position of object C relative to object B
    pitch_C_to_B, roll_C_to_B, yaw_C_to_B,x_C_to_B, y_C_to_B, z_C_to_B = read_txt2pryxyz(BCtxt_file)
    rotation_B_to_C = euler_to_rotation_matrix([yaw_C_to_B, pitch_C_to_B, roll_C_to_B])

    ################# Calculation process #################
    # Calculate the pitch, roll, and yaw of object C relative to object A (in degrees)
    pitch_C_to_A, roll_C_to_A, yaw_C_to_A = calculate_relative_angles(rotation_A_to_B, rotation_B_to_C)

    # Calculate the position of object C relative to object A
    position_C_to_B = np.array([x_C_to_B, y_C_to_B, z_C_to_B])
    position_C_to_A = transform_point(rotation_A_to_B, position_C_to_B) + np.array([x_B_to_A, y_B_to_A, z_B_to_A])

    ################# Print out calculated parameters #################
    print("===================================================================")
    print(f"Pitch of C relative to A: {pitch_C_to_A:.2f}")
    print(f"Roll of C relative to A: {roll_C_to_A:.2f}")
    print(f"Yaw of C relative to A: {yaw_C_to_A:.2f}")
    print(f"Position of C relative to A: x={position_C_to_A[0]:.2f}, y={position_C_to_A[1]:.2f}, z={position_C_to_A[2]:.2f}")

    x = position_C_to_A[0]
    y = position_C_to_A[1]
    z = position_C_to_A[2]
    M = transform_matrix(pitch_C_to_A, roll_C_to_A, yaw_C_to_A, x, y, z)

    print("M:",M)

    save_parameters_to_txt(pitch_C_to_A, roll_C_to_A, yaw_C_to_A, x, y, z)
    # input_yaml_file = './yaml_input'
    # output_yaml_file = './yaml_output'

    # read_yaml_and_change_M(input_file,M,output_file)


