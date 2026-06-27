import math
import numpy as np
import yaml
import os

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

def read_yaml_and_extract(input_file):
    # Load the data from the YAML file
    with open(input_file, 'r') as file:
        yaml_data = yaml.safe_load(file)

    # Extract the transformation matrix from the 'CameraExtrinsicMat' dictionary
    M = np.array(yaml_data['CameraExtrinsicMat']['data']).reshape((4, 4))

    # Extract the rotation matrix and translation vector from the transformation matrix
    R = M[0:3, 0:3]
    T = M[0:3, 3]

    # Compute the pitch, roll, and yaw angles from the rotation matrix
    pitch = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    roll = math.degrees(math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2)))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    # Compute the translation vector
    x = T[0]
    y = T[1]
    z = T[2]

    # Print the extracted values
    print('Pitch: {}'.format(pitch))
    print('Roll: {}'.format(roll))
    print('Yaw: {}'.format(yaw))
    print('x: {}'.format(x))
    print('y: {}'.format(y))
    print('z: {}'.format(z))

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
    print('Pitch: {}'.format(pitch))
    print('Roll: {}'.format(roll))
    print('Yaw: {}'.format(yaw))
    print('x: {}'.format(x))
    print('y: {}'.format(y))
    print('z: {}'.format(z))

    return pitch,roll,yaw,x,y,z

if __name__ == '__main__':
    path = "/home/zhaoliang/zzl/zhz03_github/lidar_camera_calibration/lidar_cam_ws/src/lidar_camera_calibration_gui/parameters"
    txt_name = "parameters_2023-04-15_23-56-27.txt"
    txt_file = os.path.join(path,txt_name)
    pitch,roll,yaw,x,y,z = read_txt2pryxyz(txt_file)
    # Define the camera transformation parameters
    # pitch=-0.182393
    # roll=-89.7181
    # yaw=-88.32740000000013
    # x=0.761997
    # y=0.11216959999999998
    # z=-0.605502

    # Get the transformation matrix
    M = transform_matrix(pitch, roll, yaw, x, y, z)
    input_file = 'forward_camera_calibration_mod.yaml'
    output_file = 'forward_camera_calibration_new2.yaml'
    # print(M.flatten().tolist()[0:])
    # new_data = M.flatten().tolist()
    # Read the YAML file and change the 'data' field of the 'CameraExtrinsicMat' dictionary

    read_yaml_and_change_M(input_file,M,output_file)
