#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from PyQt5.QtWidgets import QLineEdit,QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QFont
import sys
import os 
from datetime import datetime
import math
import numpy as np

class PitchControl(QWidget):
    def __init__(self):
        super().__init__()

        # setup the width of display and input bar
        self.width_label = 100
        self.width_input_txt = 100

        # Initialize data to 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.factor_angle=2

        # Set up the GUI
        self.init_ui()

        # Set up ROS node and publisher
        rospy.init_node('pitch_control')
        self.pitch_pub = rospy.Publisher('manual_calib_pitch', Float64, queue_size=10)
        self.roll_pub = rospy.Publisher('manual_calib_roll', Float64, queue_size=10)
        self.yaw_pub = rospy.Publisher('manual_calib_yaw', Float64, queue_size=10)
        self.x_pub = rospy.Publisher('manual_calib_x', Float64, queue_size=10)
        self.y_pub = rospy.Publisher('manual_calib_y', Float64, queue_size=10)
        self.z_pub = rospy.Publisher('manual_calib_z', Float64, queue_size=10)
        
    def init_ui(self):
        # Set font for buttons and display
        font = QFont()
        font.setPointSize(16)

        # Create buttons
        plus_pitch_btn = QPushButton("+")
        plus_pitch_btn.setFont(font)
        minus_pitch_btn = QPushButton("-")
        minus_pitch_btn.setFont(font)

        plus_roll_btn = QPushButton("+")
        plus_roll_btn.setFont(font)
        minus_roll_btn = QPushButton("-")
        minus_roll_btn.setFont(font)
    
        plus_yaw_btn = QPushButton("+")
        plus_yaw_btn.setFont(font)
        minus_yaw_btn = QPushButton("-")
        minus_yaw_btn.setFont(font)

        plus_x_btn = QPushButton("+")
        plus_x_btn.setFont(font)
        minus_x_btn = QPushButton("-")
        minus_x_btn.setFont(font)

        plus_y_btn = QPushButton("+")
        plus_y_btn.setFont(font)
        minus_y_btn = QPushButton("-")
        minus_y_btn.setFont(font)

        plus_z_btn = QPushButton("+")
        plus_z_btn.setFont(font)
        minus_z_btn = QPushButton("-")
        minus_z_btn.setFont(font)

        # Create display labels
        self.pitch_display_lbl = QLabel(str(self.pitch))
        self.pitch_display_lbl.setFont(font)
        self.pitch_display_lbl.setMaximumWidth(self.width_label)  # Set maximum width of label to self.width_label

        self.roll_display_lbl = QLabel(str(self.roll))
        self.roll_display_lbl.setFont(font)
        self.roll_display_lbl.setMaximumWidth(self.width_label)  # Set maximum width of label to self.width_label

        self.yaw_display_lbl = QLabel(str(self.yaw))
        self.yaw_display_lbl.setFont(font)
        self.yaw_display_lbl.setMaximumWidth(self.width_label)

        self.x_display_lbl = QLabel(str(self.x))
        self.x_display_lbl.setFont(font)
        self.x_display_lbl.setMaximumWidth(self.width_label)

        self.y_display_lbl = QLabel(str(self.y))
        self.y_display_lbl.setFont(font)
        self.y_display_lbl.setMaximumWidth(self.width_label)

        self.z_display_lbl = QLabel(str(self.z))
        self.z_display_lbl.setFont(font)
        self.z_display_lbl.setMaximumWidth(self.width_label)

        # Create input text windows
        self.pitch_input = QLineEdit(str(self.pitch))
        self.pitch_input.setFont(font)
        self.pitch_input.setMaximumWidth(self.width_input_txt)  # Set maximum width of label to self.width_input_txt
        self.pitch_input.returnPressed.connect(self.update_pitch)

        self.roll_input = QLineEdit(str(self.roll))
        self.roll_input.setFont(font)
        self.roll_input.setMaximumWidth(self.width_input_txt)  # Set maximum width of label to self.width_input_txt
        self.roll_input.returnPressed.connect(self.update_roll)

        self.yaw_input = QLineEdit(str(self.yaw))
        self.yaw_input.setFont(font)
        self.yaw_input.setMaximumWidth(self.width_input_txt)
        self.yaw_input.returnPressed.connect(self.update_yaw)

        self.x_input = QLineEdit(str(self.x))
        self.x_input.setFont(font)
        self.x_input.setMaximumWidth(self.width_input_txt)
        self.x_input.returnPressed.connect(self.update_x)

        self.y_input = QLineEdit(str(self.y))
        self.y_input.setFont(font)
        self.y_input.setMaximumWidth(self.width_input_txt)
        self.y_input.returnPressed.connect(self.update_y)

        self.z_input = QLineEdit(str(self.z))
        self.z_input.setFont(font)
        self.z_input.setMaximumWidth(self.width_input_txt)
        self.z_input.returnPressed.connect(self.update_z)

        self.display_pitch_name = QLabel(str('Pitch'))
        self.display_pitch_name.setFont(font)

        self.display_roll_name = QLabel(str('Roll'))
        self.display_roll_name.setFont(font)

        self.display_yaw_name = QLabel(str('Yaw'))
        self.display_yaw_name.setFont(font)

        self.display_x_name = QLabel(str('X'))
        self.display_x_name.setFont(font)

        self.display_y_name = QLabel(str('Y'))
        self.display_y_name.setFont(font)

        self.display_z_name = QLabel(str('Z'))
        self.display_z_name.setFont(font)

        # Connect button signals to slots
        # pitch buttons
        plus_pitch_btn.clicked.connect(self.increment_pitch)
        minus_pitch_btn.clicked.connect(self.decrement_pitch)

        # Roll buttons
        plus_roll_btn.clicked.connect(self.increment_roll)
        minus_roll_btn.clicked.connect(self.decrement_roll)

        # Yaw buttons
        plus_yaw_btn.clicked.connect(self.increment_yaw)
        minus_yaw_btn.clicked.connect(self.decrement_yaw)

        # X buttons
        plus_x_btn.clicked.connect(self.increment_x)
        minus_x_btn.clicked.connect(self.decrement_x)

        # Y buttons
        plus_y_btn.clicked.connect(self.increment_y)
        minus_y_btn.clicked.connect(self.decrement_y)

        # Z buttons
        plus_z_btn.clicked.connect(self.increment_z)
        minus_z_btn.clicked.connect(self.decrement_z)

        # Create save button
        save_btn = QPushButton("Save")
        save_btn.setFont(font)

        # get the current working directory
        directory_path = os.getcwd() # normally, this should be the workspace directory
        parameters_path = "/src/lidar_camera_calibration_gui/parameters"
        directory_path = directory_path + parameters_path        

        # if the directory path doesn't exist, create one
        if not os.path.exists(directory_path):
            os.makedirs(directory_path)
        filename = "parameters_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".yaml"
        full_path = os.path.join(directory_path, filename)

        # Connect save button signal to slot
        save_btn.clicked.connect(lambda: self.save_parameters(full_path))

        # Set up layout
        h_pitch_layout = QHBoxLayout()
        h_pitch_layout.addWidget(self.display_pitch_name)
        h_pitch_layout.addWidget(self.pitch_input)
        h_pitch_layout.addWidget(minus_pitch_btn)
        h_pitch_layout.addWidget(plus_pitch_btn)
        h_pitch_layout.addWidget(self.pitch_display_lbl)

        h_roll_layout = QHBoxLayout()
        h_roll_layout.addWidget(self.display_roll_name)
        h_roll_layout.addWidget(self.roll_input)
        h_roll_layout.addWidget(minus_roll_btn)
        h_roll_layout.addWidget(plus_roll_btn)
        h_roll_layout.addWidget(self.roll_display_lbl)

        h_yaw_layout = QHBoxLayout()
        h_yaw_layout.addWidget(self.display_yaw_name)
        h_yaw_layout.addWidget(self.yaw_input)
        h_yaw_layout.addWidget(minus_yaw_btn)
        h_yaw_layout.addWidget(plus_yaw_btn)
        h_yaw_layout.addWidget(self.yaw_display_lbl)

        h_x_layout = QHBoxLayout()
        h_x_layout.addWidget(self.display_x_name)
        h_x_layout.addWidget(self.x_input)
        h_x_layout.addWidget(minus_x_btn)
        h_x_layout.addWidget(plus_x_btn)
        h_x_layout.addWidget(self.x_display_lbl)

        h_y_layout = QHBoxLayout()
        h_y_layout.addWidget(self.display_y_name)
        h_y_layout.addWidget(self.y_input)
        h_y_layout.addWidget(minus_y_btn)
        h_y_layout.addWidget(plus_y_btn)
        h_y_layout.addWidget(self.y_display_lbl)

        h_z_layout = QHBoxLayout()
        h_z_layout.addWidget(self.display_z_name)
        h_z_layout.addWidget(self.z_input)
        h_z_layout.addWidget(minus_z_btn)
        h_z_layout.addWidget(plus_z_btn)
        h_z_layout.addWidget(self.z_display_lbl)

        v_layout = QVBoxLayout()
        v_layout.addLayout(h_pitch_layout)
        v_layout.addLayout(h_roll_layout)
        v_layout.addLayout(h_yaw_layout)
        v_layout.addLayout(h_x_layout)
        v_layout.addLayout(h_y_layout)
        v_layout.addLayout(h_z_layout)
        v_layout.addWidget(save_btn)

        # Set main layout
        self.setLayout(v_layout)

    # ======================save parameters===========================
    def save_parameters(self, file_path):
        with open(file_path, 'w') as f:
            f.write('pitch={}\n'.format(self.pitch))
            f.write('pitch={}\n'.format(self.pitch))
            f.write('roll={}\n'.format(self.roll))
            f.write('yaw={}\n'.format(self.yaw))
            f.write('x={}\n'.format(self.x))
            f.write('y={}\n'.format(self.y))
            f.write('z={}\n'.format(self.z))            

        rospy.loginfo("The file has been saved to: {}".format(file_path))

    # ======================transform matrix===========================
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

    # ======================update===========================
    def update_pitch(self):
        self.pitch = float(self.pitch_input.text())
        self.pitch_display_lbl.setText(str(self.pitch))
        self.publish_pitch()

    def update_roll(self):
        self.roll = float(self.roll_input.text())
        self.roll_display_lbl.setText(str(self.roll))
        self.publish_roll()

    def update_yaw(self):
        self.yaw = float(self.yaw_input.text())
        self.yaw_display_lbl.setText(str(self.yaw))
        self.publish_yaw()

    def update_x(self):
        self.x = float(self.x_input.text())
        self.x_display_lbl.setText(str(self.x))
        self.publish_x()

    def update_y(self):
        self.y = float(self.y_input.text())
        self.y_display_lbl.setText(str(self.y))
        self.publish_y()

    def update_z(self):
        self.z = float(self.z_input.text())
        self.z_display_lbl.setText(str(self.z))
        self.publish_z()

    # ======================increment/decrement===========================
    def increment_pitch(self):
        self.pitch += 0.01*self.factor_angle
        self.pitch_display_lbl.setText(str(self.pitch))
        self.publish_pitch()

    def decrement_pitch(self):
        self.pitch -= 0.01*self.factor_angle
        self.pitch_display_lbl.setText(str(self.pitch))
        self.publish_pitch()

    def increment_roll(self):
        self.roll += 0.01*self.factor_angle
        self.roll_display_lbl.setText(str(self.roll))
        self.publish_roll()

    def decrement_roll(self):
        self.roll -= 0.01*self.factor_angle
        self.roll_display_lbl.setText(str(self.roll))
        self.publish_roll()

    def increment_yaw(self):
        self.yaw += 0.01*self.factor_angle
        self.yaw_display_lbl.setText(str(self.yaw))
        self.publish_yaw()

    def decrement_yaw(self):
        self.yaw -= 0.01*self.factor_angle
        self.yaw_display_lbl.setText(str(self.yaw))
        self.publish_yaw()

    def increment_x(self):
        self.x += 0.01
        self.x_display_lbl.setText(str(self.x))
        self.publish_x()

    def decrement_x(self):
        self.x -= 0.01
        self.x_display_lbl.setText(str(self.x))
        self.publish_x()

    def increment_y(self):
        self.y += 0.01
        self.y_display_lbl.setText(str(self.y))
        self.publish_y()

    def decrement_y(self):
        self.y -= 0.01
        self.y_display_lbl.setText(str(self.y))
        self.publish_y()

    def increment_z(self):
        self.z += 0.01
        self.z_display_lbl.setText(str(self.z))
        self.publish_z()

    def decrement_z(self):
        self.z -= 0.01
        self.z_display_lbl.setText(str(self.z))
        self.publish_z()


    def publish_pitch(self):
        # Publish pitch data to ROS topic
        self.pitch_pub.publish(self.pitch)
    
    def publish_roll(self):
        # Publish roll data to ROS topic
        self.roll_pub.publish(self.roll)   

    def publish_yaw(self):
        # Publish yaw data to ROS topic
        self.yaw_pub.publish(self.yaw)

    def publish_x(self):
        # Publish x data to ROS topic
        self.x_pub.publish(self.x)

    def publish_y(self):
        # Publish y data to ROS topic
        self.y_pub.publish(self.y)

    def publish_z(self):
        # Publish z data to ROS topic
        self.z_pub.publish(self.z)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    pc = PitchControl()
    pc.show()
    sys.exit(app.exec_())
