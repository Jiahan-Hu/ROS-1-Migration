#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from PyQt5.QtWidgets import QLineEdit,QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QFont
import sys
import os 
from datetime import datetime

class PitchControl(QWidget):
    def __init__(self):
        super().__init__()

        # setup the width of display and input bar
        self.width_label = 100
        self.width_input_txt = 100

        # Initialize data to 0.0
        # for left cam
        # self.pitch=-0.712594
        # self.roll=-109.082
        # self.yaw=-83.3685
        # self.x=-0.05424999999999979
        # self.y=0.025678400000000004
        # self.z=1.46805
        # for fl
        # self.pitch=-0.34402000000000044
        # self.roll=-92.9293
        # self.yaw=-102.6
        # self.x=0.37265400000000015
        # self.y=1.0800000000000014
        # self.z=-0.5700000000000004
        # for left_cam_left_corner_lidar
        # self.pitch=88.63150000000002
        # self.roll=-0.7125939999999966
        # self.yaw=-109.08200000000001
        # self.x=-2.2098516998264626
        # self.y=-0.2929786403034571
        # self.z=0.2680499999999999
        # new parameters for left_cam_left_corner_lidar
        # self.pitch=-0.712594
        # self.roll=-112.56200000000007
        # self.yaw=89.28149999999943
        # self.x=-3.548121643069991
        # self.y=-0.5471203315260003
        # self.z=-0.04000000000000006
        # right camera for white truck
        # self.pitch=0.919948
        # self.roll=-109.235
        # self.yaw=-99.8127
        # self.x=-0.549143
        # self.y=-0.019592
        # self.z=1.4142
        # fron right camera for white truck        
        # self.pitch=-92.3586
        # self.roll=-1.96538
        # self.yaw=-95.4296
        # self.x=0.802553
        # self.y=0.627499
        # self.z=-0.442336

        # # infra 214 mid
        # self.pitch=6.59057
        # self.roll=-100.852
        # self.yaw=-56.813100000000006
        # self.x=0.1192855
        # self.y=0.054994999999999995
        # self.z=0.312747
        # infra 214 final 
        # self.pitch=3.8905699999999968
        # self.roll=-95.55200000000002
        # self.yaw=-102.11309999999966
        # self.x=-0.8807144999999998
        # self.y=-0.34500500000000006
        # self.z=5.112746999999997

        # infra 214_final
        # self.pitch=4.499569999999989
        # self.roll=-95.55200000000002
        # self.yaw=-101.88409999999966
        # self.x=-0.9807144999999998
        # self.y=-0.5750050000000002
        # self.z=5.112746999999997
        
        # infra 215
        self.pitch=6.02541
        self.roll=-100.24100000000003
        self.yaw=-155.496
        self.x=-0.3731789999999999
        self.y=-0.564187
        self.z=4.7645300000000015
        
        self.factor_angle=100
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

        plus_fa_btn = QPushButton("+")
        plus_fa_btn.setFont(font)
        minus_fa_btn = QPushButton("-")
        minus_fa_btn.setFont(font)

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

        self.fa_display_lbl = QLabel(str(self.factor_angle))
        self.fa_display_lbl.setFont(font)
        self.fa_display_lbl.setMaximumWidth(self.width_label)

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

        self.fa_input = QLineEdit(str(self.factor_angle))
        self.fa_input.setFont(font)
        self.fa_input.setMaximumWidth(self.width_input_txt)
        self.fa_input.returnPressed.connect(self.update_factor_angle)

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

        self.display_fa_name = QLabel(str('FA'))
        self.display_fa_name.setFont(font)

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

        # FA buttons
        plus_fa_btn.clicked.connect(self.increment_fa)
        minus_fa_btn.clicked.connect(self.decrement_fa)

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
        filename = "parameters_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".txt"
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

        h_fa_layout = QHBoxLayout()
        h_fa_layout.addWidget(self.display_fa_name)
        h_fa_layout.addWidget(self.fa_input)
        h_fa_layout.addWidget(minus_fa_btn)
        h_fa_layout.addWidget(plus_fa_btn)
        h_fa_layout.addWidget(self.fa_display_lbl)

        v_layout = QVBoxLayout()
        v_layout.addLayout(h_pitch_layout)
        v_layout.addLayout(h_roll_layout)
        v_layout.addLayout(h_yaw_layout)
        v_layout.addLayout(h_x_layout)
        v_layout.addLayout(h_y_layout)
        v_layout.addLayout(h_z_layout)
        v_layout.addLayout(h_fa_layout)
        v_layout.addWidget(save_btn)

        # Set main layout
        self.setLayout(v_layout)

    # ======================save parameters===========================
    def save_parameters(self, file_path):
        with open(file_path, 'w') as f:
            f.write('self.pitch={}\n'.format(self.pitch))
            f.write('self.roll={}\n'.format(self.roll))
            f.write('self.yaw={}\n'.format(self.yaw))
            f.write('self.x={}\n'.format(self.x))
            f.write('self.y={}\n'.format(self.y))
            f.write('self.z={}\n'.format(self.z))            

        rospy.loginfo("The file has been saved to: {}".format(file_path))

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
    
    def update_factor_angle(self):
        self.factor_angle = float(self.fa_input.text())
        self.fa_display_lbl.setText(str(self.factor_angle))

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
        self.x += 0.01*self.factor_angle
        self.x_display_lbl.setText(str(self.x))
        self.publish_x()

    def decrement_x(self):
        self.x -= 0.01*self.factor_angle
        self.x_display_lbl.setText(str(self.x))
        self.publish_x()

    def increment_y(self):
        self.y += 0.01*self.factor_angle
        self.y_display_lbl.setText(str(self.y))
        self.publish_y()

    def decrement_y(self):
        self.y -= 0.01*self.factor_angle
        self.y_display_lbl.setText(str(self.y))
        self.publish_y()

    def increment_z(self):
        self.z += 0.01*self.factor_angle
        self.z_display_lbl.setText(str(self.z))
        self.publish_z()

    def decrement_z(self):
        self.z -= 0.01*self.factor_angle
        self.z_display_lbl.setText(str(self.z))
        self.publish_z()

    def increment_fa(self):
        self.factor_angle = self.factor_angle * 10
        self.fa_display_lbl.setText(str(self.factor_angle))

    def decrement_fa(self):
        self.factor_angle = self.factor_angle / 10
        self.fa_display_lbl.setText(str(self.factor_angle))


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
