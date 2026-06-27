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

        # Initialize pitch data to 0.0
        self.pitch = 0.0 
        # Set up the GUI
        self.init_ui()

        # Set up ROS node and publisher
        rospy.init_node('pitch_control')
        self.pitch_pub = rospy.Publisher('pitch', Float64, queue_size=10)

    def init_ui(self):
        # Set font for buttons and display
        font = QFont()
        font.setPointSize(16)

        # Create buttons
        plus_btn = QPushButton("+")
        plus_btn.setFont(font)
        minus_btn = QPushButton("-")
        minus_btn.setFont(font)

        # Create display label
        self.display_lbl = QLabel(str(self.pitch))
        self.display_lbl.setFont(font)
        self.display_lbl.setMaximumWidth(self.width_label)  # Set maximum width of label to self.width_label

        # Create input text window
        self.pitch_input = QLineEdit(str(self.pitch))
        self.pitch_input.setFont(font)
        self.pitch_input.setMaximumWidth(self.width_input_txt)  # Set maximum width of label to self.width_input_txt
        self.pitch_input.returnPressed.connect(self.update_pitch)

        self.display_name = QLabel(str('Pitch'))
        self.display_name.setFont(font)

        # Create save button
        save_btn = QPushButton("Save")
        save_btn.setFont(font)

        # Connect button signals to slots
        plus_btn.clicked.connect(self.increment_pitch)
        minus_btn.clicked.connect(self.decrement_pitch)

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
        h_layout = QHBoxLayout()
        h_layout.addWidget(self.display_name)
        h_layout.addWidget(self.pitch_input)
        h_layout.addWidget(minus_btn)
        h_layout.addWidget(plus_btn)
        h_layout.addWidget(self.display_lbl)
        
        v_layout = QVBoxLayout()
        v_layout.addLayout(h_layout)
        v_layout.addWidget(save_btn)

        # Set main layout
        self.setLayout(v_layout)

    def save_parameters(self, file_path):
        with open(file_path, 'w') as f:
            f.write('pitch={}\n'.format(self.pitch))

        rospy.loginfo("The file has been saved to: {}".format(file_path))

    def update_pitch(self):
        self.pitch = float(self.pitch_input.text())
        self.display_lbl.setText(str(self.pitch))
        self.publish_pitch()

    def increment_pitch(self):
        self.pitch += 0.1
        self.display_lbl.setText(str(self.pitch))
        self.publish_pitch()

    def decrement_pitch(self):
        self.pitch -= 0.1
        self.display_lbl.setText(str(self.pitch))
        self.publish_pitch()

    def publish_pitch(self):
        # Publish pitch data to ROS topic
        self.pitch_pub.publish(self.pitch)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    pc = PitchControl()
    pc.show()
    sys.exit(app.exec_())

