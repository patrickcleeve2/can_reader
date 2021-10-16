#!/usr/bin/env python3


from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *
import sys

import rclpy
from rclpy.node import Node
import yaml

import time
import numpy as np

from can_reader_msgs.msg import CarState



class CANDecoderUINode(Node):

    def __init__(self) -> None:
        super().__init__("can_decoder_ui")

        self.window = CANDecoderUIWindow()

        self.state_subscriber = self.create_subscription(
            CarState, "/mini/vehicle_state", self.state_callback, 10
        )

    def run_once(self):
        
        rclpy.spin_once(self)

    def state_callback(self, msg):

        self.get_logger().info(f"msg: {msg}")

        self.window.msg = msg
        self.window.refresh_layout()



class CANDecoderUIWindow(QMainWindow):

    def __init__(self) -> None:
        super().__init__()

        # background colour
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)

        self.vbox = QVBoxLayout()
        self.header_label = QLabel()
        self.header_label.setText("Car State")
        self.header_label.setStyleSheet("color: white")
        self.header_label.setAlignment(Qt.AlignCenter)
        self.header_label.setFont(QFont("Arial", 32, weight=QFont.Bold))
        self.vbox.addWidget(self.header_label)

        self.msg_labels = [] 
        for i in range(9):
            label = QLabel()
            label.setStyleSheet("color: white")
            # label.setText("Hello World")
            label.setFont(QFont("Arial", 24))
            label.setAlignment(Qt.AlignLeft)
            label.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
            self.vbox.addWidget(label)
            self.msg_labels.append(label)


        self.main_widget = QWidget()
        self.main_widget.setLayout(self.vbox)
        self.setCentralWidget(self.main_widget)
        self.setGeometry(300, 300, 600, 600)
        self.setWindowTitle("CAN Decoder UI")
        self.show()

    def refresh_layout(self):

        self.msg_labels[0].setText(f"VELOCITY: {self.msg.velocity:.2f} m/s")
        self.msg_labels[1].setText(f"THROTTLE: {self.msg.throttle:.2f} %")
        self.msg_labels[2].setText(f"STEER: {self.msg.steer:.2f} rad") # TODO: check units on this
        self.msg_labels[3].setText(f"STEER_RATE: {self.msg.steer_rate:.2f} rad/s") # TODO: check units on this
        self.msg_labels[4].setText(f"BRAKE: {self.msg.brake:.2f} %")
        self.msg_labels[5].setText(f"BRAKE_PRESSED: {self.msg.brake_pressed}")
        self.msg_labels[6].setText(f"YAW_RATE: {self.msg.yaw_rate:.2f} rad/s")  # TODO: check units on this
        self.msg_labels[7].setText(f"LEFT_IND: {self.msg.left_indicator}")
        self.msg_labels[8].setText(f"RIGHT_IND: {self.msg.right_indicator}")
        


        # self.header_label.setText(f"Vel:{self.msg.velocity:.2f} m/s")

# TODO:

# callback for state info 
# UI formatting

import signal
# signal.signal(signal.SIGINT, signal.SIG_DFL)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    can_decoder_ui_node = CANDecoderUINode()

    # connect spin_once to a Qt Timer to allow Qt and ros to spin separately
    # https://github.com/ADVRHumanoids/rosee_gui/issues/1
    timer = QTimer()
    timer.timeout.connect(can_decoder_ui_node.run_once)
    timer.start(500) #ms

    # app.exec_()
    sys.exit(app.exec_())




if __name__ == "__main__":

    main()