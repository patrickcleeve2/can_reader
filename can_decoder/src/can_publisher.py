#!/usr/bin/env python3

import time
from pprint import pprint

import cantools
import rospy
import serial
from can_msgs.msg import Frame
from can_decoder.msg import CarState

KPH_TO_MS = 1. / 3.6
# ref
# https://github.com/commaai/openpilot/blob/master/selfdrive/car/hyundai/carstate.py

class CANDecoder(object):
    def __init__(self, dbc_file) -> None:
        super().__init__()

        self.db = cantools.database.load_file(
            dbc_file,
            strict=False,
        )

        self.can_data = {}
        self.total_msgs = 162

        self.can_sub = rospy.Subscriber("/mini/can", Frame, self.can_callback)
        self.state_pub = rospy.Publisher("mini/vehicle_state", CarState, queue_size=1)

    def can_callback(self, msg):

        message_id = msg.id
        message_data = list(msg.data)
        # rospy.loginfo(f"{msg.id}: {list(msg.data)}")
        
        message = self.db.decode_message(message_id, message_data)
        self.can_data.update(message)
        # pprint(self.can_data)
        
        if len(self.can_data) == self.total_msgs: # only publish once all msgs are received
            rospy.loginfo_once(f"starting to publish: {len(self.can_data)}/{self.total_msgs}")
            self.publish_state()
        else:
            rospy.loginfo_throttle(0.25, f"waiting on msgs: {len(self.can_data)}/{self.total_msgs}")

    def publish_state(self):

        # get car velocity from wheel speeds
        wheel_speed_fl = self.can_data["WHL_SPD_FL"] *KPH_TO_MS
        wheel_speed_fr = self.can_data["WHL_SPD_FR"] *KPH_TO_MS
        wheel_speed_ll = self.can_data["WHL_SPD_RL"] *KPH_TO_MS
        wheel_speed_rr = self.can_data["WHL_SPD_RR"] *KPH_TO_MS
        v_ego = (wheel_speed_fl + wheel_speed_fr + wheel_speed_ll + wheel_speed_rr) / 4.0
    

        msg = CarState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "car"
        msg.throttle_position = self.can_data["Accel_Pedal_Pos"] # / 254.
        msg.velocity = v_ego
        msg.steering_angle = self.can_data["SAS_Angle"]
        msg.steering_rate = self.can_data["SAS_Speed"]
        msg.brake_pressed = self.can_data["DriverBraking"]  # nb this is not brake position, just a flag

        msg.yaw_rate = self.can_data["YAW_RATE"]
        msg.left_blinker = self.can_data["CF_Gway_TurnSigLh"]
        msg.right_blinker = self.can_data["CF_Gway_TurnSigRh"]
        
        self.state_pub.publish(msg)

# TODO: door open, seatbelt, acc, lka, steer torque

if __name__ == "__main__":

    rospy.init_node("can_decoder")
    rospy.loginfo("hello can decoder")

    # dbc_file = "/home/patrick/self_drive/repos/can_demo/opendbc/hyundai_i30_2014.dbc"
    dbc_file = "/home/patrick/self_drive/repos/can_demo/opendbc/hyundai_kia_generic.dbc"
    can_decoder = CANDecoder(dbc_file=dbc_file)
    
    rospy.spin()


# NB:
# For custom msgs, the package name cannot be the same as the file name...what
# https://answers.ros.org/question/105711/rospy-custom-message-importerror-no-module-named-msg/