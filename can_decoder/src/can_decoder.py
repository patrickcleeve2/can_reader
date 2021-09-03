#!/usr/bin/env python3

import time
from pprint import pprint

import cantools
import rospy
import serial
from can_msgs.msg import Frame


class CANDecoder(object):
    def __init__(self, dbc_file) -> None:
        super().__init__()

        self.db = cantools.database.load_file(
            dbc_file,
            strict=False,
        )

        self.can_sub = rospy.Subscriber("/mini/can", Frame, self.can_callback)

    def can_callback(self, msg):

        message_id = msg.id
        message_data = list(msg.data)
        # rospy.loginfo(f"{msg.id}: {list(msg.data)}")
        
        message = self.db.decode_message(message_id, message_data)
        rospy.loginfo(f"Message: {message_id}:")
        pprint(message)
        rospy.loginfo("-"*50)

# TODO: add a custom message for vehicle status


if __name__ == "__main__":

    rospy.init_node("can_decoder")
    rospy.loginfo("hello can decoder")

    can_decoder = CANDecoder(dbc_file="/home/jetson03/self_drive/can_demo/opendbc/hyundai_i30_2014.dbc")

    rospy.spin()
