#!/usr/bin/env python3

import time
from pprint import pprint

import cantools
import rospy
import serial
import yaml
from can_decoder.msg import CarState
from can_msgs.msg import Frame

KPH_TO_MS = 1.0 / 3.6
# ref
# https://github.com/commaai/openpilot/blob/master/selfdrive/car/hyundai/carstate.py


class CANDecoder(object):
    def __init__(self, dbc_file, config) -> None:
        super().__init__()

        self.db = cantools.database.load_file(
            dbc_file,
            strict=False,
        )

        self.config = self.load_config(config)

        self.can_data = {}
        self.total_msgs = self.config["total_messages"]

        self.can_sub = rospy.Subscriber("/mini/can", Frame, self.can_callback)
        self.state_pub = rospy.Publisher("mini/vehicle_state", CarState, queue_size=1)

    def load_config(self, config):

        with open(config, "r") as f:

            config_dict = yaml.safe_load(f)
            rospy.loginfo(f"CONFIG LOADED: {config}")

        return config_dict

    def can_callback(self, msg):

        message_id = msg.id
        message_data = list(msg.data)
        # rospy.loginfo(f"{msg.id}: {list(msg.data)}")

        message = self.db.decode_message(message_id, message_data)
        self.can_data.update(message)
        # pprint(self.can_data)

        if (
            len(self.can_data) == self.total_msgs
        ):  # only publish once all msgs are received
            rospy.loginfo_once(
                f"starting to publish: {len(self.can_data)}/{self.total_msgs}"
            )
            self.publish_state()
        else:
            rospy.loginfo_throttle(
                0.25, f"waiting on msgs: {len(self.can_data)}/{self.total_msgs}"
            )

    def publish_state(self):

        # get car velocity from wheel speeds
        wheel_speed_fl = self.can_data[self.config["wheel_speed"]["FL"]] * KPH_TO_MS
        wheel_speed_fr = self.can_data[self.config["wheel_speed"]["FR"]] * KPH_TO_MS
        wheel_speed_ll = self.can_data[self.config["wheel_speed"]["RL"]] * KPH_TO_MS
        wheel_speed_rr = self.can_data[self.config["wheel_speed"]["RR"]] * KPH_TO_MS
        v_ego = (
            wheel_speed_fl + wheel_speed_fr + wheel_speed_ll + wheel_speed_rr
        ) / 4.0

        msg = CarState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "car"
        msg.throttle = self.can_data[self.config["control"]["throttle"]]  # / 254.
        msg.velocity = v_ego
        msg.steer = self.can_data[self.config["control"]["steer"]]
        msg.steer_rate = self.can_data[self.config["control"]["steer_rate"]]
        msg.brake_pressed = self.can_data[
            self.config["control"]["brake_pressed"]
        ]  # nb this is not brake position, just a flag

        msg.yaw_rate = self.can_data[self.config["control"]["yaw_rate"]]
        msg.left_indicator = self.can_data[self.config["indicator"]["left"]]
        msg.right_indicator = self.can_data[self.config["indicator"]["right"]]

        self.state_pub.publish(msg)


# TODO: door open, seatbelt, acc, lka, steer torque

if __name__ == "__main__":

    rospy.init_node("can_decoder")
    rospy.loginfo("hello can decoder")

    # TODO: move these to a relative path based on $(find...)
    # https://answers.ros.org/question/235337/unable-to-read-a-file-while-using-relative-path/

    # dbc_file = "/home/patrick/self_drive/repos/can_demo/opendbc/hyundai_i30_2014.dbc"
    dbc_file = "/home/patrick/self_drive/repos/can_demo/opendbc/hyundai_kia_generic.dbc"
    config_file = "/home/patrick/self_drive/catkin_ws/src/can_reader/can_decoder/config/hyundai_config.yaml"
    can_decoder = CANDecoder(dbc_file=dbc_file, config=config_file)

    rospy.spin()


# NB:
# For custom msgs, the package name cannot be the same as the file name...what
# https://answers.ros.org/question/105711/rospy-custom-message-importerror-no-module-named-msg/
