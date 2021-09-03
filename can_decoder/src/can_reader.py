#!/usr/bin/env python3

import rospy
import serial
from can_msgs.msg import Frame

if __name__ == "__main__":

    rospy.init_node("can_reader")
    rospy.loginfo("hello can reader")

    ser = serial.Serial("/dev/ttyACM0")
    ser.flushInput()

    can_pub = rospy.Publisher("mini/can", Frame, queue_size=1)

    while not rospy.is_shutdown():
        try:
            ser_bytes = ser.readline()
            decoded_bytes = ser_bytes[0 : len(ser_bytes) - 3].decode("utf-8")
            bytes_list = [my_str for my_str in decoded_bytes.split(",")]

            # # decode can frame
            if len(bytes_list) == 10:
                message_id = int(bytes_list[1], base=16)
                message_data = [int(num, base=16) for num in bytes_list[2:]]

                # publish ros can frame
                frame = Frame()
                frame.header.stamp = rospy.Time.now()
                frame.id = message_id
                frame.dlc = len(message_data)
                frame.data = bytes(message_data)

                can_pub.publish(frame)

                rospy.loginfo(f"{message_id}: {message_data}")

        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            break
