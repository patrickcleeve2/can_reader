#!/usr/bin/env python3

from pprint import pprint

import cantools
import serial

if __name__ == "__main__":

    # rospy.init_node("can_reader")
    # rospy.loginfo("hello can reader")

    db = cantools.database.load_file(
        "opendbc/hyundai_kia_generic.dbc",
        strict=False,
    )

    ser = serial.Serial("/dev/ttyACM0")
    ser.flushInput()

    # can_pub = rospy.Publisher("mini/can", Frame, queue_size=1)

    while True:
        try:
            ser_bytes = ser.readline()

            decoded_bytes = ser_bytes.decode("utf-8", errors="ignore").split(",")

            if len(decoded_bytes) == 11:  # only parse full messages

                frame = str(decoded_bytes[0])
                message_id = int(decoded_bytes[1], base=16)
                message_data = [int(num, base=16) for num in decoded_bytes[2:-1]]

                message = db.decode_message(message_id, message_data)
                pprint(message)

                # # publish ros can frame
                # frame = Frame()
                # frame.header.stamp = rospy.Time.now()
                # frame.id = message_id
                # frame.dlc = len(message_data)
                # frame.data = bytes(message_data)

                # can_pub.publish(frame)

                # rospy.loginfo(f"{message_id}: {message_data}")

        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            break
