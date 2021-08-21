#!/usr/bin/env python3


import time
from pprint import pprint

import cantools
import serial

db = cantools.database.load_file(
    "../opendbc/hyundai_i30_2014.dbc",
    strict=False,
)


# pprint(db.messages)
example_message = db.get_message_by_name("CLU2")
# pprint(example_message.signals)

# for msg in db.messages:
#     print(msg.name)
#     example_message = db.get_message_by_name(msg.name)
#     pprint(example_message.signals)



ser = serial.Serial("/dev/cu.usbmodem144301")
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        decoded_bytes = ser_bytes[0 : len(ser_bytes) - 3].decode("utf-8")
        bytes_list = [my_str for my_str in decoded_bytes.split(",")]

        # # decode can frame
        if len(bytes_list) > 5:
            message_id = int(bytes_list[1], base=16)
            message_data = [int(num, base=16) for num in bytes_list[2:]]
            message = db.decode_message(message_id, message_data)
            pprint(message)
            print("-"*50)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        break
