#!/usr/bin/env python3

### Simple CAN Reader Demo
# Read and Decode CAN Frames send over Arduino serial connection

from pprint import pprint

# import cantools
import serial


if __name__ == "__main__":

    # db = cantools.database.load_file(
    #     "opendbc/hyundai_kia_generic.dbc",
    #     strict=False,
    # )

    ser = serial.Serial("/dev/ttyACM0")
    ser.flushInput()

    while True:
        try:
            ser_bytes = ser.readline()

            decoded_bytes = ser_bytes.decode("utf-8", errors="ignore").split(",")

            if len(decoded_bytes) == 11:  # only parse full messages

                frame = str(decoded_bytes[0])
                message_id = int(decoded_bytes[1], base=16)
                message_data = [int(num, base=16) for num in decoded_bytes[2:-1]]

                print(message_id, message_data)
                # message = db.decode_message(message_id, message_data)
                # pprint(message)

        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            break
