#!/usr/bin/env python3

import glob
import time
from pprint import pprint

import cantools
import serial
import streamlit as st

st.header("CAN Reader App")

# read dbc file
dbc_filenames = glob.glob("../opendbc/hyundai*.dbc")
db_file = st.selectbox("Select a DBC File ", dbc_filenames)
db = cantools.database.load_file(
    db_file,
    strict=False,
)

# setup serial connection
ser = serial.Serial("/dev/cu.usbmodem144301")
ser.flushInput()

# select data
frame_id = st.selectbox(
    "Select a CAN Frame ID: ", [hex(msg.frame_id) for msg in db.messages]
)

selected_message = db.get_message_by_frame_id(int(frame_id, base=16))

selected_signal = st.selectbox(
    "Select a signal: ", [None] + [signal.name for signal in selected_message.signals]
)

read_can_data = st.button("Read CAN Data")
st.subheader("Current CAN Frame")
can_msg = st.empty()

# read 'CAN' data from serial connection
while read_can_data:

    try:
        # unpack CAN frame as string
        ser_bytes = ser.readline()
        decoded_bytes = ser_bytes[0 : len(ser_bytes) - 3].decode("utf-8")
        can_frame = [my_str for my_str in decoded_bytes.split(",")]

        # # decode CAN frame
        if len(can_frame) > 5:
            if can_frame[1] == frame_id:
                message_id = int(can_frame[1], base=16)
                message_data = [int(num, base=16) for num in can_frame[2:]]
                message = db.decode_message(message_id, message_data)

                if selected_signal:
                    can_msg.write(f"{selected_signal}: {message[selected_signal]}")
                else:
                    can_msg.write(message)
                # print(message_data)
                # print("-"*50)
            else:
                can_msg.write("No matching frames.")
            time.sleep(0.5)

    except KeyboardInterrupt:
        # print("Keyboard Interrupt")
        break


# TODO:
# - display raw data as a table
# - change colour of values

# DONE:
# - read can message over serial
# - display message in streamlit
# - select dbc file and can frame to view
# - filter down to individual signal