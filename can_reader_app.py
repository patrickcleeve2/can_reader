#!/usr/bin/env python3

import glob
import time
from pprint import pprint

import cantools
import serial
import streamlit as st

import pandas as pd

st.header("CAN Reader App")

# read dbc file
dbc_filenames = glob.glob("../opendbc/hyundai*.dbc")
db_file = st.selectbox("Select a DBC File ", dbc_filenames)
db = cantools.database.load_file(
    db_file,
    strict=False,
)

df = pd.DataFrame(columns=["frame_id", "byte_0", "byte_1", "byte_2", "byte_3", 
                        "byte_4", "byte_5", "byte_6", "byte_7"])

# setup serial connection
ser = serial.Serial("/dev/cu.usbmodem144301")
ser.flushInput()

SINGLE_FRAME = st.sidebar.select_slider("Data Filtering", [False, True])


if SINGLE_FRAME:
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

def decode_single_frame():
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
        # else:
        #     can_msg.write("No matching frames.")

def color_background(val):
    color = 'limegreen' if val<75 else 'cornflowerblue' if val <=150 else "gold" if val<=200 else 'lightcoral'
    return f'background-color: {color}'
# https://discuss.streamlit.io/t/change-background-color-based-on-value/2614/6
# https://www.w3schools.com/cssref/css_colors.asp



def visualise_all_frames(df, can_frame):
    if len(can_frame) == 10:
        can_frame = can_frame[1:] # strip out header
        # figure out a non-dumb way to do this
        can_dict = {}
        for idx, col in enumerate(df.columns):
            if idx != 0:
                can_dict[col] = int(can_frame[idx], base=16) # to decimal
            else:
                can_dict[col] = can_frame[idx]

        # check if frame_id is already in df and replace
        if can_frame[0] in df["frame_id"].values:

            df.loc[df["frame_id"] == can_frame[0]] = pd.DataFrame.from_records([can_dict])

        else:
            df = df.append(pd.DataFrame.from_records([can_dict]))
    return df

while read_can_data:

    try:
        # unpack CAN frame as string
        ser_bytes = ser.readline()
        decoded_bytes = ser_bytes[0 : len(ser_bytes) - 3].decode("utf-8")
        can_frame = [my_str for my_str in decoded_bytes.split(",")]
        
        if SINGLE_FRAME:
            decode_single_frame()
        else:
            df = visualise_all_frames(df, can_frame)            

            # what 
            can_msg.write(df.reset_index(drop=True).\
                sort_values("frame_id").\
                    style.applymap(color_background,    subset=df.columns[1:]))
    

        time.sleep(0.5)

    except KeyboardInterrupt:
        # print("Keyboard Interrupt")
        break


# TODO:
# - cleanup

# DONE:
# - read can message over serial
# - display message in streamlit
# - select dbc file and can frame to view
# - filter down to individual signal
# - display raw data as a table
# - change colour of values