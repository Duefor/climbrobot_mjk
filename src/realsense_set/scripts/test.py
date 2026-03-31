#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import sys
sys.path.append('/usr/lib/python2.7/dist-packages')

import pyrealsense2 as rs

def callback(frame):
    if not frame.is_motion_frame():
        return

    m = frame.as_motion_frame()
    data = m.get_motion_data()
    st = m.get_profile().stream_type()

    if st == rs.stream.gyro:
        print("GYRO  :", data.x, data.y, data.z)
    elif st == rs.stream.accel:
        print("ACCEL :", data.x, data.y, data.z)

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.accel)

pipeline.start(config, callback)

print("IMU started...")
try:
    while True:
        pass
except KeyboardInterrupt:
    pipeline.stop()