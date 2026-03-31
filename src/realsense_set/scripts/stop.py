#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('/usr/lib/python2.7/dist-packages')
import pyrealsense2 as rs
import time

ctx = rs.context()
for dev in ctx.query_devices():
    print("Resetting:", dev.get_info(rs.camera_info.name))
    dev.hardware_reset()

time.sleep(2)