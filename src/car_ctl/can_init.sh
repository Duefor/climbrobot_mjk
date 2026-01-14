#!/bin/bash

# 配置 CAN 引脚
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
sudo busybox devmem 0x0c303008 w 0xc458
sudo busybox devmem 0x0c303000 w 0xc400

# 加载 CAN 内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# 配置并启动 can0
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0