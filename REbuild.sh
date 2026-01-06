#!/bin/bash

# 删除旧的 build 目录
rm -rf build/
rm -rf devel/

# 重新构建
catkin_make
