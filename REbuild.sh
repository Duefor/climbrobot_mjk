# 该文件可以在每次添加新文件后自动执行构建过程（包括删除 build 目录、重新运行 catkin_make）

#!/bin/bash

# 删除旧的 build 目录
rm -rf build/
rm -rf devel/

# 重新构建
catkin_make
