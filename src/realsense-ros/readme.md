该包为realsense官方ros包
----------------------------------------------------
25/11/20
启动realsense相机的时候，如果使用官方realsense-ros包的话，需要保证

duefor@DF:~/realsense2_ws$ dpkg -l | grep realsense
ii  librealsense2:amd64                             2.55.1-0~realsense.12473             amd64        Intel(R) RealSense(tm) Cross Platform API - runtime
ii  librealsense2-dbg:amd64                         2.55.1-0~realsense.12473             amd64        Intel(R) RealSense(tm) Camera Capture API - debug symbols
ii  librealsense2-dev:amd64                         2.55.1-0~realsense.12473             amd64        Intel(R) RealSense(tm) Camera Capture API - development files
ii  librealsense2-dkms                              1.3.27-0ubuntu1                      all          Modified kernel modules for librealsense2
ii  librealsense2-gl:amd64                          2.55.1-0~realsense.12473             amd64        Intel(R) RealSense(tm) - GLSL-enabled extensions
ii  librealsense2-udev-rules:amd64                  2.55.1-0~realsense.12473             amd64        Intel(R) RealSense(tm) Camera Capture API - udev rules
ii  librealsense2-utils:amd64                       2.55.1-0~realsense.12473             amd64        Intel(R) RealSense(tm) Camera Capture API - utils and demos

也就是需要保证librealsense的版本在2.55以上。不能用sudo命令安装，否则librealsense版本会在2.50
也就是说官方realsense-ros包需要源码安装，用sudo安装的话会自动安装2.50版本的librealsense并链接。

错误的安装方法可能导致realsense d435i 无法开启imu流
删除旧版本的librealsense包也会一并删除rtabmap，故rtabmap和rtabmap_ros也需要源码编译。
----------------------------------------------------