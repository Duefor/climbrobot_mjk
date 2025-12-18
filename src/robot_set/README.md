# 关于机械臂
> 在新端系统安装时需要修改cmakelist中的include_directories中关于robotsdk的位置

## 25/10/22
pose_error_controller.cpp文件用于pid控制,pid参数还没调试完美,现在的参数只是临时用
    后续调的时候可以尝试让v_d=0
main.cpp文件用于机械臂sdk控制,后续可以限制一下tcp速度,以防万一
    调试的时候注意随时停止main文件,以免发生意外
pub_posANDvel.cpp文件用于发布测试代码,绘制一个三角形

先运行robot_main文件,等到机械臂运行到初始位姿时候,再同时运行pose_error_controller和pub_posANDvel,或者是main.launch
## 25/12/18
感觉机械臂的自动规划很奇怪，之后遥操作设置零点的时候还是用关节，不用tcp了
给机械臂的移动加限制，限制关节角转动速度和tcp速度