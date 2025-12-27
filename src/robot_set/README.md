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
## 25/12/19
需要注意一个事情，robot_main本质是调用tcp速度的，ik靠机械臂内部程序实现，在tcp速度处限幅，关节速度也可能超额，特别是在奇异点附近容易出问题，
最好的方法是调用joint速度，也就是将tcp速度进行ik得到joint速度，robot_main_1，但是速度ik有点问题，需要解决一下
但是如果使用joint速度控制的话有另一个问题，就是在对joint关节角速度进行安全限制的话，速度ik得到的joint角速度就会扭曲，可能实际tcp速度与期望不一样
### **重大更新**
该更新针对ik_velocity_solver.cpp文件
其订阅的期望tcp速度为vc = [ vx, vy, vz, rx, ry, rz ]，[vx, vy, vz]TCP 线速度（m/s），在基坐标系表达；[rx, ry, rz]旋转向量（rotation vector）的时间导数（rad/s）
与cs66机械臂sdk定义一致
## 25/12/24
(**引用部分弃用**)
> cs66机械臂的初始位姿由如下公式得到：
> $${O}_{\mathrm{Ui}}=U_{i\mathrm{min}}+\frac{\left|P_{Oi}-P_{i\mathrm{min}}\right|}{P_{i\mathrm{max}}-P_{i\mathrm{min}}}*(U_{i\mathrm{max}}-U_{i\mathrm{min}})$$
> 其中，i表示x,y,z三个方向之一，$O_{ui}$为cs66机器人设定的初始位置i某一方向上的值，$U_{imin}$为cs66机器人实际范围的最小值，$U_{imax}$为cs66机器人实际范围的最大值，$P_{imax}$与Pimin为Geomagic Touch设备范围最大与最小值，$P_{oi}$为Geomagic Touch初始位置i方向上的值。
---
touch的活动范围为：x[-0.21,0.21] y[-0.08,0.07] z[-0.11,0.18]
cs66的活动范围为：x[-0.8,0.8] y[-0.8,0.8] z[-0.8,0.8]

---
注意示教器控制的是更底层的数据，如果示教器中的速度百分比没有设置成100%，则远程控制中的速度也会等比例变化

---
机械臂的末端执行器正面朝下时需要(rx,ry,rz) = (3.14,0,0)
机械臂基座坐标系以孔的方向为x轴，与touch的xy方向是相反的。
机械臂初始位置设为(0.28,-0.15,0.48)
其工作空间设置为
x(0.28,0.63) y(-0.63,0.26) z(0.18,0.65)
## 25/12/25
将角速度限制设置在官方手册中能达到的最大值的1/20，这是实地操作得到的最安全数值