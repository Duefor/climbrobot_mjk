# 遥操作 touch + cs66robot
## 25/11/11
小车用的机械臂sdk是老版sdk，但调试用的sdk是新版的，新旧sdk安装会覆盖对方的include文件夹中的hpp文件（/usr/local/include/Elite），但不会覆盖动态库（/usr/local/lib）
新旧sdk有些许不一样：名称；部分函数。需要改对应的cmakelist文件及其他
## 25/12/19
将该包同步到gitee上，在没有梯子的情况下可以通过gitee拉取代码
## 25/12/27
启用遥操作机械臂：
1. 启动touch节点    
```rosrun touch_set touch_pub```    
发布话题：`/phantom/pose`   
> 该节点用于发布手柄的tcp位姿和tcp线速度

2. 启动映射节点  
```rosrun controller_set pose_converter```  
订阅话题：`/phantom/pose`  
发布话题：`/pose_converter/desired_robot_sub`  
> 该节点用于将touch的tcp位姿和tcp线速度映射为期望的机械臂tcp位姿和线速度  

3. 启动PID控制节点  
```rosrun robot_set pose_error_controller```  
订阅话题：`/cs66/tcp_state`  
订阅话题：`/pose_converter/desired_robot_sub`  
发布话题：`/controller/cartesian_vel`  
> 该节点用于修正期望机械臂位姿与实际机械臂位姿，发布最终tcp速度控制命令  

4. 启动速度ik节点  
```rosrun robot_set ik_velocity_solver```  
订阅话题：`/controller/cartesian_vel`  
订阅话题：`/cs66/joint_states`  
发布话题：`/velocity_ik/joint_vel`  
> 该节点用于将期望的机械臂tcp速度转为joint速度  

5. 启动机械臂节点   
```rosrun robot_set robot_main_m```  
首先机械臂会移动到零点位姿，随后  
发布话题：`/cs66/joint_states`  
发布话题：`/cs66/tcp_state`  
订阅话题：`/velocity_ik/joint_vel`  
> 该节点按照订阅话题的关节速度执行命令  
> **该节点强停可以强制停止机械臂**

## 26/1/6
在barry电脑中需要做的操作： 
c_cpp_properties文件改路径名    
robot_set包的cmakelist中改include_directories的路径 
注释掉touch_set包中cmakelist的catkin_package    
进入root操作    
编译时使用catkin_make_pkg.sh

## 26/1/7
手柄第六轴旋转的时候注意不要转到另一侧去了，否则机械臂会翻转一圈，怀疑是ik节点的问题