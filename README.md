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
~~c_cpp_properties文件改路径名~~    
~~robot_set包的cmakelist中改include_directories的路径~~     
注释掉touch_set包中cmakelist的catkin_package    
~~进入root操作~~    
放开robot_set包的cmakelist中的add_dependencies，保证自定义msg可以被正常编译    
编译时使用catkin_make_pkg.sh
## 26/1/7
手柄第六轴旋转的时候注意不要转到另一侧去了，否则机械臂会翻转一圈，怀疑是ik节点的问题
## 26/1/8
新想法：虽然是通过tcp遥操作的，但是可以加入关节角做辅助，让机械臂看起来和手柄更相似，否则就容易出现同一tcp位姿但是机械臂转到另一侧去了，特别是手柄大角度转动的时候
## 26/1/10
机械臂系统升级后无法使用老版sdk了，只能用新版
## 26/1/23
香橙派上无法使用轨迹跟踪，改为servoj
连接机械臂逻辑更改
如何使得在不将机械臂移动回零点的时候也能直接连接手柄操作
## 26/1/28
在进入遥操作初始阶段，如果机械臂未在零点（手柄和机械臂之间有较大误差），那么会导致机械臂猛的去跟手柄，随后速度降不下来，经查询发现是因为关节角加速度限制给的太高，机械臂无法立刻停下来。故将关节角加速度限制放开。同时设置好机械臂权重跟随，让机械臂慢慢跟手柄到重合后才恢复权重。但这样做会导致机械臂抖动严重。下一步消除抖动。

## 26/3/4
~~无法编译touch包，编译出现错误：宏展开失效或是宏未定义~~     
~~考虑如下：在顶层cmakelist中 取消显示指定c++版本 / 显式指定c++版本为14 / 显示指定c++版本为17~~
由于openhaptics包需要低版本的c++14；而cs66机械臂的sdk需要高版本的c++17。故在全局cmakelist中不能定义整体c++版本，而需要在使用cs66机械臂的sdk的功能包中的cmakelist中设置 `add_compile_options(-std=c++17)`

## 26/4/10
改用catkin build编译，首先需要安装catkin-tools工具：`sudo apt install python3-catkin-tools`

---


### 一体化节点 `robot_main`（推荐用于新部署）

将手柄订阅、工作空间映射、笛卡尔 PID+前馈、**MoveIt 雅可比阻尼最小二乘速度 IK**、SDK 状态发布与关节速度执行合并为单进程，减少话题延迟与多节点参数重复配置。

启动前需在参数服务器上加载 `robot_description`（与 `cs66_moveit_config` 中 SRDF 一致）。示例：

```bash
roslaunch robot_set robot_main.launch
```

在此之前需运行手柄驱动节点（例如 `touch_set`），使其按 `robot_set/TCPState` 发布位姿与线速度（默认订阅话题：`/phantom/pose`）。

**主要私有参数（`~` 命名空间）**

| 参数名 | 含义 |
|--------|------|
| `haptic_tcp_topic` | 手柄 TCP 话题（`TCPState`） |
| `joint_states_pub_topic` / `tcp_state_pub_topic` / `force_pub_topic` | 机械臂关节、TCP、力传感器发布话题 |
| `robot_sdk_rate` / `robot_pub_rate` | SDK 控制周期与状态发布频率（Hz） |
| `cmd_timeout` | 有效关节速度指令超时（秒），超时后自动下发零速 |
| `desired_timeout` | 手柄期望位姿超时（秒），超时后复位积分与 IK 平滑状态 |
| `planning_group` / `tip_link` | MoveIt 运动组与末端链节名（默认 `planning_group` / `ee_link`） |
| `ik_svd_threshold` / `ik_damping_gain` | 奇异区域阻尼 IK 阈值与增益 |
| `MAX_QDOT_SCALE_1` … `6` | 相对 SDK 最大关节速度的执行比例（与 `ik_velocity_solver` 含义一致） |
| `MAX_QDDOT_1` … `6` | IK 输出关节加速度限幅 |
| `H_INIT_*` / `H_MAX_*` / `H_MIN_*` / `R_INIT_*` / `R_MAX_*` / `R_MIN_*` / `CONTROL_MODE` | 手柄到机械臂工作空间映射（同原 `pose_converter`） |
| `KP_LIN_N` / `KI_LIN_N` / `KD_LIN_N` / `KP_ROT_N` / `KVIR_ROT_N` / `EQUAL1` / `EQUAL2` / `E_FF_ON` / `integral_limit` / `beta_tau` | 笛卡尔跟踪与前馈（同原 `pose_error_controller`） |

**发布内容**：关节位置（`sensor_msgs/JointState`）；TCP 位姿与由数值微分得到的线速度、角速度（`TCPState.velocity` 前 3 为 m/s，后 3 为 rad/s）；六维力（`std_msgs/Float64MultiArray`，与 SDK 一致）。