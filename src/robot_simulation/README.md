# 用于机械臂仿真，在gazebo和rviz中进行

本包提供底盘仿真支持，包含 RViz 可视化和 Gazebo 模型预览。

可用启动文件：

- `roslaunch robot_simulation car_simulation_rviz.launch`
- `roslaunch robot_simulation car_simulation_gazebo.launch`

仿真节点 `car_speed` 订阅 `/wheel_speed_cmd`，并发布 `/odom`、`/joint_states` 以及 TF `odom -> base_link`。

在 `car_ctl` 包中运行底盘控制节点时，使用参数 `_sim_mode:=true` 可让节点在仿真中发送速度命令。
