# 用于控制底盘的运动
## 26/1/13
也哥的连接can命令：  
//必要  
修改寄存器的值,使能CAN控制器  
sudo busybox devmem 0x0c303018 w 0xc458  
sudo busybox devmem 0x0c303010 w 0xc400  
sudo busybox devmem 0x0c303008 w 0xc458  
sudo busybox devmem 0x0c303000 w 0xc400  

挂载CAN内核  
sudo modprobe can  
sudo modprobe can_raw  
sudo modprobe mttcan  

验证内核是否加载  
lsmod | grep can  

查看接口  
ip link show  

设置接口波特率  
sudo ip link set can0 type can bitrate 500000  

启动 can0  
sudo ip link set up can0  
**写了一个自动化脚本，可以直接开**  

## 26/4/7
cmakelist中的socketcan包可能要改一下链接名，arm64和x86架构链接的包名不一致

## 26/4/15
新增统一底盘动作服务器：
- 新增 action 定义 `action/CarWheelMotion.action`
- 新增动作服务器节点 `src/car_main_1.cpp`
- 新增动作客户端测试程序 `src/car_wheel_action_client.cpp`
- 修复 CMake `src/CMakeLists.txt` 中 `test.cpp` 目标名冲突，避免与 CTest 保留目标冲突

### 新增功能说明
- 速度模式（`control_mode=0`）
  - 直接持续接收当前目标左右轮速度
  - 持续向 CAN IDs `0x601`/`0x602` 发送速度命令
  - 目标持续执行直到 action 被取消或收到新 goal
- 位置模式（`control_mode=1`）
  - 参考 `car_move_pos_test.cpp` 实现位置控制命令序列
  - 发送位置模式初始化、目标位置、相对位移和启动运动命令

### 使用方法
1. 构建 package：
   ```bash
   cd /home/duefor/climbrobot_mjk
   catkin build car_ctl
   ```
2. 启动动作服务器：
   ```bash
   rosrun car_ctl car_main_1
   ```
3. 速度模式测试：
   ```bash
   rosrun car_ctl car_wheel_action_client 0 1500 1500
   ```
4. 位置模式测试：
   ```bash
   rosrun car_ctl car_wheel_action_client 1 -0.25 -0.25 10.0
   ```

### 新文件列表
- `action/CarWheelMotion.action`
- `src/car_main_1.cpp`
- `src/car_wheel_action_client.cpp`
