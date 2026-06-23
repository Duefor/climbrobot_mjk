# CS66 Robot Controller - Servoj Control

基于 `writeServoj()` 的高精度实时机器人控制实现，提供平滑、精确的运动控制能力。

## 主要特性

### 🚀 实时控制
- **高精度时序**：4ms精确控制周期（250Hz）
- **实时线程**：使用Linux实时调度策略（SCHED_FIFO）
- **CPU亲和性**：绑定到专用CPU核心，避免上下文切换
- **内存锁定**：防止内存换出，确保低延迟

### 🎯 平滑运动
- **缓动插值**：使用三次贝塞尔曲线进行平滑插值
- **自适应控制**：根据运动距离自动调整控制参数
- **轨迹优化**：支持复杂轨迹的平滑执行

### 🔧 灵活控制
- **双模式支持**：实时模式和非实时模式
- **多空间控制**：关节空间和笛卡尔空间控制
- **队列模式**：支持轨迹点队列执行

## 架构设计

### 实时控制架构
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS Topics    │    │  Realtime Thread │    │   writeServoj   │
│                 │───▶│                  │───▶│                 │
│ - joint_command │    │ - 250Hz cycle    │    │ - 4ms precision │
│ - tcp_command   │    │ - FIFO scheduling│    │ - Smooth interp │
│ - emergency_stop│    │ - CPU affinity   │    │ - Queue mode    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 控制流程
1. **话题接收**：ROS话题接收运动命令
2. **参数设置**：配置运动参数（时间、平滑度等）
3. **实时执行**：实时线程执行平滑插值
4. **精确控制**：4ms周期发送servoj命令

## 使用方法

### 基本配置

```yaml
# config/servoj_control.yaml
enable_realtime_control: true  # 启用实时模式
servoj_frequency: 250.0        # 250Hz控制频率
realtime_cpu_core: 2           # 绑定CPU核心2
```

### 关节空间控制

```cpp
// 平滑关节运动
std::vector<double> target_joints = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
controller->moveToJointPositions(target_joints, 2.0, true);  // 2秒平滑运动

// 快速关节运动
controller->moveToJointPositions(target_joints, 0.1, false); // 100ms快速运动
```

### 笛卡尔空间控制

```cpp
// 平滑TCP运动
std::vector<double> target_pose = {0.5, 0.3, 0.4, 0.0, 0.0, 0.0};
controller->moveToTCPPose(target_pose, 2.0, true);  // 2秒平滑运动

// 快速TCP运动
controller->moveToTCPPose(target_pose, 0.1, false); // 100ms快速运动
```

### 轨迹执行

```cpp
// 复杂轨迹执行
std::vector<std::vector<double>> trajectory = {
    {0.1, 0.2, 0.3, 0.0, 0.0, 0.0},
    {0.2, 0.3, 0.4, 0.0, 0.0, 0.1},
    {0.3, 0.4, 0.5, 0.0, 0.0, 0.2}
};
controller->executeTrajectory(trajectory, 3.0, true); // 3秒轨迹，笛卡尔空间
```

## 性能优化

### 实时线程设置
```cpp
// 自动设置实时线程属性
void setupRealtimeThread() {
    mlockall(MCL_CURRENT | MCL_FUTURE);  // 锁定内存
    RT_UTILS::setThreadFiFoScheduling(handle, max_priority);  // FIFO调度
    RT_UTILS::bindThreadToCpus(handle, cpu_core);  // CPU亲和性
}
```

### 控制循环优化
```cpp
// 精确时序控制
auto next_cycle = std::chrono::steady_clock::now();
const auto cycle_time = std::chrono::microseconds(4000); // 4ms

while (control_running) {
    // 执行控制逻辑
    executeControl();
    
    // 精确等待下一个周期
    next_cycle += cycle_time;
    std::this_thread::sleep_until(next_cycle);
}
```

## 插值算法

### 缓动函数
```cpp
double easeInOutCubic(double t) {
    if (t < 0.5) {
        return 4 * t * t * t;  // 加速阶段
    } else {
        double p = 2 * t - 2;
        return 1 + p * p * p / 2;  // 减速阶段
    }
}
```

### 位置插值
```cpp
vector6d_t interpolatePosition(const vector6d_t& start, 
                              const vector6d_t& target, 
                              double t) {
    vector6d_t result;
    for (int i = 0; i < 6; ++i) {
        result[i] = start[i] + t * (target[i] - start[i]);
    }
    return result;
}
```

## 系统要求

### 硬件要求
- **CPU**：支持多核心的x86_64处理器
- **内存**：至少2GB RAM
- **网络**：千兆以太网连接

### 软件要求
- **操作系统**：Linux（推荐Ubuntu 18.04+）
- **内核**：支持实时调度的Linux内核
- **权限**：需要root权限运行实时线程

### 系统配置
```bash
# 设置实时调度权限
echo "@realtime soft rtprio 99" >> /etc/security/limits.conf
echo "@realtime hard rtprio 99" >> /etc/security/limits.conf

# 配置CPU隔离（可选）
# 在GRUB中添加：isolcpus=2 nohz_full=2 rcu_nocbs=2
```

## 性能指标

### 控制精度
- **时序精度**：±0.1ms
- **位置精度**：±0.001 rad
- **控制延迟**：<1ms

### 运动平滑度
- **速度连续性**：C²连续
- **加速度限制**：可配置
- **轨迹偏差**：<0.1mm

## 故障排除

### 常见问题

1. **实时线程创建失败**
   ```bash
   # 检查权限
   ulimit -a | grep rtprio
   
   # 解决方案：以root权限运行或配置用户权限
   ```

2. **控制延迟过大**
   ```bash
   # 检查系统负载
   top -p $(pgrep robot_controller)
   
   # 解决方案：关闭不必要的服务，使用CPU隔离
   ```

3. **内存锁定失败**
   ```bash
   # 检查内存限制
   cat /proc/sys/vm/swappiness
   
   # 解决方案：减少swap使用，增加物理内存
   ```

## 示例程序

运行示例程序：
```bash
# 编译
catkin_make

# 运行示例
rosrun cs66_ctl_manipulation servoj_control_example

# 使用自定义配置
rosrun cs66_ctl_manipulation servoj_control_example _config_file:=config/servoj_control.yaml
```

## 更新日志

### v2.0.0 - Servoj Control
- ✅ 完全基于 `writeServoj()` 实现
- ✅ 集成实时线程技术
- ✅ 添加平滑插值算法
- ✅ 支持双模式控制（实时/非实时）
- ✅ 优化控制精度和性能
- ❌ 移除过时的轨迹控制接口

### v1.0.0 - Legacy
- 基于 `writeTrajectoryControlAction()` 实现
- 使用ROS定时器控制
- 基础运动控制功能





