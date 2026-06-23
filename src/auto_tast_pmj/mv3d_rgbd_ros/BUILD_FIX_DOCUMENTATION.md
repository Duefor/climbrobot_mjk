# mv3d_rgbd_ros 编译修复文档

## 概述

本文档详细说明了为解决编译问题而对 `CMakeLists.txt` 所做的修改，包括新增的依赖包、修改的原因以及它们的作用。

## 修改日期

2024年（根据实际编译问题修复时间）

## 问题背景

在编译 `mv3d_rgbd_ros` 功能包时遇到了以下问题：

1. **empy 版本冲突**：anaconda 环境中的 empy 4.2 与 ROS noetic 的 genmsg 不兼容
2. **C++ 标准版本过旧**：使用 `-std=c++0x` 导致 PCL 1.10 编译错误
3. **libusb 链接错误**：PCL 库需要 libusb-1.0 但未正确链接

## 详细修改说明

### 1. 新增依赖包：PkgConfig

#### 修改位置
```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(USB REQUIRED libusb-1.0)
```

#### 为什么需要 PkgConfig？

**PkgConfig** 是一个用于管理编译和链接标志的工具，它通过 `.pc` 文件提供库的编译和链接信息。

**作用：**
- 自动查找系统已安装的库（如 libusb-1.0）
- 获取库的头文件路径（`USB_INCLUDE_DIRS`）
- 获取库的链接标志（`USB_LIBRARIES`）
- 获取库的链接目录（`USB_LIBRARY_DIRS`）
- 获取编译标志（`USB_CFLAGS_OTHER`）

**为什么选择 PkgConfig 而不是直接链接？**
- **标准化**：大多数 Linux 系统库都提供 `.pc` 文件，使用标准方式查找
- **跨平台兼容**：不同发行版可能将库安装在不同位置，PkgConfig 自动处理
- **依赖管理**：自动处理库的依赖关系
- **版本检查**：可以检查库的版本是否符合要求

**示例：**
```bash
# PkgConfig 会查找 /usr/lib/pkgconfig/libusb-1.0.pc 文件
# 该文件包含：
#   - 库路径：/usr/lib/x86_64-linux-gnu
#   - 头文件路径：/usr/include/libusb-1.0
#   - 链接标志：-lusb-1.0
```

### 2. 新增依赖包：libusb-1.0

#### 修改位置
```cmake
pkg_check_modules(USB REQUIRED libusb-1.0)
```

在所有可执行文件的链接中添加：
```cmake
target_link_libraries(... ${USB_LIBRARIES})
```

#### 为什么需要 libusb-1.0？

**libusb-1.0** 是一个用户空间的 USB 库，提供跨平台的 USB 设备访问接口。

**作用：**
- **PCL 库的依赖**：PCL (Point Cloud Library) 的 `libpcl_io.so` 模块需要 libusb-1.0 来支持 USB 设备的点云数据读取
- **USB 设备通信**：如果项目需要直接与 USB 设备（如 3D 相机、深度传感器）通信，libusb-1.0 提供底层支持
- **解决链接错误**：修复 `undefined reference to 'libusb_set_option'` 错误

**为什么会出现链接错误？**
- PCL 1.10 的 `libpcl_io.so` 在编译时链接了 libusb-1.0
- 但我们的可执行文件在链接时没有显式包含 libusb-1.0
- 导致链接器找不到 `libusb_set_option` 等符号

**解决方案：**
通过 `pkg_check_modules(USB REQUIRED libusb-1.0)` 获取：
- `USB_LIBRARIES`：包含 `-lusb-1.0` 链接标志
- `USB_INCLUDE_DIRS`：libusb 头文件路径（如果需要直接使用 libusb API）

然后在所有可执行文件中添加 `${USB_LIBRARIES}` 到链接列表。

### 3. C++ 标准版本升级

#### 修改前
```cmake
ADD_DEFINITIONS("-std=c++0x -O3")
```

#### 修改后
```cmake
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
ADD_DEFINITIONS("-O3")
```

#### 为什么需要升级到 C++14？

**原因：**
1. **PCL 1.10 要求**：PCL 1.10 使用了 C++14 特性（如 lambda 表达式的类型推断增强）
2. **编译器兼容性**：`-std=c++0x` 是 C++11 的旧写法，现代编译器推荐使用标准版本号
3. **代码兼容性**：避免 PCL 库中的 C++14 特性导致编译错误

**具体错误示例：**
```
error: no match for call to '(pcl::getFieldIndex(...)::<lambda(int)>) (const pcl::PCLPointField&)'
```
这是因为 PCL 使用了 C++14 的 lambda 类型推断，需要 C++14 标准。

**修改说明：**
- `CMAKE_CXX_STANDARD 14`：设置 C++ 标准为 C++14
- `CMAKE_CXX_STANDARD_REQUIRED ON`：强制要求编译器支持 C++14，不支持则报错
- 移除了 `-std=c++0x`，改用 CMake 的标准方式设置

## 完整的修改清单

### CMakeLists.txt 修改

1. **第 22-23 行**：添加 PkgConfig 和 libusb-1.0 查找
   ```cmake
   find_package(PkgConfig REQUIRED)
   pkg_check_modules(USB REQUIRED libusb-1.0)
   ```

2. **第 42-43 行**：升级 C++ 标准
   ```cmake
   set(CMAKE_CXX_STANDARD 14)
   set(CMAKE_CXX_STANDARD_REQUIRED ON)
   ```

3. **第 90, 93, 97, 101, 105, 109 行**：在所有可执行文件中添加 USB 库链接
   ```cmake
   target_link_libraries(... ${USB_LIBRARIES})
   ```

## 依赖关系图

```
mv3d_rgbd_ros
├── PCL (Point Cloud Library)
│   └── libpcl_io.so
│       └── 需要 libusb-1.0 ← 这就是为什么需要添加 libusb-1.0
├── OpenCV
├── ROS packages (roscpp, sensor_msgs, etc.)
└── Mv3dRgbdSDK (第三方 SDK)
```

## 编译说明

### 正常编译命令

由于 empy 版本冲突问题，编译时需要设置 PYTHONPATH：

```bash
cd /home/barry/workspace/ws_moveit
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH catkin_make
```

### 为什么需要设置 PYTHONPATH？

- anaconda 环境中的 empy 4.2 与 ROS noetic 的 genmsg 不兼容
- 通过设置 PYTHONPATH，优先使用系统的 empy 3.3.2
- 这确保了 ROS 消息生成工具能正常工作

### 永久解决方案（可选）

如果需要永久解决 empy 问题，可以在 `~/.bashrc` 中添加：

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
```

或者卸载 anaconda 的 empy 包（如果不需要）：

```bash
conda remove empy
```

## 验证修改

编译成功后，应该能看到所有可执行文件：

```bash
ls devel/lib/mv3d_rgbd_ros/
# 应该看到：
# - hik_camera_image_pub
# - hik_camera_image_sub
# - hik_camera_image_client
# - hik_camera_image_server
# - hik_camera_point_cloud_client
# - hik_camera_point_cloud_server
```

## 常见问题

### Q1: 如果系统没有安装 libusb-1.0 开发包怎么办？

**A:** 安装 libusb-1.0 开发包：
```bash
sudo apt-get install libusb-1.0-0-dev
```

### Q2: 为什么使用 PkgConfig 而不是 find_library？

**A:** 
- PkgConfig 是更标准的方式，大多数 Linux 库都提供 `.pc` 文件
- 自动处理依赖关系和版本检查
- 跨发行版兼容性更好

### Q3: 可以移除 libusb-1.0 依赖吗？

**A:** 不可以。PCL 的 `libpcl_io.so` 在编译时已经链接了 libusb-1.0，如果我们的可执行文件不链接它，会导致符号未定义错误。

### Q4: C++14 是必需的吗？可以用 C++11 吗？

**A:** 不建议。PCL 1.10 使用了 C++14 特性，使用 C++11 可能导致编译错误。如果必须使用 C++11，可能需要降级 PCL 版本，但这可能带来其他兼容性问题。

## 总结

本次修改主要解决了三个问题：

1. **libusb-1.0 链接问题**：通过 PkgConfig 查找并链接 libusb-1.0，解决 PCL 库的依赖
2. **C++ 标准问题**：升级到 C++14，满足 PCL 1.10 的要求
3. **empy 版本冲突**：通过设置 PYTHONPATH 解决（需要在编译命令中设置）

这些修改确保了项目能够成功编译，并且保持了与 ROS noetic 和 PCL 1.10 的兼容性。

## 参考资料

- [PkgConfig CMake 文档](https://cmake.org/cmake/help/latest/module/FindPkgConfig.html)
- [libusb-1.0 官方文档](https://libusb.info/)
- [PCL 官方文档](https://pointclouds.org/)
- [CMake C++ 标准设置](https://cmake.org/cmake/help/latest/prop_tgt/CXX_STANDARD.html)

