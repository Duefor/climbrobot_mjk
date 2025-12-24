# 人机接口尺度映射
## 25/12/23
为了在该包中引用robot_set中的自定义msg：TCPstate，需要对其做一系列处理：
1. package.xml
必须声明对 msg 包的依赖：

<build_depend>pkg_msgs</build_depend>
<exec_depend>pkg_msgs</exec_depend>

<build_depend>message_runtime</build_depend>
<exec_depend>message_runtime</exec_depend>

2. CMakeLists.txt
    1. 查找依赖
    ```cmake
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    pkg_msgs
    )
    ```

    2. 声明 catkin_package
    ```cmake
    catkin_package(
    CATKIN_DEPENDS roscpp pkg_msgs
    )
    ```

    3. 添加 include 路径
    ```cmake
    include_directories(
    ${catkin_INCLUDE_DIRS}
    )
    ```

    4. 添加对 msg 生成的依赖（很重要）
    ```cmake
    add_executable(node src/node.cpp)
    add_dependencies(node ${catkin_EXPORTED_TARGETS})
    target_link_libraries(node ${catkin_LIBRARIES})
    ```
> 需要注意的是，在touch_set中引用robot_set中的自定义msg：TCPstate，则无需进行以上处理，这是为何

## 25/12/24
touch笔的初始位姿为：[0.0,0.08,-0.065] [-0.13629035784750088, 0.5623532458328435, 2.6132248962045517]
touch的活动范围为：x[-0.21,0.21] y[-0.08,0.08] z[-0.11,0.18]
cs66的活动范围为：x[-0.8,0.8] y[-0.8,0.8] z[-0.8,0.8]
cs66机械臂的初始位姿由如下公式得到：
$${O}_{\mathrm{Ui}}=U_{i\mathrm{min}}+\frac{\left|P_{Oi}-P_{i\mathrm{min}}\right|}{P_{i\mathrm{max}}-P_{i\mathrm{min}}}*(U_{i\mathrm{max}}-U_{i\mathrm{min}})$$
其中，i表示x,y,z三个方向之一，$O_{ui}$为cs66机器人设定的初始位置i某一方向上的值，$U_{imin}$为cs66机器人实际范围的最小值，$U_{imax}$为cs66机器人实际范围的最大值，$P_{imax}$与Pimin为Geomagic Touch设备范围最大与最小值，$P_{oi}$为Geomagic Touch初始位置i方向上的值。
映射比例由如下公式得到：
$$k_i=\frac{U_{i\max}-O_{Ui}}{P_{i\max}-O_{Pi}}$$
其中，$O_{pi}$是touch的初始位置