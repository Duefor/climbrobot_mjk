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