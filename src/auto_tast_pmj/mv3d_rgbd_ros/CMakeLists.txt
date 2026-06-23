# 最低需求版本
cmake_minimum_required(VERSION 3.10.2)
# 项目名
project(mv3d_rgbd_ros) # 生成两个变量 PROJECT_BINARY_DIR / PROJECT_SOURCE_DIR

set(MV3D_RGBD_SDK_DIRS /opt/Mv3dRgbdSDK)

# 判断架构
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH_OUTPUT)
if(ARCH_OUTPUT MATCHES "x86_64|amd64")
    set(HOST_PLATFORM "linux64")
elseif(ARCH_OUTPUT MATCHES "i686")
    set(HOST_PLATFORM "linux32")
elseif(ARCH_OUTPUT MATCHES "aarch64")
    set(HOST_PLATFORM "aarch64")
endif()

# 查找需要的 package
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Boost)
find_package(PkgConfig REQUIRED)
pkg_check_modules(USB REQUIRED libusb-1.0)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    pcl_conversions
    pcl_ros
    message_generation
)

add_service_files(
    FILES
    image.srv
    pointcloud.srv
    DetectSteelStamp.srv
    image_pointcloud.srv
)

if(NOT WIN32)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
ADD_DEFINITIONS("-O3")
endif(NOT WIN32)

# 服务器消息传输需要使用 sensor_msgs
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)

MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
MESSAGE(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/common
  ${PROJECT_SOURCE_DIR}/include
  ${MV3D_RGBD_SDK_DIRS}/include
  # 用来关联 catkin 工作空间下的头文件，包括 ros.h 之类
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加需要链接的库文件目录，用来加载外加的=动态链接库或静态链接库的搜索路径，相当于gcc的-L参数
link_directories(
    /usr/lib
    ${PROJECT_SOURCE_DIR}/lib
    ${MV3D_RGBD_SDK_DIRS}/lib/${HOST_PLATFORM}
    ${OpenCV_LIB_DIR}
    ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# add_executable(hik_camera_image_pub src/hik_camera.cpp src/create_camera_image_pub.cpp)
# target_link_libraries(hik_camera_image_pub -lMv3dRgbd ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_image_sub src/create_camera_image_sub.cpp)
# target_link_libraries(hik_camera_image_sub ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_image_client src/hik_camera.cpp src/create_camera_image_client.cpp)
# add_dependencies(hik_camera_image_client ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(hik_camera_image_client -lMv3dRgbd ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_image_server src/create_camera_image_server.cpp)
# add_dependencies(hik_camera_image_server ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(hik_camera_image_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_point_cloud_client src/hik_camera.cpp src/create_camera_point_cloud_client.cpp)
# add_dependencies(hik_camera_point_cloud_client ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(hik_camera_point_cloud_client -lMv3dRgbd ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_point_cloud_server src/hik_camera.cpp src/create_camera_point_cloud_server.cpp)
# add_dependencies(hik_camera_point_cloud_server ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(hik_camera_point_cloud_server -lMv3dRgbd ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_image_pointcloud_client src/hik_camera.cpp src/create_image_pointcloud_client.cpp)
# add_dependencies(hik_camera_image_pointcloud_client ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(hik_camera_image_pointcloud_client -lMv3dRgbd ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# add_executable(hik_camera_image_pointcloud_server src/create_image_pointcloud_server.cpp)
# add_dependencies(hik_camera_image_pointcloud_server ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(hik_camera_image_pointcloud_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${USB_LIBRARIES})

# catkin_install_python(PROGRAMS
#   scripts/camera_test.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )