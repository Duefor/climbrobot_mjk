cmake_minimum_required(VERSION 3.0.2)
project(cs66_ctl_manipulation)

## Compile as C++17, required for Elite SDK features
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
  car_ctl
  message_generation
  roslib
  tf2
  tf2_ros
  tf2_geometry_msgs
  mv3d_rgbd_ros
  moveit_ros_planning_interface
  moveit_core
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cs66_ctl_manipulation
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs trajectory_msgs control_msgs actionlib actionlib_msgs car_ctl tf2 tf2_ros tf2_geometry_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/cs66_robot_controller.cpp
  src/cs66_follow_joint_traj_server.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  elite-cs-series-sdk
)

## Declare C++ executable
add_executable(cs66_follow_joint_trajectory_server src/cs66_follow_joint_traj_server.cpp)


add_executable(task_main src/task_main.cpp)
target_link_libraries(task_main
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(task_main_wall src/task_main_wall.cpp)
target_link_libraries(task_main_wall
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(task_main_mjk src/task_main_mjk.cpp)
target_link_libraries(task_main_mjk
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(camera_calib_eval_node src/camera_calib_eval_node.cpp)
target_link_libraries(camera_calib_eval_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(force_mode_probe_node src/force_mode_probe_node.cpp)
target_link_libraries(force_mode_probe_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

## Add cmake target dependencies of the executable
add_dependencies(cs66_follow_joint_trajectory_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(task_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(task_main_wall ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(task_main_mjk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(camera_calib_eval_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(force_mode_probe_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(cs66_follow_joint_trajectory_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  elite-cs-series-sdk
)

## Install executables
install(TARGETS cs66_follow_joint_trajectory_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml" PATTERN "*.txt"
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
