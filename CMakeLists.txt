cmake_minimum_required(VERSION 3.0.2)
project(tracking_pid)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    angles
    dynamic_reconfigure
    geometry_msgs
    message_generation
    nav_msgs
    pluginlib
    roscpp
    roslib
    roslint
    rostest
    std_msgs
    std_srvs
    tf
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
    PidDebug.msg
    traj_point.msg
    trajectory.msg
)

add_action_files(
  DIRECTORY
    action
  FILES
    FollowPath.action
)

generate_dynamic_reconfigure_options(
  cfg/Pid.cfg
  cfg/TargetVelocity.cfg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    angles
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    std_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

add_executable(controller src/controller.cpp src/controller_node.cpp)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/controller.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

include_directories(
  include
    ${catkin_INCLUDE_DIRS}
)

# Configure roslint for nodes

# Roslint cpp
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

install(
  TARGETS
    controller
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    include/controller
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    nodes/path_interpolator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_controller test/test_controller.cpp src/controller.cpp)
  add_dependencies(test_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_controller ${catkin_LIBRARIES})

  catkin_add_nosetests(test/test_interpolator.py)

  add_rostest(test/${PROJECT_NAME}/test_${PROJECT_NAME}.test)
endif()
