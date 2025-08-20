cmake_minimum_required(VERSION 3.8)
project(parol6_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(UI_FILES
  mainwindow.ui
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Qt5 (switch to Qt6 if needed)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

add_executable(${PROJECT_NAME}
  main.cpp
  mainwindow.cpp
  ${UI_FILES}
)

# ---- Force ROS include dir for ALL sources (incl. moc/uic) ----
# If your distro env isn't set inside CMake, this falls back to humble.
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_INC_DIR "/opt/ros/$ENV{ROS_DISTRO}/include")
else()
  set(ROS_INC_DIR "/opt/ros/humble/include")
endif()
if(EXISTS "${ROS_INC_DIR}")
  message(STATUS "Adding ROS include dir: ${ROS_INC_DIR}")
  # Global include (covers automoc too)
  include_directories(SYSTEM "${ROS_INC_DIR}")
endif()

# Also fetch interface include dirs from imported targets
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  $<TARGET_PROPERTY:tf2::tf2,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:tf2_ros::tf2_ros,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:tf2_geometry_msgs::tf2_geometry_msgs,INTERFACE_INCLUDE_DIRECTORIES>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  std_srvs
  control_msgs
  moveit_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# Link imported targets explicitly (helps transitive propagation)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${UI_FILES}
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
