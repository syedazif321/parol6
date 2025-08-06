cmake_minimum_required(VERSION 3.8)
project(camera_tf_publisher_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/camera_tf_publisher_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros geometry_msgs)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)


ament_package()
