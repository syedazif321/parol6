cmake_minimum_required(VERSION 3.10)

project(parol6_gui VERSION 0.1 LANGUAGES CXX)

# Enable Qt and C++ settings
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Qt
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    Qt${QT_VERSION_MAJOR}::Widgets
)

# Declare ROS 2 dependencies for this target
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    geometry_msgs
    std_srvs
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
