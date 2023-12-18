cmake_minimum_required(VERSION 3.8)
project(robot_mobile_pkg_cpp)
#
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#
#
set(ROS_DEPENDS
  roscpp
  std_msgs
  )
#

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
#

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5Widgets REQUIRED)

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files


add_executable(my_node_ihm
  src/my_node.cpp
  src/mainwindow.cpp
  include/mainwindow.hpp
  #include/ui_mainwindow.h
  include/mainwindow.ui
)
ament_target_dependencies(my_node_ihm rclcpp std_msgs sensor_msgs)
target_link_libraries(my_node_ihm Qt5::Widgets)



install(TARGETS 
  my_node_ihm
  DESTINATION lib/${PROJECT_NAME})
ament_package()
