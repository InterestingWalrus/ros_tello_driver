cmake_minimum_required(VERSION 2.8.3)
project(ros_tello_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  FlightTelemetry.msg
  TelloResponse.msg
)

add_service_files(
  FILES
  TelloAction.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        nav_msgs
        std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ros_tello
               src/tello_driver.cpp 
               src/h264_decoder.cpp )

add_dependencies(ros_tello
 ros_tello_driver_generate_messages_cpp)



target_link_libraries(ros_tello ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} avcodec avutil swscale)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)




