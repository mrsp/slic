cmake_minimum_required(VERSION 2.8.3)
project(slic)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  image_proc
  cv_bridge
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED)

find_package( OpenCV 2 REQUIRED )

################################################
## Declare ROS messages, services and actions ##
################################################



catkin_package(
INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})



###########
## Build ##
###########


## Declare a C++ executable
 add_executable(${PROJECT_NAME} src/main.cpp src/slic.cpp include/slic.h)

add_dependencies(${PROJECT_NAME} sensor_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} )



install(TARGETS slic
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 install(DIRECTORY launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
