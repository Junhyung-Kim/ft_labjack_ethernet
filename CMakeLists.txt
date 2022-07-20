cmake_minimum_required(VERSION 3.0.2)
project(ft_labjack_ethernet)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES voltage
 CATKIN_DEPENDS roscpp rospy std_msgs
## DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LJM_Utilities.cpp
)
set(HEADS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ft_labjack_ethernet/LJM_Utilities.h
    /usr/local/lib/libLabJackM.so
)
## Declare a C++ library
add_library(headers  ${SRCS}
)

add_executable(ft_labjack_ethernet  src/LJM_Utilities.cpp) 

add_dependencies(ft_labjack_ethernet 
	   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
	   ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(ft_labjack_ethernet
   headers
   ${HEADS}
   ${catkin_LIBRARIES}
)     
