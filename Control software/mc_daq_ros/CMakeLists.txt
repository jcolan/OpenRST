cmake_minimum_required(VERSION 2.8.3)
project(mc_daq_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)

find_library(ULDAQ_LIB uldaq)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   daq_cmd.srv
  )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usb1608 usb3104 usbquad08
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  )

## Declare a C++ library
add_library(usb1608 src/usb1608/usb1608.cpp)
add_library(usb3104 src/usb3104/usb3104.cpp)
add_library(usbquad08 src/usbquad08/usbquad08.cpp)

## Declare a C++ executable
add_executable(mc_daq_usb1608_stream src/usb1608_stream.cpp )
add_executable(mc_daq_usb3104_stream src/usb3104_stream.cpp )
add_executable(mc_daq_usbquad08_stream src/usbquad08_stream.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(usb1608
  ${ULDAQ_LIB}
  ${catkin_LIBRARIES}
  uldaq
)

target_link_libraries(usb3104
  ${ULDAQ_LIB}
  ${catkin_LIBRARIES}
  uldaq
)

target_link_libraries(usbquad08
  ${ULDAQ_LIB}
  ${catkin_LIBRARIES}
  uldaq
)

target_link_libraries(mc_daq_usb1608_stream
  usb1608
  ${ULDAQ_LIB}
  ${catkin_LIBRARIES}
)

target_link_libraries(mc_daq_usb3104_stream
  usb3104
  ${ULDAQ_LIB}
  ${catkin_LIBRARIES}
)

target_link_libraries(mc_daq_usbquad08_stream
  usbquad08
  ${ULDAQ_LIB}
  ${catkin_LIBRARIES}
)
