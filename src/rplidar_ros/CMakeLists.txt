cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosconsole
  sensor_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   coordinate_msg.msg
#   Message2.msg
 )
generate_messages(
DEPENDENCIES
 std_msgs
)

#寻找OpenCV库
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
##include_directories(
### include
##  ${catkin_INCLUDE_DIRS}
##  include/
##)

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})



install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#添加链接库
TARGET_LINK_LIBRARIES(rplidarNodeClient ${OpenCV_LIBS}) #robot_vision
TARGET_LINK_LIBRARIES(rplidarNode ${OpenCV_LIBS}) #robot_vision

