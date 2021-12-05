cmake_minimum_required(VERSION 3.0.2)
project(ncu_mdog)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  i2cpwm_board
  geometry_msgs
  sensor_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

set(SDK_PATH "./sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
  "${SDK_PATH}/src/*.c"
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project_sample
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/project_sample.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/project_sample_node.cpp)
add_executable(ydlidar_node src/driver_src/ydlidar_node.cpp  ${SDK_SRC})
add_executable(ydlidar_client src/driver_src/ydlidar_client.cpp)
target_link_libraries(ydlidar_node
   ${catkin_LIBRARIES} 
 )
target_link_libraries(ydlidar_client
   ${catkin_LIBRARIES} 
 )

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(mpu9250_node src/driver_src/mpu9250_node.cpp)
target_link_libraries(
  mpu9250_node
  ${catkin_LIBRARIES}
  RTIMULib
)

add_executable(mpu9250_client src/driver_src/mpu9250_client.cpp)
target_link_libraries(
  mpu9250_client
  ${catkin_LIBRARIES}
  RTIMULib
)

add_executable(motor_node src/driver_src/motor_node.cpp)
target_link_libraries(motor_node ${catkin_LIBRARIES})

install(TARGETS ydlidar_node ydlidar_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch startup sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
