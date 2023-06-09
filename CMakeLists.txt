cmake_minimum_required(VERSION 3.0.2)
project(dynamixel)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  dynamixel_workbench_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
    LIBRARIES dynamixel_current_2port
  CATKIN_DEPENDS
    dynamixel_sdk 
    dynamixel_workbench_msgs 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
  DEPENDS 
    system_lib
    Boost
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIR}
  ${dynamixel_workbench_msgs_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ./include
)

add_executable(
  dynamixel
  src/main.cpp
  src/dynamixel.cpp ${dynamixel.hpp}
  src/callback.cpp ${callback.hpp}
  src/dynamixel_controller.cpp ${dynamixel_controller.hpp}
  src/Walkingpattern_generator.cc ${Walkingpattern_generator.hpp}
  src/sensor.cpp ${sensor.hpp}

)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel ${catkin_LIBRARIES})

install(TARGETS dynamixel
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
