cmake_minimum_required(VERSION 3.0.2)
project(sp_a_loam_ros)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(SP_A_LOAM REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_CONVERSIONS_INCLUDE_DIRS})

add_executable(scan_registration_node src/scan_registration_wrapper.cpp src/scan_registration_node.cpp)
target_link_libraries(scan_registration_node ${catkin_LIBRARIES} SP_A_LOAM)

add_executable(laser_odometry_node src/laser_odometry_wrapper.cpp src/laser_odometry_node.cpp)
target_link_libraries(laser_odometry_node ${catkin_LIBRARIES} SP_A_LOAM)

install(TARGETS scan_registration_node laser_odometry_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
