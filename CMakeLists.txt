cmake_minimum_required(VERSION 2.8)
project(tc2li_slam)

IF (NOT CMAKE_BUILD_TYPE) # MUST BE Release
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

add_definitions(-w)
add_definitions(-DPCL_NO_PRECOMPILE)

# Macro parameters 1: No effects on execution results
add_definitions(-DHUMAN_READABLE=0)  # bool: whether to tidy up and print (too detailed) results to screen
add_definitions(-DSAVE_MID_FILE=0)   # bool: whether to save intermediate files (unnecessary to the program)
add_definitions(-DPUB_ROS_MSG=0)     # bool: whether to publish message to the program

# Macro parameters 2: Affects execution results
add_definitions(-DDYNAMIC_THRES=0)   # bool: whether to use dynamic threshold when checking (lift lower bound on the go)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3 -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -ggdb -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -ggdb -march=native")
# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV 4.2 not found.")
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        geometry_msgs
        sensor_msgs
        nav_msgs
        std_msgs
        message_filters
        roscpp
        rospy
        tf
        tf2
        pcl_conversions
        message_generation
)

add_message_files(
        FILES
        Pose6D.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)
find_package(Eigen3 3.3.7 REQUIRED)
find_package(PCL 1.12 REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs pcl_conversions dynamic_reconfigure message_runtime
        LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/slam_framework
        ${PROJECT_SOURCE_DIR}/slam_framework/include
        ${PROJECT_SOURCE_DIR}/slam_framework/include/CameraModels
        ${PROJECT_SOURCE_DIR}/slam_framework/Thirdparty
        ${PROJECT_SOURCE_DIR}/slam_framework/Thirdparty
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

include(${PROJECT_SOURCE_DIR}/slam_framework/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/slam_framework/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/slam_framework/lib)

file(GLOB_RECURSE SLAM_FRAMEWORK_SOURCES
        "slam_framework/src/*.cpp"
        "slam_framework/src/*.cc"
        "slam_framework/include/*.cpp"
        "slam_framework/include/*.hpp"
        "slam_framework/include/*.h")

add_library(${PROJECT_NAME} SHARED
        ${SLAM_FRAMEWORK_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/slam_framework/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/slam_framework/Thirdparty/g2o/lib/libg2o.so
        -lboost_system
        -lboost_serialization
        -lcrypto
)

## ROS node
add_executable(ros_stereo_inertial_lidar
        src/examples/camera_lidar_inertial.cc
)
target_link_libraries(ros_stereo_inertial_lidar
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

## ROS node
add_executable(ros_stereo_lidar
        src/examples/camera_lidar.cc
)
target_link_libraries(ros_stereo_lidar
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

