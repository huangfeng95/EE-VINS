cmake_minimum_required(VERSION 2.8.3)
project(EE-VINS)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    dvs_msgs
    dv_ros_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport
    pcl_conversions
    pcl_ros
    roslib)

find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(vins_node
    vins_estimator/lock.cpp
    vins_estimator/rosNodeTest.cpp
    vins_estimator/estimator/parameters.cpp
    vins_estimator/estimator/estimator.cpp
    vins_estimator/estimator/feature_manager.cpp
    vins_estimator/factor/pose_local_parameterization.cpp
    vins_estimator/factor/projectionTwoFrameOneCamFactor.cpp
    vins_estimator/factor/projectionTwoFrameTwoCamFactor.cpp
    vins_estimator/factor/projectionOneFrameTwoCamFactor.cpp
    vins_estimator/factor/marginalization_factor.cpp
    vins_estimator/utility/utility.cpp
    vins_estimator/utility/visualization.cpp
    vins_estimator/utility/CameraPoseVisualization.cpp
    vins_estimator/initial/solve_5pts.cpp
    vins_estimator/initial/initial_aligment.cpp
    vins_estimator/initial/initial_sfm.cpp
    vins_estimator/initial/initial_ex_rotation.cpp
    vins_estimator/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
