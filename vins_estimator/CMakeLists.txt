cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_models
  image_transport
)

find_package(OpenCV REQUIRED)
message(WARNING "## OpenCV_VERSION      : " ${OpenCV_VERSION})
message(WARNING "## OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})
message(WARNING "## OpenCV_LIBRARIES    : " ${OpenCV_LIBRARIES})
message(WARNING "## OpenCV_LIBS         : " ${OpenCV_LIBS})

find_package(Ceres REQUIRED)

find_package(Glog REQUIRED)
# include_directories(${Glog_INCLUDE_DIRS})

# include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Glog_INCLUDE_DIRS}
)

catkin_package()

add_library(vins_lib
  src/estimator/parameters.cpp
  src/estimator/estimator.cpp
  src/estimator/feature_manager.cpp
  src/featureTracker/feature_tracker.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_alignment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projectionTwoFrameOneCamFactor.cpp
  src/factor/projectionTwoFrameTwoCamFactor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility_ros/RosVisualization.cpp
  src/utility_ros/CameraPoseVisualization.cpp
)

target_link_libraries(vins_lib 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${CERES_LIBRARIES}
)


add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib) 

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test vins_lib) 

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test vins_lib) 

