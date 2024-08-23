cmake_minimum_required(VERSION 3.4)
project(localizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -fexceptions -g -ggdb -O2")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -pthread -O2 -Wall -Wextra")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  quatro
)
catkin_python_setup()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

set(TBB2020_INCLUDE_DIR "/usr/local/tbb-2019_U8/include") #CMAKE_CURRENT_SOURCE_DIR should be used when in add_subdirectory
set(TBB2020_LIBRARY_DIR "/usr/local/tbb-2019_U8/lib")
include_directories(${TBB2020_INCLUDE_DIR})
link_directories(${TBB2020_LIBRARY_DIR})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localizer
#  CATKIN_DEPENDS rospy geometry_msgs roscpp sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(global_localizer src/node.cpp src/GlobalLocalizer.cpp)

target_link_libraries(global_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  tbb
  ${EIGEN3_LIBS} 
)
