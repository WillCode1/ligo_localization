cmake_minimum_required(VERSION 3.5)
project(ligo_localization)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -Wno-unused-local-typedefs -Wno-format-truncation -Wno-unused-result -Wno-pedantic -Wno-unused-parameter -Wno-unused-variable")

# echo "/home/will/coredump/core-%e-%p-%s-%t" > /proc/sys/kernel/core_pattern
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(aarch32)|(AARCH32)|(aarch64)|(AARCH64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 6)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    add_definitions(-DBNB_PROC_NUM=6)
    message("core for MP: 4")
  elseif(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    add_definitions(-DBNB_PROC_NUM=4)
    message("core for MP: 4")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    add_definitions(-DBNB_PROC_NUM=3)
    message("core for MP: 3")
  else()
    add_definitions(-DMP_PROC_NUM=1)
    add_definitions(-DBNB_PROC_NUM=2)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
  add_definitions(-DBNB_PROC_NUM=2)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  cv_bridge
  livox_ros_driver
  message_generation
  eigen_conversions
  gnss_comm
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization timer thread chrono)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_message_files(
  FILES
  LocalSensorExternalTrigger.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL GTSAM CERES
  INCLUDE_DIRS
)

add_executable(ligo_localization src/laserMapping.cpp 
                include/Urbannav_process/handler.cpp
                src/li_initialization.cpp src/parameters.cpp src/preprocess.cpp src/Estimator.cpp 
                src/IMU_Processing.cpp src/GNSS_Processing_fg.cpp src/GNSS_Initialization.cpp src/GNSS_Assignment.cpp
                src/NMEA_Processing_fg.cpp src/NMEA_Assignment.cpp
                include/backend_optimization/global_localization/scancontext/Scancontext.cpp
                include/backend_optimization/global_localization/InitCoordinate.cpp)
target_link_libraries(ligo_localization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ceres gtsam stdc++fs)
# target_link_libraries(ligo_localization dw)
target_link_libraries(ligo_localization ${Sophus_LIBRARIES} fmt)
# target_include_directories(ligo_localization PRIVATE ${PYTHON_INCLUDE_DIRS})



