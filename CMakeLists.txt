# Copyright (c) 2023 Kosuke Suzuki
# Released under the MIT license
# https://opensource.org/licenses/mit-license.php

cmake_minimum_required(VERSION 3.0.2)
project(ndt_localizer)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

find_package(ndt_omp QUIET)

if (NOT ndt_omp_FOUND)
  message(WARNING
    " The ndt_omp package was not found.\n"
    " To enable multiprocessing NDT using OpenMP, install package from\n \n"
    " \thttps://github.com/koide3/ndt_omp.git\n \n"
    " After installation, please clean and re-build this package.\n"
    " The ndt_omp package is optional, and without it, NDT will run with the single-threaded PCL implementation."
  )
endif()

find_package(PCL REQUIRED COMPONENTS
  io
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ndt_omp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(pose_graph src/pose_graph.cpp)
add_library(ndt_tools src/ndt_tools.cpp)

add_executable(ndt_localizer src/ndt_localizer.cpp)
add_executable(ndt_mapper src/ndt_mapper.cpp)
add_executable(ndt_logger src/ndt_logger.cpp)
add_executable(map_saver src/map_saver.cpp)

if (ndt_omp_FOUND)
  target_compile_definitions(ndt_localizer PRIVATE USE_OPENMP_PCL=1)
  target_compile_definitions(ndt_mapper PRIVATE USE_OPENMP_PCL=1)
else()
  target_compile_definitions(ndt_localizer PRIVATE USE_OPENMP_PCL=0)
  target_compile_definitions(ndt_mapper PRIVATE USE_OPENMP_PCL=0)
endif()

target_link_libraries(ndt_localizer
  ${catkin_LIBRARIES}
  ${ndt_omp_LIBRARIES}
  ndt_tools
)
target_link_libraries(ndt_mapper
  ${catkin_LIBRARIES}
  ${ndt_omp_LIBRARIES}
  pose_graph
  ndt_tools
)
target_link_libraries(ndt_logger
  ${catkin_LIBRARIES}
)
target_link_libraries(map_saver
  ${catkin_LIBRARIES}
)
target_link_libraries(ndt_tools
  ${PCL_LIBRARIES}
)

target_compile_options(ndt_localizer PUBLIC -O3 -Wall)
target_compile_options(ndt_mapper PUBLIC -O3 -Wall)
target_compile_options(ndt_logger PUBLIC -O3 -Wall)
target_compile_options(map_saver PUBLIC -O3 -Wall)
target_compile_options(pose_graph PUBLIC -O3 -Wall)
target_compile_options(ndt_tools PUBLIC -O3 -Wall)

#############
## Install ##
#############

install(TARGETS ndt_localizer ndt_mapper ndt_logger map_saver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
