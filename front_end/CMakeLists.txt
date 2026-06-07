cmake_minimum_required(VERSION 3.0.2)
project(front_end)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FRONT_END_IDE_BUILD "Build front_end without catkin/ROS for IDE indexing only." OFF)
option(FRONT_END_BUILD_JPS_PLANNER_COPY "Compile jps_planner_copy.cpp in IDE-only build." ON)

find_package(Eigen3 REQUIRED)

if(FRONT_END_IDE_BUILD)
  message(STATUS "front_end: using IDE-only standalone build (catkin/ROS not required)")

  set(FRONT_END_IDE_STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ide_stubs)

  add_library(front_end_ide OBJECT
    src/jps_planner/graph_search.cpp
    src/jps_planner/jps_planner.cpp
  )

  target_include_directories(front_end_ide PRIVATE
    ${FRONT_END_IDE_STUB_DIR}
    include
  )

  target_link_libraries(front_end_ide PRIVATE Eigen3::Eigen)

  # IDE-only toggle:
  # Set FRONT_END_BUILD_JPS_PLANNER_COPY=OFF when you no longer want to compile
  # the annotated learning copy src/jps_planner/jps_planner_copy.cpp.
  if(FRONT_END_BUILD_JPS_PLANNER_COPY)
    add_library(front_end_ide_jps_planner_copy OBJECT
      src/jps_planner/jps_planner_copy.cpp
    )

    target_include_directories(front_end_ide_jps_planner_copy PRIVATE
      ${FRONT_END_IDE_STUB_DIR}
      include
    )

    target_link_libraries(front_end_ide_jps_planner_copy PRIVATE Eigen3::Eigen)
  endif()

  add_custom_target(front_end_ide_all DEPENDS front_end_ide)
  if(FRONT_END_BUILD_JPS_PLANNER_COPY)
    add_dependencies(front_end_ide_all front_end_ide_jps_planner_copy)
  endif()

else()
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
    plan_env
  )

  find_package(cmake_modules REQUIRED)
  find_package(PCL REQUIRED)

  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES front_end
   CATKIN_DEPENDS cv_bridge geometry_msgs roscpp sensor_msgs std_msgs visualization_msgs plan_env
  )

  add_library(front_end
    src/jps_planner/graph_search.cpp
    src/jps_planner/jps_planner.cpp
  )

  target_include_directories(front_end PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
endif()
