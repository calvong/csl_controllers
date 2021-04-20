cmake_minimum_required(VERSION 3.4)
project(csl_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.1 REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES csl_controllers FAMGP
  LIBRARIES csl_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    std_msgs
  DEPENDS Franka
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


add_library(csl_controllers
  src/csl_impedance_controller.cpp
)

# add_library(FAMGP 
#   include/csl_controllers/EigKernel.cpp
#   include/csl_controllers/ExpEigKernel.cpp
#   include/csl_controllers/PerEigKernel.cpp
#   include/csl_controllers/FAMGP.cpp
# )

add_dependencies(csl_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(csl_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

# target_link_libraries(FAMGP PUBLIC ${catkin_LIBRARIES})

target_include_directories(csl_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# target_include_directories (FAMGP SYSTEM PUBLIC 
#   ${EIGEN3_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

target_include_directories(csl_controllers PUBLIC
  include
)

install(TARGETS csl_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES csl_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)


if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(csl_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(csl_controllers
    FILES ${SOURCES}
    DEPENDS csl_controllers
  )
endif()