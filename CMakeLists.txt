cmake_minimum_required(VERSION 3.5.1)
project(ros_optimizer)
# set(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
# -Wno-ignored-attributes
# -Wno-ignored-attributes
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes")
# Add -O0 to remove optimizations when using gcc
IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_geometry
  std_msgs
  message_generation
  geometric_shapes
  # pcl_ros
  # pcl_conversions
  geometry_msgs
  nav_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  # grid_map_rviz_plugin
  grid_map_visualization
  octomap_msgs
  filters

  # grid_map_rviz_plugin
  # grid_map_visualization
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
 # INCLUDE_DIRS include
 # LIBRARIES pack1
 CATKIN_DEPENDS
      roscpp
      rospy
      std_msgs
      message_runtime
      # sensor_msgs
      message_generation
      # pcl_ros
      # pcl_conversions
      # geometry_msgs
      # nav_msgs
      # grid_map_core
      # grid_map_ros
      # grid_map_cv
      # grid_map_filters
      # grid_map_loader
      # grid_map_msgs
      # grid_map_octomap
      # grid_map_rviz_plugin
      # grid_map_visualization
      # cv_bridge
      # octomap_msgs
      # filters
 # DEPENDS system_lib
 # DEPENDS Eigen
)

###########
## Build ##
###########

find_package(Eigen3 REQUIRED)
find_package( OpenCV 3.3.1 REQUIRED )
find_package(yaml-cpp REQUIRED)
# include_directories(HEADERS_EXECUTABLE
#     ${YAML_INCLUDE_DIRS}
# )
# target_link_libraries(${CMAKE_PROJECT_NAME}
#     yaml-cpp
# )


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${YAML_INCLUDE_DIRS}
# ${Eigen_INCLUDE_DIRS}
)


set(OPTI_ENV
  src/environment/robot.cpp
  src/environment/robot.hpp
  src/environment/ocm.hpp
  src/environment/ocm.cpp
  # src/optimizer/optimizer.hpp
  # src/optimizer/optimizer.cpp
  # src/pool.hpp
  # src/pool.cpp
  src/tools/path_tools.h
  src/tools/path_tools.cpp
  # src/optimizer/ga_path_generator.h
  # src/optimizer/ga_path_generator.cpp
  # src/optimizer/grid_search.h
  # src/optimizer/grid_search.cpp
  src/optimizer/optimizer.h
  src/optimizer/optimizer.cpp
  src/tools/debug.h
  src/tools/debug.cpp
  src/tools/mapGen.h
  src/tools/mapGen.cpp
  src/tools/pa_serializer.h
  src/tools/pa_serializer.cpp
  src/tools/genome_tools.h
  src/tools/genome_tools.cpp
  src/tools/configuration.h
  src/tools/configuration.cpp

)

# include_directories(  )


add_executable(opti
  src/main.cpp
  # src/environment/robot.cpp
  # src/environment/robot.hpp
  ${OPTI_ENV}
  )

target_link_libraries(opti ${catkin_LIBRARIES} yaml-cpp)


#############
## Install ##
#############








#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

# catkin_add_gtest(opti-test test/test_ros_opti.cpp)
catkin_add_executable_with_gtest(test-tools
  test/test_tools.cpp
  ${OPTI_ENV}
  # src/tools/path_tools.h
  # src/tools/path_tools.cpp
  # ${catkin_LIBRARIES}
  )

# catkin_add_executable_with_gtest(test-opti
#   test/test_optimizer.cpp
#   ${OPTI_ENV}
#   # src/tools/path_tools.h
#   # src/tools/path_tools.cpp
#   # ${catkin_LIBRARIES}
#   )

# catkin_add_executable_with_gtest(test-app
#   test/test_app.cpp
#   ${OPTI_ENV}
#   # src/tools/path_tools.h
#   # src/tools/path_tools.cpp
#   # ${catkin_LIBRARIES}
#   )

# catkin_add_executable_with_gtest(test-io
#   test/test_io_tools.cpp
#   ${OPTI_ENV}
#   # src/tools/path_tools.h
#   # src/tools/path_tools.cpp
#   # ${catkin_LIBRARIES}
#   )

catkin_add_executable_with_gtest(test-ga
  test/test_new_optimizer.cpp
  ${OPTI_ENV}
  # src/tools/path_tools.h
  # src/tools/path_tools.cpp
  # ${catkin_LIBRARIES}
  )


# if(TARGET test-path-tools)
target_link_libraries(test-tools ${catkin_LIBRARIES} yaml-cpp)
# target_link_libraries(test-opti ${catkin_LIBRARIES})
# target_link_libraries(test-app ${catkin_LIBRARIES})
# target_link_libraries(test-io ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(test-ga ${catkin_LIBRARIES} yaml-cpp)
# endif()
