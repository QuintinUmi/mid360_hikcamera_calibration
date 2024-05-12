cmake_minimum_required(VERSION 3.0.2)
project(livox_hikcamera_cal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aruco REQUIRED)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PointcloudFilter.cfg
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

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
 INCLUDE_DIRS include/livox_hikcamera_cal
 LIBRARIES livox_hikcamera_cal
 CATKIN_DEPENDS geometry_msgs livox_ros_driver  nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf dynamic_reconfigure
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include/livox_hikcamera_cal/
  /usr/local/include/
)

## Declare a C++ library
add_library(point_cloud_subscriber_publisher_lib
  include include/livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h
  lib/point_cloud_subscriber_publisher.cpp
)

add_library(point_cloud_process_lib
  include include/livox_hikcamera_cal/pointcloud2_opr/point_cloud_subscriber_publisher.h
  include include/livox_hikcamera_cal/pointcloud2_opr/point_cloud_process.h
  lib/point_cloud_process.cpp
)

add_library(dynamic_reconfigure_lib
  include include/livox_hikcamera_cal/pointcloud2_opr/dynamic_reconfigure.h
  lib/dynamic_reconfigure.cpp
)

add_library(recorder_lib
  include include/livox_hikcamera_cal/recorder.h
  lib/recorder.cpp
)

add_library(pcd_saver_lib
  include include/livox_hikcamera_cal/pointcloud2_opr/pcd_saver.h
  lib/pcd_saver.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(point_cloud_subscriber_publisher_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(point_cloud_process_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dynamic_reconfigure_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(recorder_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pcd_saver_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_cloud_subscriber_publisher_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(point_cloud_process_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  point_cloud_subscriber_publisher_lib
)
target_link_libraries(dynamic_reconfigure_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(recorder_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(pcd_saver_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(test1 src/test.cpp)
add_executable(test_outer src/test_outer.cpp)
add_executable(caliboard_cloud_detection src/caliboard_cloud_detection.cpp)
add_executable(point_cloud_filter src/point_cloud_filter.cpp)
# add_executable(rosbag_record_example src/rosbag_record_example.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(test1
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   point_cloud_subscriber_publisher_lib
#   point_cloud_process_lib
#   dynamic_reconfigure_lib
#   recorder_lib
#   pcd_saver_lib
# )
target_link_libraries(test_outer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  point_cloud_subscriber_publisher_lib
  point_cloud_process_lib
  dynamic_reconfigure_lib
  recorder_lib
)
target_link_libraries(caliboard_cloud_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  point_cloud_subscriber_publisher_lib
  point_cloud_process_lib
  dynamic_reconfigure_lib
  recorder_lib
)
target_link_libraries(point_cloud_filter
  ${catkin_LIBRARIES}
  point_cloud_subscriber_publisher_lib
  point_cloud_process_lib
  dynamic_reconfigure_lib
  recorder_lib
)
# target_link_libraries(rosbag_record_example
#   ${catkin_LIBRARIES}
#   point_cloud_subscriber_publisher_lib
#   point_cloud_process_lib
#   dynamic_reconfigure_lib
#   recorder_lib
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/livox_pointcloud2_opr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)