cmake_minimum_required(VERSION 3.0.2)
project(social_nav_tests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  sensor_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  actionlib
  actionlib_msgs
  nav_msgs
  pedsim_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES social_nav_tests
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs tf tf2 tf2_ros actionlib nav_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(odom_publisher src/odom_publisher.cpp)
# target_link_libraries(odom_publisher ${catkin_LIBRARIES})
# add_dependencies(odom_publisher ${catkin_EXPORTED_TARGETS})
