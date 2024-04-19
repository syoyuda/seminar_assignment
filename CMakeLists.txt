cmake_minimum_required(VERSION 3.0.2)
project(seminar_assignment)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
#  LIBRARIES kadai5
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable( planning src/planning )
add_executable( planning2 src/planning2 )
add_executable( planning3 src/planning3 )
add_executable( odom src/odom )
add_executable( odom2 src/odom2 )

target_link_libraries(planning  ${catkin_LIBRARIES} )
target_link_libraries(planning2  ${catkin_LIBRARIES} )
target_link_libraries(planning3  ${catkin_LIBRARIES} )
target_link_libraries( odom  ${catkin_LIBRARIES} )
target_link_libraries( odom2  ${catkin_LIBRARIES} )