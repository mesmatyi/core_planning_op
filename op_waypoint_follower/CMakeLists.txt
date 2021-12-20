cmake_minimum_required(VERSION 2.8.3)
project(op_waypoint_follower)

find_package(autoware_build_flags REQUIRED)

find_package(OpenCV REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_msgs
  autoware_can_msgs
  geometry_msgs
  op_ros_helpers
  map_file
  op_planner  
  op_utility
  roscpp
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/pid_tune.cfg
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_msgs
    autoware_can_msgs
    geometry_msgs
    op_ros_helpers
    map_file
    op_planner    
    op_utility
    dynamic_reconfigure
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
## SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  #  /usr/local/include/zmp/minivan
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)
add_executable(
  op_waypoint_follower
  nodes/op_waypoint_follower.cpp
  nodes/op_waypoint_follower_core.cpp
)
target_link_libraries(
  op_waypoint_follower
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)



#link_directories(/usr/local/lib/zmp/minivan)
#add_executable(op_waypoint_follower nodes/op_waypoint_follower.cpp nodes/op_waypoint_follower_core.cpp )
#target_link_libraries(op_waypoint_follower utilityh plannerh CANUSB  ${catkin_LIBRARIES})

add_dependencies(
  op_waypoint_follower
  
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(op_waypoint_follower ${PROJECT_NAME}_gencfg)

install(
  TARGETS op_waypoint_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)
