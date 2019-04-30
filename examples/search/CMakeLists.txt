cmake_minimum_required(VERSION 2.8.3)
project(search)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  actionlib_msgs
  actionlib
)

find_package(Despot CONFIG REQUIRED)

add_action_files(
  DIRECTORY action
  FILES
  TrackObs.action
)


generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

#add_executable(hsrb_interface src/robot_controller/hsrb_interface.cpp)
#add_dependencies(hsrb_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hsrb_interface ${catkin_LIBRARIES})

## Declare a C++ executable
#add_executable(tiger_controller src/robot_controller/tiger_controller.cpp)
#add_dependencies(tiger_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(tiger_controller ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(pomdp_planner  src/search.cpp
    src/search_world.cpp src/main.cpp)
add_dependencies(pomdp_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pomdp_planner despot ${catkin_LIBRARIES})




#add_executable("${PROJECT_NAME}_tiger"
  #src/search.cpp
  #src/main.cpp
#)
#target_link_libraries("${PROJECT_NAME}_tiger"
  #"${PROJECT_NAME}"
#)
#install(TARGETS "${PROJECT_NAME}_tiger"
  #RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
#)
