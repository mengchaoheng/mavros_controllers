cmake_minimum_required(VERSION 3.0.2)
project(trajectory_publisher VERSION 0.0.0)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  controller_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/TrajectoryPublisher.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs nav_msgs geometry_msgs tf mav_planning_msgs controller_msgs mavros_msgs dynamic_reconfigure
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(trajectory_publisher
        src/trajectoryPublisher.cpp
        src/trajectoryPublisher_node.cpp
        src/trajectory.cpp
        src/polynomialtrajectory.cpp
        src/shapetrajectory.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES
  rqt_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
##########
# EXPORT #
##########
#cs_install()
#cs_export()
