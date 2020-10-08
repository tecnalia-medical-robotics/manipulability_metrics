cmake_minimum_required(VERSION 3.0.2)
project(manipulability_metrics_examples)

find_package(catkin REQUIRED COMPONENTS
  kdl_conversions
  manipulability_metrics
  roscpp
  sensor_msgs
  tf2_ros
  urdf
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(panda_manipulability_demo src/panda_manipulability_demo.cpp)
target_link_libraries(panda_manipulability_demo ${catkin_LIBRARIES})

add_executable(dual_panda_manipulability_demo src/dual_panda_manipulability_demo.cpp)
target_link_libraries(dual_panda_manipulability_demo ${catkin_LIBRARIES})

install(TARGETS panda_manipulability_demo dual_panda_manipulability_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
