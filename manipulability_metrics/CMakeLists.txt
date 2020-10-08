cmake_minimum_required(VERSION 3.0.2)
project(manipulability_metrics)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  eigen_conversions
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS kdl_parser eigen_conversions visualization_msgs
  DEPENDS orocos_kdl EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/chain.cpp
  src/damm.cpp
  src/dual_chain.cpp
  src/ellipsoid.cpp
  src/inverse_condition_number.cpp
  src/manipulability_measure.cpp
  src/markers.cpp
  src/minimum_singular_value.cpp
  src/similarity.cpp
  src/todamm.cpp
  src/tomm.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
