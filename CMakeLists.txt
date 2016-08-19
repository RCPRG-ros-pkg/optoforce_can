cmake_minimum_required(VERSION 2.8)
project(optoforce_can)

find_package(catkin REQUIRED COMPONENTS
    roscpp eigen_conversions geometry_msgs can_driver
  )

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES optoforce_can
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(optoforce_can_test src/test.cpp src/OptoforceSensor.cpp)
add_library(optoforce_can src/OptoforceSensor.cpp)

target_link_libraries(optoforce_can_test ${catkin_LIBRARIES})
target_link_libraries(optoforce_can ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
