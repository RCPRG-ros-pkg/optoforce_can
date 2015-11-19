cmake_minimum_required(VERSION 2.8)
project(optoforce_can)

find_package(catkin REQUIRED COMPONENTS
    roscpp cmake_modules eigen_conversions geometry_msgs can_driver
  )

find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES optoforce_can
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(optoforce_can_test src/test.cpp src/OptoforceSensor.cpp)
add_library(optoforce_can src/OptoforceSensor.cpp)

target_link_libraries(optoforce_can_test ${catkin_LIBRARIES})
target_link_libraries(optoforce_can ${catkin_LIBRARIES})
