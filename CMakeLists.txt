cmake_minimum_required(VERSION 2.8)
project(optoforce_can)

find_package(catkin REQUIRED COMPONENTS
    roscpp cmake_modules eigen_conversions geometry_msgs can_driver
  )

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
endif()

find_package(Eigen REQUIRED)

#Start of user code user_build
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
endif()
#End of user code

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(optoforce_can_test src/test.cpp src/OptoforceSensor.cpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(optoforce_can_test ${catkin_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(optoforce_can_test ${catkin_LIBRARIES})
endif()

