cmake_minimum_required(VERSION 2.8.3)
project(sawyer_robot)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  image_geometry
  image_view
)
find_package(OpenCV REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sawyer_robot
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs image_transport cv_bridge #OpenCV
#  DEPENDS system_lib
#)


include_directories(    #Where can header files be found for the code (most common in C/C++) being built
    include
    ${catkin_INCLUDE_DIRS}
		${GSTREAMER_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		)

set(
    EXECUTABLE_OUTPUT_PATH
    /home/larissa/Documents/vrep
    )

set(
    CMAKE_CXX_FLAGS
    "-std=c++0x ${CMAKE_CXX_FLAGS}"
   )

set(
    LIBRARY_OUTPUT_PATH
    ${PROJECT_SOURCE_DIR}/lib
    )

add_executable(
    sawyer_robot
    src/sawyer_robot.cpp
    )

add_dependencies(
		sawyer_robot
		${catkin_EXPORTED_TARGETS}
		)

target_link_libraries(
    sawyer_robot
		${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
		)
