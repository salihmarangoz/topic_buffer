cmake_minimum_required(VERSION 3.0.2)
project(topic_buffer)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  topic_tools
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS nodelet roscpp topic_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(topic_buffer
  src/topic_buffer_nodelet.cpp
)

target_link_libraries(topic_buffer ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(topic_buffer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
