cmake_minimum_required(VERSION 2.8.3)
project(mtrackercar)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs tf message_generation)

add_service_files(FILES Trigger.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include/mtrackercar
  CATKIN_DEPENDS roscpp geometry_msgs tf message_runtime
  DEPENDS system_lib)

include_directories(
  #${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/mtrackercar)

add_executable(mtrackercar
  src/mtrackercar.cpp  src/rs232.c
  include/mtrackercar/Robot.hpp
  include/mtrackercar/Serial.hpp
  include/mtrackercar/rs232.h)
add_dependencies(mtrackercar mtracker_generate_messages_cpp)
target_link_libraries(mtrackercar ${catkin_LIBRARIES})


add_executable(controller
  src/controller.cpp)
add_dependencies(controller mtracker_generate_messages_cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(circular_trajectory
  src/circular_trajectory.cpp)
add_dependencies(circular_trajectory mtracker_generate_messages_cpp)
target_link_libraries(circular_trajectory ${catkin_LIBRARIES})

add_executable(simulator
  src/simulator.cpp)
add_dependencies(simulator mtracker_generate_messages_cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
