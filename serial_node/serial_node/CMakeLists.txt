cmake_minimum_required(VERSION 2.8.3)
project(serial_node)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(serial src/serial.cpp src/file_rw.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node serial file_rw ${catkin_LIBRARIES})
