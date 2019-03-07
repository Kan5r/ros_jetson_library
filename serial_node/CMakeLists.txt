cmake_minimum_required(VERSION 2.8.3)
project(serial_node)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp gpio_node)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# add_library(file_rw1 src/file_rw.cpp)
# target_link_libraries(file_rw1 ${catkin_LIBRARIES})

add_library(serial src/serial.cpp)
target_link_libraries(serial file_rw ${catkin_LIBRARIES})

add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node serial file_rw ${catkin_LIBRARIES})
