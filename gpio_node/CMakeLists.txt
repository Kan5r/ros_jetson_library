cmake_minimum_required(VERSION 2.8.3)
project(gpio_node)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(file_rw src/file_rw.cpp)
add_dependencies(file_rw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(file_rw ${catkin_LIBRARIES})

add_library(gpio src/gpio.cpp)
target_link_libraries(gpio file_rw ${catkin_LIBRARIES})

add_executable(gpio_node src/gpio_node.cpp)
target_link_libraries(gpio_node file_rw gpio ${catkin_LIBRARIES})
