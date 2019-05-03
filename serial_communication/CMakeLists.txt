cmake_minimum_required(VERSION 2.8.3)
project(serial_communication)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(serial_wrapper src/serial_wrapper.cpp src/file_rw.cpp)
target_link_libraries(serial_wrapper ${catkin_LIBRARIES})

add_executable(serial_communication_receiver src/serial_communication_receiver.cpp)
target_link_libraries(serial_communication_receiver serial_wrapper ${catkin_LIBRARIES})

add_executable(serial_communication_transmitter src/serial_communication_transmitter.cpp)
target_link_libraries(serial_communication_transmitter serial_wrapper ${catkin_LIBRARIES})

add_executable(transmitter_example src/transmitter_example.cpp)
target_link_libraries(transmitter_example ${catkin_LIBRARIES})