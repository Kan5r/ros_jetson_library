#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial_communication/serial_wrapper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_communication_transmitter");
  ros::NodeHandle nh;
  SerialNode node;
  ros::spin();
  
  return 0;
}