#include "ros/ros.h"
#include "serial_communication/serial_wrapper.hpp"
#include "tf/transform_listener.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_communication_receiver");
  SerialNode node;

  while (ros::ok()) {
    std::string rx_data;
    try {
      node.read(rx_data);
    }
    catch(const std::exception& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ros::spinOnce();
  }
  return 0;
}