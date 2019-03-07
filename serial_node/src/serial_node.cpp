#include <ros/ros.h>
#include "serial_node/serial.hpp"
#include <iostream>

using namespace ros::jetson;

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string device;
  pnh.param<std::string>("device", device, "/dev/ttyUSB0");

  Serial serial(device.c_str(), BR9600, DPS8N1);
  ros::Rate rate(200);
  std::string rx_str; rx_str.resize(256);
  std::string tx_str = "HelloWorld";

  while (ros::ok()) { 
    serial.read(rx_str);
    ROS_INFO("%s", rx_str.c_str());
    rx_str.clear();

    serial.write(tx_str);
    
    rate.sleep();
  }

  serial.~Serial();
  return 0;
}
