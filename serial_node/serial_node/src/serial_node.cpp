#include <ros/ros.h>
#include "serial_node/serial.hpp"
#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string device;
  pnh.param<std::string>("device", device, "/dev/ttyUSB0");

  ros::Serial serial(device.c_str(), ros::BR9600, ros::DPS8N1);
  ros::Rate rate(50);
  std::string rx_str; rx_str.resize(256);
  std::string tx_str = "HelloWorld";

  while (ros::ok()) { 
    //serial.read(rx_str);
    /*rx_str = serial;
    std::cout << rx_str << std::endl;
    rx_str.clear();*/

    //serial.write(tx_str);
    serial = tx_str;

    rate.sleep();
  }

  serial.~Serial();
  return 0;
}
