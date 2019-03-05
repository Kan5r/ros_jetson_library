#include <ros/ros.h>
#include "serial_node/serial.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string device;
  pnh.param<std::string>("device", device, "/dev/ttyUSB0");

  ros::jetson::Serial serial(device.c_str(), ros::jetson::BR115200, ros::jetson::DPS8N1);
  ros::Rate rate(50);
  std::string str("HelloWorld");

  while (ros::ok()) {
    serial.write(str);
    rate.sleep();
  }

  serial.~Serial();
  return 0;
}
