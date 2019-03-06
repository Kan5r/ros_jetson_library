#include <ros/ros.h>
#include "serial_node/serial.hpp"

using namespace ros::jetson;

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string device;
  pnh.param<std::string>("device", device, "/dev/ttyUSB0");

  Serial serial(device.c_str(), BR115200, DPS8N1);
  ros::Rate rate(50);
  std::string str("HelloWorld");

  while (ros::ok()) {
    serial.write(str);
    rate.sleep();
  }

  serial.~Serial();
  return 0;
}
