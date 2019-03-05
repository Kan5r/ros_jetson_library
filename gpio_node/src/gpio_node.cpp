#include <ros/ros.h>
#include "gpio_node/gpio.hpp"
#include <string>

using namespace ros::jetson;

int main(int argc, char **argv) {
  ros::init(argc, argv, "gpio_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Gpio gpio(GPIO15, INPUT);
  bool ispushed = false;

  while (ros::ok()) {
    if (gpio.read()) {
      if (!ispushed) {
        ROS_INFO("Button is pushed");
        ispushed = true;
      }
    } else {
      ispushed = false;
    }
  } 

  gpio.~Gpio();
  return 0;
}