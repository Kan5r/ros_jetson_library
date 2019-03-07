#include <ros/ros.h>
#include "gpio_node/gpio.hpp"
#include <string>

using namespace ros::jetson;

int main(int argc, char **argv) {
  ros::init(argc, argv, "gpio_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Gpio gpio_sw(GPIO15, INPUT);
  Gpio gpio_led(GPIO18, OUTPUT);
  bool ispushed = false;

  while (ros::ok()) {
    if (gpio_sw.read()) {
      gpio_led.write(HIGH);
      if (!ispushed) {
        ROS_INFO("Button is pushed");
        ispushed = true;
      }
    } else {
      gpio_led.write(LOW);
      ispushed = false;
    }
  } 

  gpio_sw.~Gpio();
  gpio_led.~Gpio();
  return 0;
}
