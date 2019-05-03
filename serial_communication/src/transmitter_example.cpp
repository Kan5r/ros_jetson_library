#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_example");
  ros::NodeHandle nh;
  ros::Publisher serial_pub = nh.advertise<std_msgs::String>("serial_transmission_data", 10);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    std_msgs::String serial;
    serial.data = "HelloWorld!";
    serial_pub.publish(serial);
    loop_rate.sleep();
  }
  return 0;
}