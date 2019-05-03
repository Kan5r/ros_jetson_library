#ifndef SERIAL_H_
#define SERIAL_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include <string>

namespace ros {

enum BaudRate {
  BR9600   = B9600,
  BR19200  = B19200,
  BR38400  = B38400,
  BR57600  = B57600,
  BR115200 = B115200,
};

enum Dps {
  DPS8N1, 
  DPS8O1, 
  DPS8E1
};

class Serial {
public:
  Serial(std::string device, std::string baudrate, std::string dps);
  ~Serial();  
  void write(std::string&  tx_data); 
  bool read(std::string& rx_data);

  void operator=(std::string& tx_data);
  operator std::string();

private:
  int fd_;
  termios oldtio_, newtio_;
  std::string device_, baudrate_, dps_;
  void init(std::string baudrate, std::string dps);
};
}

class SerialNode {
public:
  SerialNode();
  ~SerialNode();
  void read(std::string rx_data) throw(std::runtime_error);
private:
  void init();
  void write(const std_msgs::String::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Serial *serial_;
  std::string device_;
  std::string baudrate_;
  std::string dps_;
  bool transmission_;
  bool reception_;
  std_msgs::String rx_data_;
  ros::Subscriber serial_sub_;
  ros::Publisher serial_pub_;
};
#endif
