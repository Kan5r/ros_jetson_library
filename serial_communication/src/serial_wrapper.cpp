#include "serial_communication/serial_wrapper.hpp"
#include "serial_communication/file_rw.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>

using ros::Serial;

Serial::Serial(std::string device, std::string baudrate, std::string dps) : 
  fd_(0), 
  oldtio_(), 
  newtio_(),
  device_(device),
  baudrate_(baudrate),
  dps_(dps)
{
  fd_ = open(device.c_str(), O_RDWR); 
  if (fd_ < 0) {
    perror(device.c_str());
    exit(-1);
  }

  tcgetattr(fd_, &oldtio_); 
  init(baudrate, dps);
}

Serial::~Serial() 
{
  tcsetattr(fd_, TCSANOW, &oldtio_); 
  close(fd_);
}

void Serial::init(std::string baudrate, std::string dps) 
{
  BaudRate baudrate_;
  if (baudrate == "9600") baudrate_ = ros::BR9600;
  else if (baudrate == "19600") baudrate_ = ros::BR19200;
  else if (baudrate == "38400") baudrate_ = ros::BR38400;
  else if (baudrate == "57600") baudrate_ = ros::BR57600;
  else if (baudrate == "115200") baudrate_ = ros::BR115200;

  newtio_.c_lflag &= ~ICANON;

  if (dps == "8N1") newtio_.c_cflag = CS8 | CLOCAL | CREAD;
  else if (dps == "8O1") newtio_.c_cflag = CS8 | CLOCAL | CREAD | PARENB | PARODD;
  else if (dps == "8E1") newtio_.c_cflag = CS8 | CLOCAL | CREAD | PARENB;

  newtio_.c_iflag = INPCK; 
  newtio_.c_oflag = 0;

  newtio_.c_lflag = 0;
  newtio_.c_cc[VTIME] = 0;   
  newtio_.c_cc[VMIN]  = 0;

  cfsetispeed(&newtio_, baudrate_);
 
  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &newtio_);
}

void Serial::write(std::string& tx_data)
{
  rw::writeToFile(fd_, tx_data.c_str(), tx_data.length()+1);
}

bool Serial::read(std::string& rx_data) 
{
  char character[1];
  bool success;
  
  ros::Time start_time;
  ros::Time current_time;

  start_time = current_time = ros::Time::now();

  while (1) {
    current_time = ros::Time::now();
    if ((current_time - start_time).toSec() >= 1.0) {
      success = false;
      break;
    } 
   
    if (!rw::readFromFile(fd_, character, 1)) continue;
    
    if (character[0] == '\0') {
      success = true;
      break;
    }
    rx_data += character;
  }
  return success;
}

void Serial::operator=(std::string& tx_data) 
{
  write(tx_data);
}

Serial::operator std::string() 
{
  std::string rx_data;
  read(rx_data);
  return rx_data;
}


SerialNode::SerialNode() :
nh_(""),
pnh_("~")
{
  pnh_.param<std::string>("device", device_, "/dev/ttyUSB0");
  pnh_.param<std::string>("baudrate", baudrate_, "115200");
  pnh_.param<std::string>("dps", dps_, "8N1");
  if (!pnh_.getParam("use_transmission", transmission_)) {
    ROS_ERROR("please set whether serial transmits data");
    exit(-1);
  }
  if (!pnh_.getParam("use_reception", reception_)) {
    ROS_ERROR("please set whether serial receives data");
    exit(-1);
  }
  if (reception_) serial_pub_ = nh_.advertise<std_msgs::String>("serial_reception_data", 10);
  if (transmission_) serial_sub_ = nh_.subscribe("serial_transmission_data", 10, &SerialNode::write, this);
  serial_ = new ros::Serial(device_, baudrate_, dps_);
}

SerialNode::~SerialNode() {

  serial_->~Serial();
}


void SerialNode::write(const std_msgs::String::ConstPtr& msg) 
{
  ROS_INFO_STREAM(msg->data);
  std::string serial_data = msg->data;
  serial_->write(serial_data);
}

void SerialNode::read(std::string rx_data) throw(std::runtime_error)
{
  if (!serial_->read(rx_data)) 
    return throw  std::runtime_error("buffer is empty");
  ROS_INFO_STREAM(rx_data);
  rx_data_.data = rx_data;
  serial_pub_.publish(rx_data_);
}
