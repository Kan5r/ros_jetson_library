#include "gpio_node/gpio.hpp"
#include "gpio_node/file_rw.hpp"
#include <unistd.h>
#include <fcntl.h>

using namespace ros::jetson;

Gpio::Gpio(J21PinHeader pin_num, Mode mode) {
  pinfile_ = "/sys/class/gpio/gpio";
  pin_num_ = std::to_string(pin_num);
  pinfile_ += pin_num_;
  pinfile_direction_ = pinfile_ + "/direction";
  pinfile_value_ = pinfile_ + "/value";

  exportPin();
  set(mode);
}

Gpio::~Gpio() { /*unexportPin();*/ }

void Gpio::exportPin() {
  fd_ = open("/sys/class/gpio/export", O_WRONLY);
  if (fd_ < 0) { perror("/sys/class/gpio/export"); exit(-1); }
  rw::writeToFile(fd_, pin_num_.c_str(), pin_num_.length()+1);
  close(fd_);
}

void Gpio::unexportPin() {
  fd_ = open("/sys/class/gpio/unexport", O_WRONLY);
  if (fd_ < 0) { perror("/sys/class/gpio/unexport"); exit(-1); }
  rw::writeToFile(fd_, pin_num_.c_str(), pin_num_.length()+1);
  close(fd_);
}

void Gpio::set(Mode mode) {
  fd_ = open(pinfile_direction_.c_str(), O_WRONLY);
  if (fd_ < 0) { perror(pinfile_direction_.c_str()); exit(-1); }

  std::string io;
  switch (mode) {
  case INPUT : io = "in"; break;
  case OUTPUT: io = "out"; break;
  }
  if (rw::writeToFile(fd_, io.c_str(), io.length()+1) != io.length()+1) exit(-1);
  close(fd_);
}

void Gpio::write(Level level) {
  fd_ = open(pinfile_value_.c_str(), O_WRONLY);
  if (fd_ < 0) { perror(pinfile_value_.c_str()); exit(-1); }

  std::string lh;
  switch (level) {
  case LOW : lh = "0"; break;
  case HIGH: lh = "1"; break;
  }
  rw::writeToFile(fd_, lh.c_str(), lh.length()+1);
  close(fd_);
}

Level Gpio::read() {
  fd_ = open(pinfile_value_.c_str(), O_RDONLY);
  if (fd_ < 0) { perror(pinfile_value_.c_str()); exit(-1); }

  char buf[2] = {};
  rw::readFromFile(fd_, buf, 2);
  close(fd_);
  switch (buf[0]) {
  case '0': return LOW;
  case '1': return HIGH;
  }
}
