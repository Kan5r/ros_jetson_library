#include "gpio_node/gpio.hpp"
#include "gpio_node/file_rw.hpp"
#include <unistd.h>
#include <fcntl.h>

using namespace ros::jetson;

Gpio::Gpio(J21PinHeader pin_num, Mode mode) {
  pinfile_ = "/sys/class/gpio/gpio";
  std::string pin_num_ = std::to_string(pin_num);
  pinfile_ += pin_num_;
  pinfile_direction_ = pinfile_ + "/direction";
  pinfile_value_ = pinfile_ + "/value";

  set(mode);
}

Gpio::~Gpio() {}

void Gpio::set(Mode mode) {
  fd_ = open(pinfile_direction_.c_str(), O_WRONLY);
  if (fd_ < 0) { perror(pinfile_.c_str()); exit(-1); }

  std::string io = std::to_string(mode);
  rw::writeToFile(fd_, io.c_str(), io.length());
  close(fd_);
}

void Gpio::write(Level level) {
  fd_ = open(pinfile_value_.c_str(), O_WRONLY);
  if (fd_ < 0) { perror(pinfile_value_.c_str()); exit(-1); }

  std::string lh = std::to_string(level);
  rw::writeToFile(fd_, lh.c_str(), lh.length());
  close(fd_);
}

Level Gpio::read() {
  fd_ = open(pinfile_value_.c_str(), O_RDONLY);
  if (fd_ < 0) { perror(pinfile_value_.c_str()); exit(-1); }

  char buf[2] = {};
  rw::readFromFile(fd_, buf, 2);
  switch (buf[0]) {
  case '0': return LOW;
  case '1': return HIGH;
  default : return LOW;
  }
}