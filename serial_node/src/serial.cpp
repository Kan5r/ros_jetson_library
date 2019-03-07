#include "serial_node/serial.hpp"
#include "serial_node/file_rw.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using namespace ros::jetson;

Serial::Serial(const char* device, BaudRate baudrate, Dps dps) : 
  fd_(0), oldtio_(), newtio_()
{
  fd_ = open(device, O_RDWR); 
  if (fd_ < 0) {
    perror(device);
    exit(-1);
  }

  tcgetattr(fd_, &oldtio_); 
  init(baudrate, dps);
}

Serial::~Serial() {
  tcsetattr(fd_, TCSANOW, &oldtio_); 
  close(fd_);
}

void Serial::init(BaudRate baudrate, Dps dps) {

  newtio_.c_lflag &= ~ICANON;

  switch (dps) {
  case Dps::DPS8N1:
    newtio_.c_cflag = CS8 | CLOCAL | CREAD;
    break;
  case Dps::DPS8O1:
    newtio_.c_cflag = CS8 | CLOCAL | CREAD | PARENB | PARODD;
    break;
  case Dps::DPS8E1:
    newtio_.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
    break;
  }
  
  newtio_.c_iflag = INPCK; 
  newtio_.c_oflag = 0;

  newtio_.c_lflag = 0;
  newtio_.c_cc[VTIME] = 0;   
  newtio_.c_cc[VMIN]  = 1;

  cfsetispeed(&newtio_, baudrate);
 
  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &newtio_);
}

void Serial::write(std::string& tx_data) {
  rw::writeToFile(fd_, tx_data.c_str(), tx_data.length()+1);
}

void Serial::read(std::string& rx_data) {
  char character[1];
  while (1){
    rw::readFromFile(fd_, character, 1);
    if (character[0] == '\0') break;
    rx_data += character;
  }
}
