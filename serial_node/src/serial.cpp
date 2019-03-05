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
  fd_ = open(device, O_RDWR); //読み書きモードでオープン
  if (fd_ < 0) {
    perror(device);
    exit(-1);
}

  tcgetattr(fd_, &oldtio_); //現在のポート設定を待避
  init(baudrate, dps);
}

Serial::~Serial() {
  tcsetattr(fd_, TCSANOW, &oldtio_); //設定をもとに戻す
  close(fd_);
}

void Serial::init(BaudRate baudrate, Dps dps) {

  newtio_.c_lflag &= ~ICANON; //非カノニカルモード

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
  newtio_.c_cc[VTIME] = 0;   //キャラクタ間タイマは未使用
  newtio_.c_cc[VMIN]  = 1;   //バッファに一文字入るまで待機

  cfsetispeed(&newtio_, baudrate);
 
  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &newtio_);
}

void Serial::write(std::string& tx_data) {
  rw::writeToFile(fd_, tx_data.c_str(), tx_data.length()+1);
}

void Serial::read(std::string& rx_data) {
  char* rx_data_ = new char[rx_data.length()];
  rw::readFromFile(fd_, rx_data_, rx_data.length());
  rx_data = rx_data_;
}
