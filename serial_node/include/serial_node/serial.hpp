#ifndef SERIAL_H_
#define SERIAL_H_

#include <termios.h>
#include <string>

namespace ros { namespace jetson {

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
  Serial(const char* device, BaudRate baudrate, Dps dps);
  ~Serial();
  void init(BaudRate baudrate, Dps dps);
  void write(std::string&  tx_data);
  void read(std::string& rx_data);

private:
  int fd_;
  termios oldtio_, newtio_;
};

}}

#endif
