#ifndef GPIO_H_
#define GPIO_H_

#include <string>

namespace ros { namespace jetson {

enum J21PinHeader {
  GPIO7  = 396,
  GPIO12 = 392,
  GPIO13 = 397,
  GPIO15 = 255,
  //GPIO16 = 296,
  GPIO18 = 481,
  GPIO22 = 254,
  GPIO29 = 398,
  GPIO31 = 298,
  GPIO32 = 297,
  GPIO33 = 389,
  GPIO35 = 395,
  GPIO37 = 388,
  GPIO38 = 394,
  GPIO40 = 393
};

enum Mode {
  INPUT,
  OUTPUT,
};

enum Level {
  LOW,
  HIGH
};

class Gpio {
public:
  Gpio(J21PinHeader pin_num, Mode mode);
  ~Gpio();

  void set(Mode mode);
  void write(Level level);
  Level read();
  
private:
  int fd_;
  std::string pin_num_, pinfile_, pinfile_direction_, pinfile_value_;
  void exportPin();
  void unexportPin();
};

}}
#endif
