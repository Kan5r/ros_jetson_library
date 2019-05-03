#include "serial_communication/file_rw.hpp"
#include <unistd.h>

long rw::writeToFile(int fd, const void *buf, unsigned long n) {
  return write(fd, buf, n);
}

long rw::readFromFile(int fd, void *buf, unsigned long nbytes) {
  return read(fd, buf, nbytes);
}