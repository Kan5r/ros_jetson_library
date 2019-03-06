#ifndef SERIAL_RW_H_
#define SERIAL_RW_H_

// unistd.hのwrite(), read()関数が，何故かオーバーロードできないので，この関数を使うこと

namespace rw {

long writeToFile(int fd, const void *buf, unsigned long n);
long readFromFile(int fd, void *buf, unsigned long nbytes);

}

#endif
