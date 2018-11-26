
#if !defined(RM_UART_H_)
#define RM_UART_H_

#include "platform.h"

#include <stdio.h>
#include <time.h>
#include <iostream>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

class Uart {
 private:
#if PLATFORM == MANIFOLD
  int fd;
#endif
  char buf[7];

 private:
  int set_opt(int, int, int, char, int);

 public:
  void init();
  void sendTarget(int, int, int);
};

#endif  // RM_UART_H_
