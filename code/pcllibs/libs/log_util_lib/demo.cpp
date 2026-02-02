#include "logger.h"
#include <iostream>

int main(int, char **) {
  LogConfig conf2 = {
      .level = "trace",
      .path = "../log/pv_panels_location.log",
      .size = 5 * 1024 * 1024,
      .count = 10,
      .type = 3,  // 1: 输出到文件  2: 输出到文件和控制台 其他: 输出到控制台
  };
  INITLOG(conf2);
  LOG_INFO("Release Mode");
  LOG_TRACE("ttttttttttttttttttttt");
  LOG_ERROR("mmmmmmmmmmmmmmmmmmmm");
  std::cout << GETLOGLEVEL() << std::endl;
  return 0;
}
