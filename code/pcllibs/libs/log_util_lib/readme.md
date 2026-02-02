config 字段type取值说明：

- 0（或其他值）：日志输出到控制台。
- 1：日志输出到文件，可以指定文件名，也可以使用默认文件名。
- 2：日志输出到控制台和文件。

add_subdirectory(libs/log_util_lib)

target_link_libraries( 
  log_util_lib
)


#include "logger.h"



