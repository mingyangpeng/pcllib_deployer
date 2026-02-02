#include "logger.h"

void Logger::Init(const LogConfig &conf) {
  if (conf.type == 1) {
    // 日志输出到文件
    loggerPtr = spdlog::rotating_logger_mt<spdlog::async_factory>(
        "file_logger", conf.path.c_str(), conf.size, conf.count);
    loggerPtr->set_pattern("%Y-%m-%d %H:%M:%S | %l | %s-%!:%# - %v");
    // 设置日志级别
    loggerPtr->set_level(spdlog::level::from_str(conf.level));
    // 设置刷新日志的日志级别，当出现level或更高级别日志时，立刻刷新日志到 disk
    loggerPtr->flush_on(spdlog::level::from_str(conf.level));

  } else if (conf.type == 2) {
    /* file sink */
    // 2：日志输出到控制台和文件。
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        conf.path.c_str(), conf.size, conf.count);
    file_sink->set_level(spdlog::level::from_str(conf.level));
    file_sink->set_pattern("%Y-%m-%d %H:%M:%S.%f |%l | %s-%!:%# - %v");

    /* 控制台sink */
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::from_str(conf.level));
    console_sink->set_pattern("%Y-%m-%d %H:%M:%S.%f |%l | %s-%!:%# - %v");

    /* Sink组合 */
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(console_sink);
    sinks.push_back(file_sink);
    loggerPtr = std::make_shared<spdlog::logger>("multi-sink", begin(sinks),
                                                 end(sinks));
    loggerPtr->set_level(spdlog::level::from_str(conf.level));
    loggerPtr->flush_on(spdlog::level::from_str(conf.level));
  } else {
    // 日志输出到控制台和文件。
    loggerPtr = spdlog::stdout_color_mt("console");
    loggerPtr->set_pattern("%Y-%m-%d %H:%M:%S.%f |%l | %s-%!:%# - %v");
    // 设置日志级别
    loggerPtr->set_level(spdlog::level::from_str(conf.level));
    // 设置刷新日志的日志级别，当出现level或更高级别日志时，立刻刷新日志到 disk
    loggerPtr->flush_on(spdlog::level::from_str(conf.level));
  }
}

/*
 * trace 0
 * debug 1
 * info 2
 * warn 3
 * error 4
 * critical 5
 * off 6 (not use)
 */
std::string Logger::GetLogLevel() {
  auto level = loggerPtr->level();
  return spdlog::level::to_string_view(level).data();
}

void Logger::SetLogLevel(const std::string &log_level) {
  auto level = spdlog::level::from_str(log_level);
  if (level == spdlog::level::off) {
    LOG_WARN("Given invalid log level {}", log_level);
  } else {
    loggerPtr->set_level(level);
    loggerPtr->flush_on(level);
  }
}
