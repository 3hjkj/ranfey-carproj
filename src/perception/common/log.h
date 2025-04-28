#ifndef PERCEPTION_LOG_H_
#define PERCEPTION_LOG_H_

// ── 1. 在包含任何 <rclcpp/...> 之前先处理自定义宏 ────────────────
#ifdef WARN
  #undef WARN
#endif
#ifdef INFO
  #undef INFO
#endif
#ifdef ERROR
  #undef ERROR
#endif
#ifdef DEBUG
  #undef DEBUG
#endif

#define SPDLOG_NAME "perception"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#define DEBUG(...)                                                  \
  SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__);   \
  SPDLOG_LOGGER_DEBUG(spdlog::get(SPDLOG_NAME), __VA_ARGS__)

#define INFO(...)                                                   \
  SPDLOG_LOGGER_INFO (spdlog::default_logger_raw(), __VA_ARGS__);   \
  SPDLOG_LOGGER_INFO (spdlog::get(SPDLOG_NAME), __VA_ARGS__)

#define WARN(...)                                                   \
  SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__);    \
  SPDLOG_LOGGER_WARN(spdlog::get(SPDLOG_NAME), __VA_ARGS__)

#define ERROR(...)                                                  \
  SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__);   \
  SPDLOG_LOGGER_ERROR(spdlog::get(SPDLOG_NAME), __VA_ARGS__)

// ── 2. spdlog 头放在宏之后即可 ──────────────────────────────────────
#include <chrono>
#include <cstdio>
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/rotating_file_sink.h>

#endif  // PERCEPTION_LOG_H_
