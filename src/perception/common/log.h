#ifndef PERCEPTION_LOG_H
#define PERCEPTION_LOG_H
#define SPDLOG_NAME "perception"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define DEBUG(...)                                                  \
    SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__); \
    SPDLOG_LOGGER_DEBUG(spdlog::get(SPDLOG_NAME), __VA_ARGS__)
#define INFO(...)                                                  \
    SPDLOG_LOGGER_INFO(spdlog::default_logger_raw(), __VA_ARGS__); \
    SPDLOG_LOGGER_INFO(spdlog::get(SPDLOG_NAME), __VA_ARGS__)
#define WARN(...)                                                  \
    SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__); \
    SPDLOG_LOGGER_WARN(spdlog::get(SPDLOG_NAME), __VA_ARGS__)
#define ERROR(...)                                                  \
    SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__); \
    SPDLOG_LOGGER_ERROR(spdlog::get(SPDLOG_NAME), __VA_ARGS__)

#include <cstdio>
#include <chrono>
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"  // support for loading levels from the environment variable
#include "spdlog/fmt/ostr.h" // support for user defined types
#include "spdlog/sinks/rotating_file_sink.h"

// static auto rotating_logger = spdlog::rotating_logger_mt(SPDLOG_NAME, "logs/rotating.txt", 1048576 * 5, 3);
#endif