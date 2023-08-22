#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <iostream>
#include <string>

// Define log levels
enum class LogLevel { INFO, WARN, ERROR, DEBUG };

// Current log level setting
constexpr LogLevel currentLogLevel = LogLevel::DEBUG;

class Logger {
 public:
  // Templated log function to allow various input types
  template <typename T>
  static void log(T message, LogLevel level) {
    if (level >= currentLogLevel) {
      switch (level) {
        case LogLevel::INFO:
          std::cout << "[INFO]: " << message << std::endl;
          break;
        case LogLevel::WARN:
          std::cout << "[WARN]: " << message << std::endl;
          break;
        case LogLevel::ERROR:
          std::cout << "[ERROR]: " << message << std::endl;
          break;
        case LogLevel::DEBUG:
          std::cout << "[DEBUG]: " << message << std::endl;
          break;
      }
    }
  }
};

// Macros for easy logging
#define LOG_INFO(msg) Logger::log(msg, LogLevel::INFO)
#define LOG_WARN(msg) Logger::log(msg, LogLevel::WARN)
#define LOG_ERROR(msg) Logger::log(msg, LogLevel::ERROR)
#define LOG_DEBUG(msg) Logger::log(msg, LogLevel::DEBUG)

#endif  // DEBUG_HPP
