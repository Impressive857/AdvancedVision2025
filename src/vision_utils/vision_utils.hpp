#ifndef _VISION_UTILS_CPP_
#define _VISION_UTILS_CPP_

// std
#include <cstdio>

// ros
#include <rclcpp/rclcpp.hpp>
#include "ros_msg/msg/log.hpp"

enum class LogLevel : int32_t {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

static ros_msg::msg::Log create_log(const rclcpp::Time& time, LogLevel level, const std::string& text) {
    ros_msg::msg::Log log;
    log.header.stamp = time;
    log.level = static_cast<int32_t>(level);
    log.text = text;
    return log;
}

template <typename... _Args>
static ros_msg::msg::Log create_log(const rclcpp::Time& time, LogLevel level, const std::string& format, _Args...args) {
    ros_msg::msg::Log log;
    log.header.stamp = time;
    log.level = static_cast<int32_t>(level);
    int len = snprintf(nullptr, 0, format.c_str(), args...);
    if (len <= 0) {
        log.text = "failed format!";
        log.level = static_cast<int32_t>(LogLevel::ERROR);
        return log;
    }
    std::string text(len + 1, '\0');
    int res = snprintf(&text[0], len + 1, format.c_str(), args...);
    if (res < 0 || res >= static_cast<int>(text.size())) {
        log.text = "failed format!";
        log.level = static_cast<int32_t>(LogLevel::ERROR);
        return log;
    }
    text.resize(res);
    log.text = text;
    return log;
}

#endif // !_VISION_UTILS_CPP_