#pragma once

#include <chrono>
#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
#else
    #include <ros/ros.h>
#endif

namespace dai {

namespace ros {

enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

#ifdef IS_ROS2

    #define DEPTHAI_ROS_LOG_STREAM(loggerName, level, isOnce, args)                 \
        switch(level) {                                                             \
            case LogLevel::DEBUG:                                                   \
                if(isOnce) {                                                        \
                    RCLCPP_DEBUG_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
                } else {                                                            \
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(loggerName), args);      \
                }                                                                   \
                break;                                                              \
            case LogLevel::INFO:                                                    \
                if(isOnce) {                                                        \
                    RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
                } else {                                                            \
                    RCLCPP_INFO_STREAM(rclcpp::get_logger(loggerName), args);       \
                }                                                                   \
                break;                                                              \
            case LogLevel::WARN:                                                    \
                if(isOnce) {                                                        \
                    RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger(loggerName), args);  \
                } else {                                                            \
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(loggerName), args);       \
                }                                                                   \
                break;                                                              \
            case LogLevel::ERROR:                                                   \
                if(isOnce) {                                                        \
                    RCLCPP_ERROR_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
                } else {                                                            \
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(loggerName), args);      \
                }                                                                   \
                break;                                                              \
            case LogLevel::FATAL:                                                   \
                if(isOnce) {                                                        \
                    RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger(loggerName), args); \
                } else {                                                            \
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger(loggerName), args);      \
                }                                                                   \
                break;                                                              \
        }

#else
    #define DEPTHAI_ROS_LOG_STREAM(loggerName, level, isOnce, args)                                                                       \
        if(isOnce) {                                                                                                                      \
            ROS_LOG_STREAM_ONCE(static_cast<::ros::console::Level>(level), std::string(ROSCONSOLE_NAME_PREFIX) + "." + loggerName, args); \
        } else {                                                                                                                          \
            ROS_LOG_STREAM(static_cast<::ros::console::Level>(level), std::string(ROSCONSOLE_NAME_PREFIX) + "." + loggerName, args);      \
        }
#endif

// DEBUG stream macros on top of ROS logger
#define DEPTHAI_ROS_DEBUG_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::DEBUG, false, args)

#define DEPTHAI_ROS_DEBUG_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::DEBUG, true, args)

// INFO stream macros on top of ROS logger
#define DEPTHAI_ROS_INFO_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::INFO, false, args)

#define DEPTHAI_ROS_INFO_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::INFO, true, args)

// WARN stream macros on top of ROS logger
#define DEPTHAI_ROS_WARN_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::WARN, false, args)

#define DEPTHAI_ROS_WARN_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::WARN, true, args)

// ERROR stream macros on top of ROS logger
#define DEPTHAI_ROS_ERROR_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::ERROR, false, args)

#define DEPTHAI_ROS_ERROR_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::ERROR, true, args)

// FATAL stream macros on top of ROS logger
#define DEPTHAI_ROS_FATAL_STREAM(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::FATAL, false, args)

#define DEPTHAI_ROS_FATAL_STREAM_ONCE(loggerName, args) DEPTHAI_ROS_LOG_STREAM(loggerName, LogLevel::FATAL, true, args)

#ifdef IS_ROS2
rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                          std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                          std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint);

template <typename T>
static void setRosParameter(std::shared_ptr<rclcpp::Node> node, const char* key, T& val) {
    node->declare_parameter(key, val);
    node->get_parameter(key, val);
}

    #define req_type void
    #define req_get(x) ((*request).x)
    #define rep_get(x) ((*response).x)
    #define set_parameter(a, b) setRosParameter(node, a, b)

#else

template <typename T>
static void getParamWithWarning(::ros::NodeHandle& pnh, std::string key, T& val) {
    bool gotParam = pnh.getParam(key, val);
    if(!gotParam) {
        std::stringstream ss;
        ss << val;
        DEPTHAI_ROS_INFO_STREAM(pnh.getNamespace(), "Could not find param " << key << ". Defaulting to " << ss.str() << std::endl);
    }
}

    #define req_type bool
    #define req_get(x) (request.x)
    #define rep_get(x) (response.x)
    #define set_parameter(a, b) getParamWithWarning(node, a, b)


::ros::Time getFrameTime(::ros::Time rosBaseTime,
                         std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                         std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint);

#endif

template <typename T>
T lerp(const T& a, const T& b, const double t) {
    return a * (1.0 - t) + b * t;
}

template <typename T>
T lerpImu(const T& a, const T& b, const double t) {
    T res;
    res.x = lerp(a.x, b.x, t);
    res.y = lerp(a.y, b.y, t);
    res.z = lerp(a.z, b.z, t);
    return res;
}

}  // namespace ros
}  // namespace dai