#include <chrono>
#include <depthai_bridge/depthaiUtility.hpp>


namespace dai {

namespace ros {

#ifdef IS_ROS2
rclcpp::Time getFrameTime(rclcpp::Time rclBaseTime,
                          std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                          std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    // uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto rclStamp = rclBaseTime + elapsedTime;
    // DEPTHAI_ROS_DEBUG_STREAM("PRINT TIMESTAMP: ", "rosStamp -> " << rclStamp << "  rosBaseTime -> " << rclBaseTime);
    return rclStamp;
}

#else

::ros::Time getFrameTime(::ros::Time rosBaseTime,
                         std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime,
                         std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> currTimePoint) {
    auto elapsedTime = currTimePoint - steadyBaseTime;
    uint64_t nSec = rosBaseTime.toNSec() + std::chrono::duration_cast<std::chrono::nanoseconds>(elapsedTime).count();
    auto currTime = rosBaseTime;
    auto rosStamp = currTime.fromNSec(nSec);
    DEPTHAI_ROS_DEBUG_STREAM("PRINT TIMESTAMP: ", "rosStamp -> " << rosStamp << "  rosBaseTime -> " << rosBaseTime);
    return rosStamp;
}

#endif

}  // namespace ros
}  // namespace dai