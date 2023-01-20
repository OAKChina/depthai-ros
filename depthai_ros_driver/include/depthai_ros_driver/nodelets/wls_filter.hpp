#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "opencv2/ximgproc/disparity_filter.hpp"

namespace depthai_ros_driver {
namespace nodelets {
class WLSFilter : public nodelet::Nodelet {
   public:
    void onInit() override;

    void wlsCB(const sensor_msgs::ImageConstPtr& disp, const sensor_msgs::CameraInfoConstPtr& disp_info, const sensor_msgs::ImageConstPtr& rightImg);

    message_filters::Subscriber<sensor_msgs::Image> disparityImgSub;
    message_filters::Subscriber<sensor_msgs::Image> rightImgSub;

    message_filters::Subscriber<sensor_msgs::CameraInfo> disparityInfoSub;
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image>> sync;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter;
};
}  // namespace nodelets
}  // namespace depthai_ros_driver