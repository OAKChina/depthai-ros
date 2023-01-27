#pragma once

#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nodelet/nodelet.h"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "depthai_ros_driver/wlsConfig.h"
#include "dynamic_reconfigure/server.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

namespace depthai_ros_driver {
namespace nodelets {
class WLSFilter : public nodelet::Nodelet {
   public:
    void onInit() override;

    void wlsCB(const sensor_msgs::ImageConstPtr& disp, const sensor_msgs::CameraInfoConstPtr& disp_info, const sensor_msgs::ImageConstPtr& rightImg);

    message_filters::Subscriber<sensor_msgs::Image> disparityImgSub;
    message_filters::Subscriber<sensor_msgs::Image> rightImgSub;
    std::shared_ptr<dynamic_reconfigure::Server<wlsConfig>> paramServer;
    void parameterCB(wlsConfig& config, uint32_t level);

    message_filters::Subscriber<sensor_msgs::CameraInfo> disparityInfoSub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter;
    image_transport::CameraPublisher depthPub;
    std::shared_ptr<image_transport::ImageTransport> it;
};
}  // namespace nodelets
}  // namespace depthai_ros_driver