#include "depthai_ros_driver/nodelets/wls_filter.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "dynamic_reconfigure/server.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depthai_ros_driver {
namespace nodelets {
void WLSFilter::onInit() {
    auto pNH = getPrivateNodeHandle();
    disparityImgSub.subscribe(pNH, "stereo/disparity", 1);
    rightImgSub.subscribe(pNH, "stereo/right_rect", 1);
    disparityInfoSub.subscribe(pNH, "stereo/camera_info", 1);
    sync = std::make_unique<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image>>(
        disparityImgSub, disparityInfoSub, rightImgSub, 3);
    sync->registerCallback(std::bind(&WLSFilter::wlsCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    filter->setLambda(8000);
    filter->setSigmaColor(1.5);
}
void WLSFilter::wlsCB(const sensor_msgs::ImageConstPtr& disp, const sensor_msgs::CameraInfoConstPtr& disp_info, const sensor_msgs::ImageConstPtr& rightImg) {
    cv::Mat dispFrame, rightFrame, dispFiltered;
    cv_bridge::CvImagePtr cv_ptr;
        try
    {
      cv_ptr = cv_bridge::toCvCopy(disp, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    auto factor = disp_info->K[0] * disp_info->D[4];
    filter->filter(dispFrame, rightFrame, dispFiltered);
}
}  // namespace nodelets
}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::nodelets::WLSFilter, nodelet::Nodelet)
