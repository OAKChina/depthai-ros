#include "depthai_ros_driver/components/wls_filter.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"

namespace depthai_ros_driver {
namespace components {

WLSFilter::WLSFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node("camera", options) {
    onInit();
}
void WLSFilter::onInit() {
    disparityImgSub.subscribe(this, "/stereo/image_raw");
    rightImgSub.subscribe(this, "/stereo/right/rect");
    disparityInfoSub.subscribe(this, "/stereo/camera_info");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), disparityImgSub, disparityInfoSub, rightImgSub);
    sync->registerCallback(std::bind(&WLSFilter::wlsCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    filter->setLambda(8000);
    filter->setSigmaColor(1.5);
    depthPub = image_transport::create_camera_publisher(this, "~/stereo/wls/filtered");
}
// void WLSFilter::parameterCB(wlsConfig& config, uint32_t level){
//     filter->setLambda(config.wls_r_lambda);
//     filter->setSigmaColor(config.wls_r_sigma_color);
// }

void WLSFilter::wlsCB(const sensor_msgs::msg::Image::ConstSharedPtr& disp,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& disp_info,
                      const sensor_msgs::msg::Image::ConstSharedPtr& rightImg) {
    cv::Mat dispFrame, rightFrame, dispFiltered;
    cv_bridge::CvImagePtr dispPtr, rightPtr;
    int dispMultiplier = 1;
    try {
        if(disp->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            dispPtr = cv_bridge::toCvCopy(disp, sensor_msgs::image_encodings::TYPE_16UC1);
            dispMultiplier = 5;
        } else {
            dispPtr = cv_bridge::toCvCopy(disp, sensor_msgs::image_encodings::MONO8);
        }
        rightPtr = cv_bridge::toCvCopy(rightImg, sensor_msgs::image_encodings::MONO8);
        dispFrame = dispPtr->image;
        rightFrame = rightPtr->image;
    } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    sensor_msgs::msg::CameraInfo depthInfo;
    filter->filter(dispFrame, rightFrame, dispFiltered);
    sensor_msgs::msg::Image depth;
    auto factor = dispMultiplier * (disp_info->k[0] * disp_info->p[3]);
    cv::Mat depthOut = cv::Mat(cv::Size(dispFiltered.cols, dispFiltered.rows), CV_16UC1);
    depthOut.forEach<short>([&dispFiltered, &factor, &disp](short& pixel, const int* position) -> void {
        if(disp->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            auto dispPoint = dispFiltered.at<short>(position);
            if(dispPoint == 0)
                pixel = 0;
            else
                pixel = factor / dispPoint;
        } else {
            auto dispPoint = dispFiltered.at<uint8_t>(position);
            if(dispPoint == 0)
                pixel = 0;
            else
                pixel = factor / dispPoint;
        }
    });
    depthInfo = *disp_info;
    dispFiltered = depthOut.clone();
    cv_bridge::CvImage(disp->header, sensor_msgs::image_encodings::TYPE_16UC1, dispFiltered).toImageMsg(depth);
    depthPub.publish(depth, depthInfo);
}
}  // namespace components
}  // namespace depthai_ros_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ros_driver::components::WLSFilter);