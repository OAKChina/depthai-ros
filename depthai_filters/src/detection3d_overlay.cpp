#include "depthai_filters/detection3d_overlay.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

Detection3DOverlay::Detection3DOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("detection3d_overlay", options) {
    onInit();
}
void Detection3DOverlay::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    detSub.subscribe(this, "nn/spatial_detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    sync->registerCallback(std::bind(&Detection3DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    labelMap = this->declare_parameter<std::vector<std::string>>("label_map", defaultLabelMap);
}

void Detection3DOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                                   const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);

    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);
    auto blue = cv::Scalar(255, 0, 0);

    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size.x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size.x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size.y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size.y / 2.0;
        auto labelStr = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        auto confidence = detection.results[0].hypothesis.score;
        cv::putText(previewMat, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(previewMat, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        cv::putText(previewMat, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(previewMat, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);

        std::stringstream depthX;
        depthX << "X: " << detections->detections[0].results[0].pose.pose.position.x << " m";
        cv::putText(previewMat, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream depthY;
        depthY << "Y: " << detections->detections[0].results[0].pose.pose.position.y << " m";
        cv::putText(previewMat, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream depthZ;
        depthZ << "Z: " << detections->detections[0].results[0].pose.pose.position.z << " m";
        cv::putText(previewMat, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
    }
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::Detection3DOverlay);