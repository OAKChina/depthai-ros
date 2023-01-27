#pragma once

#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "depthai_ros_driver/cameraConfig.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { left, right };
}
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device);
    virtual ~Stereo() = default;
    void updateParams(cameraConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0);
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void setupRectifiedLeftQueue(std::shared_ptr<dai::Device> device);
    void setupRectifiedRightQueue(std::shared_ptr<dai::Device> device);
    void setupStereoQueue(std::shared_ptr<dai::Device> device);
    std::unique_ptr<dai::ros::ImageConverter> stereoImageConverter, leftImageConverter, rightImageConverter;
    image_transport::ImageTransport it;
    image_transport::CameraPublisher stereoPub, leftRectPub, rightRectPub;
    sensor_msgs::CameraInfo stereoInfo, leftRectInfo, rightRectInfo;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> stereoEnc, leftRectEnc, rightRectEnc;
    std::unique_ptr<BaseNode> left;
    std::unique_ptr<BaseNode> right;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> stereoQ, leftRectQ, rightRectQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo, xoutLeftRect, xoutRightRect;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string stereoQName, leftRectQName, rightRectQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver