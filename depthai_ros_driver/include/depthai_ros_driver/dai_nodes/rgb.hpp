#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class RGBLinkType { color, preview };
};
class RGB : public BaseNode {
   public:
    explicit RGB(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~RGB() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    void colorQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher rgbPub, previewPub;
    sensor_msgs::msg::CameraInfo rgbInfo, previewInfo;
    std::shared_ptr<dai::node::ColorCamera> colorCamNode;
    std::unique_ptr<param_handlers::RGBParamHandler> paramHandler;
    std::shared_ptr<dai::DataOutputQueue> colorQ, previewQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutColor, xoutPreview;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string colorQName, previewQName, controlQName;
};
class RGBFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<RGB>(daiNodeName, node, pipeline);
    };
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver