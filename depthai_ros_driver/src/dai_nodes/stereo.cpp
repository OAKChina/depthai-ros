#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();
    left = std::make_unique<CameraSensor>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT, false);
    right = std::make_unique<CameraSensor>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT, false);

    ph = std::make_unique<param_handlers::StereoParamHandler>(daiNodeName);
    ph->declareParams(node, stereoCamNode);
    setXinXout(pipeline);
    left->link(stereoCamNode->left);
    right->link(stereoCamNode->right);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
    rightRectQName = getName() + "_right_rect";
    leftRectQName = getName() + "_left_rect";
}

void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutStereo = pipeline->create<dai::node::XLinkOut>();
    xoutStereo->setStreamName(stereoQName);
    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
        stereoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>(getROSNode(), "i_low_bandwidth_quality"));
        stereoCamNode->disparity.link(stereoEnc->input);
        stereoEnc->bitstream.link(xoutStereo->input);
    } else {
        if(ph->getParam<bool>(getROSNode(), "i_output_disparity")) {
            stereoCamNode->disparity.link(xoutStereo->input);
        } else {
            stereoCamNode->depth.link(xoutStereo->input);
        }
    }
    if(ph->getParam<bool>(getROSNode(), "i_publish_rect_left")) {
        xoutLeftRect = pipeline->create<dai::node::XLinkOut>();
        xoutLeftRect->setStreamName(leftRectQName);
        if(ph->getParam<bool>(getROSNode(), "i_rect_left_low_bandwidth")) {
            leftRectEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>(getROSNode(), "i_rect_left_low_bandwidth_quality"));
            stereoCamNode->rectifiedLeft.link(leftRectEnc->input);
            leftRectEnc->bitstream.link(xoutLeftRect->input);
        } else {
            stereoCamNode->rectifiedLeft.link(xoutLeftRect->input);
        }
    }
    if(ph->getParam<bool>(getROSNode(), "i_publish_rect_right")) {
        xoutRightRect = pipeline->create<dai::node::XLinkOut>();
        xoutRightRect->setStreamName(rightRectQName);
        if(ph->getParam<bool>(getROSNode(), "i_rect_right_low_bandwidth")) {
            rightRectEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>(getROSNode(), "i_rect_right_low_bandwidth_quality"));
            stereoCamNode->rectifiedRight.link(rightRectEnc->input);
            rightRectEnc->bitstream.link(xoutRightRect->input);
        } else {
            stereoCamNode->rectifiedRight.link(xoutRightRect->input);
        }
    }
}

void Stereo::setupRectifiedLeftQueue(std::shared_ptr<dai::Device> device) {
    leftRectQ = device->getOutputQueue(leftRectQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto leftTFPrefix = getTFPrefix("left");
    leftImageConverter = std::make_unique<dai::ros::ImageConverter>(leftTFPrefix + "_camera_optical_frame", false);
    stereoPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/left/rect");
    leftRectInfo = sensor_helpers::getCalibInfo(getROSNode(), *leftImageConverter, device, dai::CameraBoardSocket::LEFT);
    if(ph->getParam<bool>(getROSNode(), "i_rect_left_low_bandwidth")) {
        leftRectQ->addCallback(std::bind(&sensor_helpers::compressedImgCB,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         *leftImageConverter,
                                         leftRectPub,
                                         leftRectInfo,
                                         dai::RawImgFrame::Type::GRAY8));
    } else {
        leftRectQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *leftImageConverter, leftRectPub, leftRectInfo));
    }
}
void Stereo::setupRectifiedRightQueue(std::shared_ptr<dai::Device> device) {
    rightRectQ = device->getOutputQueue(rightRectQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto rightTFPrefix = getTFPrefix("right");
    rightImageConverter = std::make_unique<dai::ros::ImageConverter>(rightTFPrefix + "_camera_optical_frame", false);
    stereoPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/right/rect");
    rightRectInfo = sensor_helpers::getCalibInfo(getROSNode(), *rightImageConverter, device, dai::CameraBoardSocket::RIGHT);
    if(ph->getParam<bool>(getROSNode(), "i_rect_right_low_bandwidth")) {
        rightRectQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          *rightImageConverter,
                                          rightRectPub,
                                          rightRectInfo,
                                          dai::RawImgFrame::Type::GRAY8));
    } else {
        rightRectQ->addCallback(
            std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *rightImageConverter, rightRectPub, rightRectInfo));
    }
}
void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device) {
    stereoQ = device->getOutputQueue(stereoQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    std::string tfPrefix;
    if(ph->getParam<bool>(getROSNode(), "i_align_depth")) {
        tfPrefix = getTFPrefix("rgb");
    } else {
        tfPrefix = getTFPrefix("right");
    }
    stereoImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
    stereoInfo = sensor_helpers::getCalibInfo(getROSNode(),
                                              *stereoImageConverter,
                                              device,
                                              static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                              ph->getParam<int>(getROSNode(), "i_width"),
                                              ph->getParam<int>(getROSNode(), "i_height"));
    auto calibHandler = device->readCalibration();
    stereoInfo.p[3] = calibHandler.getBaselineDistance() * 10.0;  // baseline in mm
    stereoPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
        if(ph->getParam<bool>(getROSNode(), "i_output_disparity")) {
            stereoQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                           std::placeholders::_1,
                                           std::placeholders::_2,
                                           *stereoImageConverter,
                                           stereoPub,
                                           stereoInfo,
                                           dai::RawImgFrame::Type::GRAY8));
        } else {
            // converting disp->depth
            stereoQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                           std::placeholders::_1,
                                           std::placeholders::_2,
                                           *stereoImageConverter,
                                           stereoPub,
                                           stereoInfo,
                                           dai::RawImgFrame::Type::RAW8));
        }
    } else {
        stereoQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *stereoImageConverter, stereoPub, stereoInfo));
    }
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    left->setupQueues(device);
    right->setupQueues(device);
    setupStereoQueue(device);
    if(ph->getParam<bool>(getROSNode(), "i_publish_rect_left")) {
        setupRectifiedLeftQueue(device);
    }
    if(ph->getParam<bool>(getROSNode(), "i_publish_rect_right")) {
        setupRectifiedRightQueue(device);
    }
    left->setupQueues(device);
    right->setupQueues(device);
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    stereoQ->close();
}

void Stereo::link(const dai::Node::Input& in, int /*linkType*/) {
    stereoCamNode->depth.link(in);
}

dai::Node::Input Stereo::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
        return stereoCamNode->left;
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
        return stereoCamNode->right;
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}

void Stereo::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
