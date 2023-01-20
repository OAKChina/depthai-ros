#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device)
    : BaseNode(daiNodeName, node, pipeline), it(node) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();
    left = std::make_unique<CameraSensor>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT, false);
    right = std::make_unique<CameraSensor>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT, false);

    ph = std::make_unique<param_handlers::StereoParamHandler>(daiNodeName);
    ph->declareParams(node, stereoCamNode);
    setXinXout(pipeline);
    left->link(stereoCamNode->left);
    right->link(stereoCamNode->right);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
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
        stereoEnc = pipeline->create<dai::node::VideoEncoder>();
        stereoEnc->setQuality(ph->getParam<int>(getROSNode(), "i_low_bandwidth_quality"));
        stereoEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
        stereoCamNode->disparity.link(stereoEnc->input);
        stereoEnc->bitstream.link(xoutStereo->input);
    } else {
        if(ph->getParam<bool>(getROSNode(), "i_output_disparity")) {
            stereoCamNode->disparity.link(xoutStereo->input);
        } else {
            stereoCamNode->depth.link(xoutStereo->input);
        }
    }
    if(ph->getParam<bool>(getROSNode(), "i_publish_left_rect")) {
        xoutLeftRect = pipeline->create<dai::node::XLinkOut>();
        xoutLeftRect->setStreamName(leftRectQName);
        if(ph->getParam<bool>(getROSNode(), "i_rect_left_low_bandwidth")) {
            leftRectEnc = pipeline->create<dai::node::VideoEncoder>();
            leftRectEnc->setQuality(50);
            leftRectEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
            stereoCamNode->rectifiedLeft.link(leftRectEnc->input);
            leftRectEnc->bitstream.link(xoutLeftRect->input);
        } else {
            stereoCamNode->rectifiedLeft.link(xoutLeftRect->input);
        }
    }
    if(ph->getParam<bool>(getROSNode(), "i_publish_right_rect")) {
        xoutRightRect = pipeline->create<dai::node::XLinkOut>();
        xoutRightRect->setStreamName(rightRectQName);
        if(ph->getParam<bool>(getROSNode(), "i_rect_right_low_bandwidth")) {
            rightRectEnc = pipeline->create<dai::node::VideoEncoder>();
            rightRectEnc->setQuality(50);
            rightRectEnc->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
            stereoCamNode->rectifiedRight.link(rightRectEnc->input);
            rightRectEnc->bitstream.link(xoutLeftRect->input);
        } else {
            stereoCamNode->rectifiedRight.link(xoutLeftRect->input);
        }
    }
}
void Stereo::setupRectifiedLeftQueue(std::shared_ptr<dai::Device> device){
    leftRectQ = device->getOutputQueue(leftRectQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto leftTFPrefix = getTFPrefix("left");
    leftImageConverter = std::make_unique<dai::ros::ImageConverter>(leftTFPrefix + "_camera_optical_frame", false);
    stereoQ->addCallback(std::bind(&Stereo::stereoQCB, this, std::placeholders::_1, std::placeholders::_2));
    leftRectPub = it.advertiseCamera(getName() + "/left_rect", 1);
    auto calibHandler = device->readCalibration();
    try {
        leftRectInfo = leftImageConverter->calibrationToCameraInfo(calibHandler,
                                                             dai::CameraBoardSocket::LEFT);
    } catch(std::runtime_error& e) {
        ROS_ERROR("No calibration! Publishing empty camera_info.");
    }

    
    
}
void Stereo::setupRectifiedRightQueue(std::shared_ptr<dai::Device> device){
rightRectQ = device->getOutputQueue(rightRectQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto rightTFPrefix = getTFPrefix("right");
    rightImageConverter = std::make_unique<dai::ros::ImageConverter>(rightTFPrefix + "_camera_optical_frame", false);
    rightRectQ->addCallback(std::bind(&Stereo::stereoQCB, this, std::placeholders::_1, std::placeholders::_2));
    rightRectPub = it.advertiseCamera(getName() + "/right_rect", 1);
    auto calibHandler = device->readCalibration();

    try {
        rightRectInfo = rightImageConverter->calibrationToCameraInfo(calibHandler,
                                                             dai::CameraBoardSocket::RIGHT);
    } catch(std::runtime_error& e) {
        ROS_ERROR("No calibration! Publishing empty camera_info.");
    }
}

void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device){
    stereoQ = device->getOutputQueue(stereoQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    std::string tfPrefix;
    if(ph->getParam<bool>(getROSNode(), "i_align_depth")) {
        tfPrefix = getTFPrefix("rgb");
    } else {
        tfPrefix = getTFPrefix("right");
    }
    stereoImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
    stereoQ->addCallback(std::bind(&Stereo::stereoQCB, this, std::placeholders::_1, std::placeholders::_2));
    stereoPub = it.advertiseCamera(getName() + "/image_raw", 1);
    auto calibHandler = device->readCalibration();
    try {
        stereoInfo = stereoImageConverter->calibrationToCameraInfo(calibHandler,
                                                             static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                             ph->getParam<int>(getROSNode(), "i_width"),
                                                             ph->getParam<int>(getROSNode(), "i_height"));
        stereoInfo.P[3] = calibHandler.getBaselineDistance() * 10.0;  // baseline in mm
    } catch(std::runtime_error& e) {
        ROS_ERROR("No calibration! Publishing empty camera_info.");
    }
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    left->setupQueues(device);
    right->setupQueues(device);
    setupStereoQueue(device);
    if(ph->getParam<bool>(getROSNode(), "i_publish_left_rect")) {
        setupRectifiedLeftQueue(device);
    }
    if(ph->getParam<bool>(getROSNode(), "i_publish_right_rect")) {
        setupRectifiedRightQueue(device);
    }
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    stereoQ->close();
}
void Stereo::stereoQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::deque<sensor_msgs::Image> deq;
    if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth"))
        stereoImageConverter->toRosMsgFromBitStream(img, deq, dai::RawImgFrame::Type::RAW8, stereoInfo);
    else
        stereoImageConverter->toRosMsg(img, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        stereoInfo.header = currMsg.header;
        stereoPub.publish(currMsg, stereoInfo);
        deq.pop_front();
    }
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

void Stereo::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
