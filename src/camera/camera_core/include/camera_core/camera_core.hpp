#pragma once

#include "util.hpp"
#include <hikcamera/image_capturer.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace camera {

struct Params {
  std::string camera_name;
  std::string cam_info_url;
  std::atomic<cv::RotateFlags> rotate_flags;
  hikcamera::ImageCapturer::CameraProfile camera_profile;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle;

  explicit Params(rclcpp::Node *node) {
    camera_name = node->declare_parameter("camera.name", "camera");
    cam_info_url = node->declare_parameter(
        "camera.cam_info_url", "package://camera_core/config/camera_info.yaml");
    float exp_us = node->declare_parameter("camera.exposure_time", 5000.0f);
    camera_profile.exposure_time =
        std::chrono::duration<float, std::micro>(exp_us);
    camera_profile.gain = node->declare_parameter("camera.gain", 8.0f);
    camera_profile.invert_image =
        node->declare_parameter("camera.invert", false);
    camera_profile.trigger_mode =
        node->declare_parameter("camera.trigger_mode", false);
    int rotate = node->declare_parameter("camera.rotate", 0);
    bool is_valid;
    auto flags = at::getRotateFlags(rotate, is_valid);
    if (is_valid) {
      rotate_flags = flags;
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "Input rotate angle is invalid, refuse to setup the params");
    }
  }
};

struct Publishers {
  image_transport::Publisher image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub;

  explicit Publishers(rclcpp::Node * node) {
    image_pub = image_transport::create_publisher(node, "/image_raw");
    cam_info_pub =
        node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);
  }
  ~Publishers() {
    image_pub.shutdown();
  }
};

struct Subscribers {
  explicit Subscribers(rclcpp::Node * /*node*/) {
    // Add subscribers here if needed
  }
};

class CameraCore final : public rclcpp::Node,
                         public std::enable_shared_from_this<CameraCore> {
public:
  explicit CameraCore(const rclcpp::NodeOptions &options);

private:
  void captureAndPub();
  rcl_interfaces::msg::SetParametersResult
  paramCallback(const std::vector<rclcpp::Parameter> &params);

  Params params_;
  Publishers pubs_;
  Subscribers subs_;
  std::unique_ptr<hikcamera::ImageCapturer> camera_;
  std::thread capture_thread;
};

} // namespace camera
