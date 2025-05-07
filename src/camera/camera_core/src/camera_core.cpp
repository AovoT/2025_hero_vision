#include "camera_core/camera_core.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <ratio>
#include <rclcpp/subscription_options.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

namespace camera {

CameraCore::CameraCore(const rclcpp::NodeOptions &options)
    : Node("camera_node", options), params_(this),
      pubs_(this),
      subs_(this) {
  camera_ = std::make_unique<hikcamera::ImageCapturer>(
      params_.camera_profile, params_.camera_name.c_str());

  params_.param_cb_handle = this->add_on_set_parameters_callback(
      std::bind(&CameraCore::paramCallback, this, std::placeholders::_1));

  capture_thread = std::thread([this] {
    while (rclcpp::ok()) {
      captureAndPub();
    }
  });
}

void CameraCore::captureAndPub() {
  auto img =
      camera_->read(std::chrono::duration<unsigned int, std::milli>(20000));
  if (img.empty()) [[unlikely]] {
    RCLCPP_INFO(get_logger(), "Image is empty !");
    return;
  }

  cv::rotate(img, img, params_.rotate_flags);

  std_msgs::msg::Header hdr;
  hdr.frame_id = "camera_optical_frame";
  hdr.stamp = this->now();

  auto msg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
  pubs_.image_pub.publish(msg);
}

rcl_interfaces::msg::SetParametersResult
CameraCore::paramCallback(const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &p : params) {
    if (p.get_name() == "camera.exposure_time") {
      float exp = p.as_double();
      camera_->setExposureTime(std::chrono::duration<float, std::micro>(exp));
    } else if (p.get_name() == "camera.gain") {
      camera_->setGain(p.as_double());
    } else if (p.get_name() == "camera.invert") {
      camera_->setInvertImage(p.as_bool());
    } else if (p.get_name() == "camera.trigger_mode") {
      camera_->setTriggerMode(p.as_bool());
    } else if (p.get_name() == "camera.rotate") {
      bool is_valid;
      auto flags = at::getRotateFlags(p.as_int(), is_valid);
      if (is_valid) {
        params_.rotate_flags = flags;
      } else {
        RCLCPP_WARN(
            get_logger(),
            "Input rotate angle is invalid, refuse to change the params");
        result.successful = false;
        result.reason = "input parameter is invalid";
      }
    }
  }

  return result;
}

} // namespace camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraCore)
