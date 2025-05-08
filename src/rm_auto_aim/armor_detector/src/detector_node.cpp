// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(get_logger(), "Starting ArmorDetectorNode...");

  // Detector
  detector_ = initDetector();
  debug_ = declare_parameter<bool>("debug", true);
  RCLCPP_INFO(get_logger(), "Detector initialized.");

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());
  RCLCPP_INFO(get_logger(), "Armors publisher created.");

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });
  RCLCPP_INFO(get_logger(), "Camera info subscription created.");
  img_sub_ = image_transport::create_subscription(
    this, "/image_raw", std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1),
    "raw");
  RCLCPP_INFO(get_logger(), "img subscription created.");
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArmorDetectorNode::paramCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Param callback function set up.");
  
  if (debug_) {
    dbg_pubs_ = std::make_unique<DebugPublishers>(this);
    RCLCPP_INFO(get_logger(), "Debug publisher created.");
  }
}

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::paramCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  auto & d = *detector_;
  auto & l = detector_->l;
  auto & a = detector_->a;
  for (const auto & p : params) {
    const auto & name = p.get_name();
    try {
      if (name == "binary_thres") {
        d.binary_thres = p.as_int();
      } else if (name == "detect_color") {
        d.detect_color = p.as_int();
      } else if (name == "light.min_ratio") {
        l.min_ratio = p.as_double();
      } else if (name == "light.max_ratio") {
        l.max_ratio = p.as_double();
      } else if (name == "light.max_angle") {
        l.max_angle = p.as_double();
      } else if (name == "armor.min_light_ratio") {
        a.min_light_ratio = p.as_double();
      } else if (name == "armor.min_small_center_distance") {
        a.min_small_center_distance = p.as_double();
      } else if (name == "armor.max_small_center_distance") {
        a.max_small_center_distance = p.as_double();
      } else if (name == "armor.min_large_center_distance") {
        a.min_large_center_distance = p.as_double();
      } else if (name == "armor.max_large_center_distance") {
        a.max_large_center_distance = p.as_double();
      } else if (name == "armor.max_angle") {
        a.max_angle = p.as_double();
      } else if (name == "classifier_threshold") {
        d.classifier->threshold = p.as_double();
      } else if (name == "debug") {
        debug_ = p.as_bool();
      }
      else {
        continue;
      }
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = std::string("Parameter update failed: ") + e.what();
      return result;
    }
  }
  return result;
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  auto armors = detectArmors(img_msg);
  if (pnp_solver_ != nullptr) {
    armors_msg_.header = img_msg->header;
    armors_msg_.armors.clear();

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success) {
        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);
        armors_msg_.armors.emplace_back(std::move(armor_msg));
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // Publishing detected armors
    armors_msg_.header.stamp = this->now();
    armors_pub_->publish(armors_msg_);
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  std::unique_ptr<Detector> d;
  rcl_interfaces::msg::ParameterDescriptor pd;
  pd.integer_range.resize(1);
  pd.integer_range[0].step = 1;
  pd.integer_range[0].from_value = 0;
  pd.integer_range[0].to_value = 255;

  int binary_thres = declare_parameter("binary_thres", 160, pd);

  pd.description = "0-RED, 1-BLUE";
  pd.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, pd);

  Detector::LightParams l_params{
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params{
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  std::string pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  std::string model_path = pkg_path + "/model/mlp.onnx";
  std::string label_path = pkg_path + "/model/label.txt";
  double threshold = declare_parameter("classifier_threshold", 0.7);
  auto ignore_classes = declare_parameter("ignore_classes", std::vector<std::string>{"negative"});

  d = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);
  d->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);
  return d;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

  auto armors = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info
  if (debug_) {
    dbg_pubs_->binary.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    dbg_pubs_->debug_lights->publish(detector_->debug_lights);
    dbg_pubs_->debug_armors->publish(detector_->debug_armors);

    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      dbg_pubs_->numbers.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    detector_->drawResults(img);
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    dbg_pubs_->result.publish(cv_bridge::CvImage(img_msg->header, "bgr8", img).toImageMsg());
  }

  return armors;
}
DebugPublishers::DebugPublishers(rclcpp::Node * node_ptr)
{
  this->debug_lights =
    node_ptr->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  this->debug_armors =
    node_ptr->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);
  this->binary = image_transport::create_publisher(node_ptr, "/detector/binary_img");
  this->numbers = image_transport::create_publisher(node_ptr, "/detector/number_img");
  this->result = image_transport::create_publisher(node_ptr, "/detector/result_img");
}
DebugPublishers::~DebugPublishers()
{
  binary.shutdown();
  numbers.shutdown();
  result.shutdown();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
