#include "rm_serial_driver/rm_serial_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <auto_aim_interfaces/msg/detail/tracker_info__struct.hpp>
#include <exception>
#include <mutex>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#include "rm_serial_driver/packet.hpp"
#include "util/mathit.hpp"
#include "util/serial_parser.hpp"

namespace rm_serial_driver
{

// Params implementation
template <typename T>
static T declare(
  rclcpp::Node * node, const std::string & name, const T & default_val,
  const rcl_interfaces::msg::ParameterDescriptor & desc =
    rcl_interfaces::msg::ParameterDescriptor())
{
  return node->declare_parameter(name, default_val, desc);
}

Params::Params(rclcpp::Node * node)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.set__read_only(true);
  config.device_name =
    declare(node, "device_name", std::string("/dev/ttyACM0"), desc);
  config.baudrate = declare(node, "baud_rate", 115200, desc);
  config.hardware_flow = declare(node, "hardware_flow", false, desc);
  config.parity = declare(node, "parity", false, desc);
  config.stop_bits = declare(node, "stop_bits", 1, desc);
  config.timeout_ms = declare(node, "timeout_ms", 1000, desc);

  debug = declare(node, "debug", false);
}

// Publishers implementation
Publishers::Publishers(rclcpp::Node * node) : tf_broadcaster(node)
{
  marker_p =
    node->create_publisher<visualization_msgs::msg::Marker>("aiming_point", 10);
}

// Subscribers implementation
Subscribers::Subscribers(rclcpp::Node * node, RMSerialDriver * parent)
{
  cb_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  target_sub =
    node->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::handleMsg, parent, std::placeholders::_1),
      options);
}

// Clients implementation
Clients::Clients(rclcpp::Node * node) : node_ptr(node)
{
  detector_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_ptr, "armor_detector");
  reset_tracker_srv =
    node_ptr->create_client<std_srvs::srv::Trigger>("/reset_tracker");
}

void Clients::setParam(const rclcpp::Parameter & param)
{
  setParamInternal(param, detector_client, "detect_color");
}

void Clients::setParamInternal(
  const rclcpp::Parameter & param,
  const rclcpp::AsyncParametersClient::SharedPtr & client,
  const std::string & name, bool invert)
{
  if (!client || !client->service_is_ready()) {
    RCLCPP_WARN(
      node_ptr->get_logger(), "%s service not ready, skipping", name.c_str());
    return;
  }

  if (!param_mutex.try_lock()) {
    RCLCPP_DEBUG(node_ptr->get_logger(), "%s param busy, skip", name.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(param_mutex, std::adopt_lock);

  int value = invert ? (1 - param.as_int()) : param.as_int();
  rclcpp::Parameter new_param(param.get_name(), value);

  RCLCPP_INFO(node_ptr->get_logger(), "Setting %s to %d", name.c_str(), value);

  auto sync_future = client->set_parameters({new_param});
  if (
    sync_future.wait_for(std::chrono::milliseconds(100)) ==
    std::future_status::ready) {
    try {
      auto results = sync_future.get();
      for (const auto & res : results) {
        if (!res.successful) {
          RCLCPP_ERROR(
            node_ptr->get_logger(), "%s set failed: %s", name.c_str(),
            res.reason.c_str());
          return;
        }
      }
      RCLCPP_INFO(
        node_ptr->get_logger(), "%s set to %d succeeded", name.c_str(), value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_ptr->get_logger(), "Exception while setting %s: %s", name.c_str(),
        e.what());
    }
  } else {
    RCLCPP_WARN(node_ptr->get_logger(), "%s set timeout", name.c_str());
  }
}

// RMSerialDriver implementation
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & opts)
: Node("rm_serial_driver", opts),
  prev_color_(0),
  params_(this),
  pubs_(this),
  subs_(this, this),
  clis_(this)
{
  serial_parser_ = std::make_unique<SerialParser>(this->get_logger());
  serial_parser_->registerHandler<ReceiveImuData>(
    0x01, [this](auto & pkt) { handlePacket(pkt); });
  serial_parser_->registerHandler<ReceiveTargetInfoData>(
    0x09, [this](auto & pkt) { handlePacket(pkt); });
  port_ = std::make_unique<SerialPort>(params_.config);
  this->openPortWithRetry();
  params_.param_cb_handle = add_on_set_parameters_callback(
    (std::bind(&RMSerialDriver::paramCallback, this, std::placeholders::_1)));
  thread_ = std::thread([this] { receiveLoop(); });
}

// === 新增: 打开串口并自动重连 ===
void RMSerialDriver::openPortWithRetry()
{
  while (rclcpp::ok()) {
    const char * who = params_.config.device_name.c_str();
    try {
      port_->open();
      RCLCPP_INFO(get_logger(), "%s serial port opened", who);
      return;  // 成功 → 直接返回
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "serial %s open failed: %s; retry in 1 s …", who,
        e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

RMSerialDriver::~RMSerialDriver()
{
  rclcpp::shutdown();
  if (thread_.joinable()) thread_.join();
}

void RMSerialDriver::receiveLoop()
{
  std::string str = params_.config.device_name;

  while (rclcpp::ok()) {
    ssize_t n = port_->read(data_buf_, BUFFER_SIZE);
    if (n > 0) {
      serial_parser_->feed(data_buf_, static_cast<size_t>(n));
    } else if (n == 0) {
      // EOF: 对端关闭，设备可能掉了
      RCLCPP_WARN(
        get_logger(), "%s port returned 0 (EOF), 尝试重连…", str.c_str());
      port_->close();
      openPortWithRetry();
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // 非阻塞模式下没数据，跳过
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s port read error (%s), 尝试重连…", str.c_str(),
        std::strerror(errno));
      port_->close();
      openPortWithRetry();
    }
  }
}

void RMSerialDriver::handlePacket(const ReceiveImuData & pkt)
{
  auto d = pkt.data;
  tf2::Quaternion q;
  q.setRPY(d.roll, d.pitch, d.yaw);
  q.normalize();
  if (
    std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) ||
    std::isnan(q.w())) {
    RCLCPP_WARN(get_logger(), "roll, pitch or yaw is invalid nan");
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = "odom";
  t.child_frame_id = "gimbal_link";
  t.transform.rotation = tf2::toMsg(q);
  pubs_.tf_broadcaster.sendTransform(t);

  static std::atomic<uint8_t> aiming_color = UNKNOWN;
  uint8_t detect_color = d.self_color == RED ? BLUE : RED;
  if (detect_color != RED && detect_color != BLUE) [[unlikely]] {
    RCLCPP_ERROR(
      get_logger(),
      "invalid color value: %d, expected color value is 0 - RED, 1 - BLUE",
      detect_color);
    return;
  }
  if (detect_color != aiming_color || aiming_color == UNKNOWN) [[unlikely]] {
    clis_.setParam(rclcpp::Parameter("detect_color", detect_color));
    aiming_color.store(detect_color);
  }
}

void RMSerialDriver::handleMsg(
  auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  static std::map<std::string, int> name_to_id = {
    {"", 0},  {"1", 1},       {"2", 2},     {"3", 3},    {"4", 4},
    {"5", 5}, {"outpost", 6}, {"guard", 7}, {"base", 8},
  };

  bool is_valid = msg->header.frame_id == "odom";
  if (!is_valid) {
    RCLCPP_WARN(
      get_logger(), "invalid target frame id : %s",
      msg->header.frame_id.c_str());
    return;
  }

  // ========== 准备共用的基础数据 ==========
  SendVisionData pkt{};
  pkt.frame_header.sof = 0x5A;
  pkt.frame_header.id = 0x03;

  auto & d = pkt.data;
  d.tracking = msg->tracking;
  d.armors_num = msg->armors_num;
  d.id = name_to_id[msg->id];
  d.yaw = msg->yaw;
  d.x = msg->position.x;
  d.y = msg->position.y;
  d.z = msg->position.z;
  d.vx = msg->velocity.x;
  d.vy = msg->velocity.y;
  d.vz = msg->velocity.z;
  d.v_yaw = msg->v_yaw;
  d.r1 = msg->radius_1;
  d.r2 = msg->radius_2;
  d.dz = msg->dz;

  pkt.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();

  auto buf = pack(pkt);
  if (params_.debug) {
    RCLCPP_INFO(
      get_logger(), "Port debug info");
    print(pkt);
  }

  if (static_cast<ssize_t>(port_->write(buf.data(), buf.size())) < 0) {
    RCLCPP_WARN(get_logger(), "Failed to send vision data to port");
  }
}

void RMSerialDriver::handlePacket(
  const ReceiveTargetInfoData & pkt)
{
  auto d = pkt.data;
  std::array<float, 3> point = {d.aim_x, d.aim_y, d.aim_z};
  auto distance = norm(point);

  if (distance > 0.01 && params_.debug) {
    visualization_msgs::msg::Marker aiming_point;
    aiming_point.header.frame_id = "gimbal_link";
    aiming_point.header.stamp = this->now();
    aiming_point.lifetime = rclcpp::Duration::from_seconds(1);
    aiming_point.pose.position.x = d.aim_x;
    aiming_point.pose.position.x = d.aim_y;
    aiming_point.pose.position.x = d.aim_z;
    pubs_.marker_p->publish(aiming_point);
  }
  static std::atomic<uint8_t> aiming_color = UNKNOWN;
  if (d.detect_color != RED && d.detect_color != BLUE) [[unlikely]] {
    RCLCPP_ERROR(
      get_logger(),
      "invalid color value: %d, expected color value is 0 - RED, 1 - BLUE",
      d.detect_color);
    return;
  }
  if (d.detect_color != aiming_color || aiming_color == UNKNOWN) [[unlikely]] {
    clis_.setParam(rclcpp::Parameter("detect_color", d.detect_color));
    aiming_color.store(d.detect_color);
  }
}

rcl_interfaces::msg::SetParametersResult RMSerialDriver::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    if (param.get_name() == "debug") {
      params_.debug = param.as_bool();
      RCLCPP_INFO(
        this->get_logger(), "Updated debug: %s",
        param.as_bool() ? "true" : "false");
    } else {
      result.successful = false;
      result.reason = "Unsupported parameter update: " + param.get_name();
    }
  }

  return result;
}

}  // namespace rm_serial_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)