// rm_serial_driver.hpp
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "auto_aim_interfaces/msg/target.hpp"
#include "packet.hpp"
#include "serial_port.hpp"
#include "util/serial_parser.hpp"

enum Color : uint8_t {
  RED = 0,
  BLUE = 1,
  UNKNOWN = 2
};

namespace rm_serial_driver
{

class RMSerialDriver;

struct Params
{
  SerialConfig config;
  std::atomic<bool> debug;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_cb_handle;

  explicit Params(rclcpp::Node * node);
};

struct Publishers
{
  tf2_ros::TransformBroadcaster tf_broadcaster;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_p;

  explicit Publishers(rclcpp::Node * node);
};

struct Subscribers
{
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr
    target_sub;

  rclcpp::CallbackGroup::SharedPtr cb_group;
  tf2_ros::Buffer::SharedPtr tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  explicit Subscribers(rclcpp::Node * node, RMSerialDriver * parent);
};

struct Clients
{
  friend class RMSerialDriver;
  using ResultFuturePtr =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

  explicit Clients(rclcpp::Node * node);
  void setParam(const rclcpp::Parameter & param);

private:
  void setParamInternal(
    const rclcpp::Parameter & param,
    const rclcpp::AsyncParametersClient::SharedPtr & client,
    const std::string & name, bool invert = false);

  rclcpp::Node * node_ptr;
  bool initial_set_param{false};
  std::mutex param_mutex;
  rclcpp::AsyncParametersClient::SharedPtr detector_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv;
};

class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver() override;
  void handleMsg(auto_aim_interfaces::msg::Target::SharedPtr msg);

private:
  static constexpr int BUFFER_SIZE = 128;
  uint8_t prev_color_{0};
  std::thread thread_;

  Params params_;
  Publishers pubs_;
  Subscribers subs_;
  Clients clis_;

  std::shared_ptr<SerialPort> port_;
  std::unique_ptr<SerialParser> serial_parser_;
  uint8_t data_buf_[BUFFER_SIZE];

  void receiveLoop();
  void openPortWithRetry();
  void handlePacket(const ReceiveImuData & pkt);
  void handlePacket(const ReceiveTargetInfoData & pkt);
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace rm_serial_driver