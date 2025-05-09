#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 初始化一个球形 Marker
 *
 * @param frame_id  坐标系 id，例如 "map"
 * @param id        同一 namespace 下的唯一 id
 * @param position  球心位置
 * @param diameter  直径（m）
 * @param color     颜色（RGBA，取值 0–1）
 * @param stamp     时间戳，通常传 this->now()
 * @param lifetime  生存时间，默认 0 表示永久
 * @param ns        namespace，用于 RViz 分组，可选
 * @return 完整配置好的 Marker
 */
inline visualization_msgs::msg::Marker initSphereMarker(
    const std::string & frame_id,
    int32_t             id,
    const geometry_msgs::msg::Point & position,
    double              diameter,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Time &            stamp,
    const rclcpp::Duration &        lifetime = rclcpp::Duration{0, 0},
    const std::string &             ns       = "sphere")
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp    = stamp;

  marker.ns    = ns;
  marker.id    = id;
  marker.type  = visualization_msgs::msg::Marker::SPHERE;
  marker.action= visualization_msgs::msg::Marker::ADD;

  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;  // 单位四元数

  marker.scale.x = marker.scale.y = marker.scale.z = diameter;

  marker.color = color;

  marker.lifetime = lifetime;  // 0 = 永久

  // 可选：使 RViz 在深度测试时总是可见
  // marker.frame_locked = false;
  // marker.mesh_use_embedded_materials = false;

  return marker;
}

#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>

template <typename T>
class LatestQueue
{
public:
  // 写入时只保留最新帧
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!queue.empty()) queue.pop();  // 丢弃旧帧
    queue.push(std::move(value));
    cv.notify_one();
  }

  // 非阻塞取出当前帧（如无则返回 false）
  bool pop(T & value)
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (queue.empty()) return false;
    value = std::move(queue.front());
    queue.pop();
    return true;
  }

  // 阻塞等待一定时间取出最新帧
  bool waitAndPop(T & value, int timeout_ms = 1000)
  {
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] {
          return !queue.empty();
        })) {
      return false;
    }
    value = std::move(queue.front());
    queue.pop();
    return true;
  }

  // 获取是否有新帧（可选）
  bool hasData() const
  {
    std::lock_guard<std::mutex> lock(mtx);
    return !queue.empty();
  }

private:
  std::queue<T> queue;
  mutable std::mutex mtx;
  std::condition_variable cv;
};
