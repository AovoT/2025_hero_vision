#pragma once

#include <chrono>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <MvCameraControl.h>
#include <tuple>

namespace hikcamera {

class ImageCapturer {
public:
  struct CameraProfile {
    CameraProfile() noexcept {
      using namespace std::chrono_literals;
      trigger_mode = false;
      invert_image = false;
      exposure_time = 5ms;
      gain = 0;
    }
    bool trigger_mode;
    bool invert_image;
    std::chrono::duration<float, std::micro> exposure_time;
    float gain;
  };

  explicit ImageCapturer(const CameraProfile &profile = CameraProfile{},
                         const char *user_defined_name = nullptr);
  ImageCapturer(const ImageCapturer &) = delete;
  ImageCapturer &operator=(const ImageCapturer &) = delete;
  ~ImageCapturer();

  [[nodiscard]] cv::Mat
  read(std::chrono::duration<unsigned int, std::milli> timeout);
  [[nodiscard]] std::tuple<int, int> get_width_height() const;

  // 动态更新接口
  void setExposureTime(std::chrono::duration<float, std::micro> exposure);
  void setGain(float gain);
  void setInvertImage(bool invert);
  void setTriggerMode(bool trigger);
private:
  // 摄像头句柄
  void *camera_handle_ = nullptr;
  // 转换缓冲区
  unsigned int converted_size_ = 0;
  unsigned char *converted_buffer_ = nullptr;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_{};
  // 配置和日志
  CameraProfile profile_;
  rclcpp::Logger logger_ = rclcpp::get_logger("hikcamera");

  MV_CC_DEVICE_INFO *search_camera(const char *user_defined_name);
  bool init_camera(const CameraProfile &profile);
  void uninit_camera();
  static bool is_rgb_pixel_type(MvGvspPixelType type);
};
} // namespace hikcamera
