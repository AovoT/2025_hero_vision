#pragma once
#include <opencv2/core.hpp>

namespace at {

inline cv::RotateFlags getRotateFlags(const int rotate, bool & is_valid) {
  cv::RotateFlags ret;
  is_valid = true;
  switch (rotate) {
  case 90:
    ret = cv::RotateFlags::ROTATE_90_CLOCKWISE;
    break;
  case 180:
    ret = cv::RotateFlags::ROTATE_180;
    break;
  case 270:
    ret = cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE;
    break;
  default:
    ret = cv::RotateFlags::ROTATE_180;
    is_valid = false;
    break;
  }
  return ret;
}
} // namespace at