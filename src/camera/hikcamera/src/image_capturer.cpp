// image_capturer.cpp
#include "hikcamera/image_capturer.hpp"
#include "CameraParams.h"
#include <cstring>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace hikcamera {

ImageCapturer::ImageCapturer(const CameraProfile &profile, const char *name)
    : profile_(profile) {
  camera_handle_ = nullptr;
  if (!search_camera(name) || !init_camera(profile_)) {
    throw std::runtime_error("Failed to initialize camera");
  }
  // 预热一帧
  cv::Mat() = read(std::chrono::seconds(1));
}

ImageCapturer::~ImageCapturer() {
  uninit_camera();
  delete[] converted_buffer_;
}

std::tuple<int, int> ImageCapturer::get_width_height() const {
  return {convert_param_.nWidth, convert_param_.nHeight};
}

cv::Mat
ImageCapturer::read(std::chrono::duration<unsigned int, std::milli> timeout) {
  MV_FRAME_OUT frame_info;
  auto ret = MV_CC_GetImageBuffer(camera_handle_, &frame_info, timeout.count());
  if (ret != MV_OK) {
    RCLCPP_ERROR(logger_, "Image timeout: %u", ret);
    throw std::runtime_error("Camera read timeout");
  }
  // 初始化转换缓冲区
  if (!converted_buffer_) {
    if (!is_rgb_pixel_type(frame_info.stFrameInfo.enPixelType))
      throw std::runtime_error("RGB required");
    converted_size_ =
        frame_info.stFrameInfo.nWidth * frame_info.stFrameInfo.nHeight * 3;
    converted_buffer_ = new unsigned char[converted_size_];
    convert_param_.nWidth = frame_info.stFrameInfo.nWidth;
    convert_param_.nHeight = frame_info.stFrameInfo.nHeight;
    convert_param_.nSrcDataLen = frame_info.stFrameInfo.nFrameLen;
    convert_param_.enSrcPixelType = frame_info.stFrameInfo.enPixelType;
    convert_param_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert_param_.pDstBuffer = converted_buffer_;
    convert_param_.nDstBufferSize = converted_size_;
  }
  convert_param_.pSrcData = frame_info.pBufAddr;
  ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
  if (ret != MV_OK) {
    RCLCPP_ERROR(logger_, "Convert fail: %u", ret);
    throw std::runtime_error("Pixel conversion failed");
  }
  cv::Mat img(frame_info.stFrameInfo.nHeight, frame_info.stFrameInfo.nWidth,
              CV_8UC3, convert_param_.pDstBuffer);
  MV_CC_FreeImageBuffer(camera_handle_, &frame_info);
  return img;
}

void ImageCapturer::setExposureTime(
    std::chrono::duration<float, std::micro> e) {
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", e.count());
  profile_.exposure_time = e;
}

void ImageCapturer::setGain(float g) {
  MV_CC_SetFloatValue(camera_handle_, "Gain", g);
  profile_.gain = g;
}

void ImageCapturer::setInvertImage(bool inv) {
  MV_CC_SetBoolValue(camera_handle_, "ReverseX", inv);
  MV_CC_SetBoolValue(camera_handle_, "ReverseY", inv);
  profile_.invert_image = inv;
}

void ImageCapturer::setTriggerMode(bool trig) {
  MV_CC_SetEnumValue(camera_handle_, "TriggerMode",
                     trig ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
  profile_.trigger_mode = trig;
}

MV_CC_DEVICE_INFO *ImageCapturer::search_camera(const char *name) {
  MV_CC_DEVICE_INFO_LIST list{};
  MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
  if (list.nDeviceNum == 0)
    return nullptr;
  if (!name)
    return list.pDeviceInfo[0];
  for (unsigned i = 0; i < list.nDeviceNum; ++i) {
    auto *info = list.pDeviceInfo[i];
    const char *devName =
        (info->nTLayerType == MV_GIGE_DEVICE)
            ? reinterpret_cast<const char *>(
                  info->SpecialInfo.stGigEInfo.chUserDefinedName)
            : reinterpret_cast<const char *>(
                  info->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    if (std::strcmp(devName, name) == 0)
      return info;
  }
  return nullptr;
}

bool ImageCapturer::init_camera(const CameraProfile &p) {
  auto *info = search_camera(nullptr);
  if (!info)
    return false;
  MV_CC_CreateHandleWithoutLog(&camera_handle_, info);
  MV_CC_OpenDevice(camera_handle_);
  MV_CC_SetEnumValue(camera_handle_, "TriggerMode",
                     p.trigger_mode ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
  MV_CC_SetBoolValue(camera_handle_, "ReverseX", p.invert_image);
  MV_CC_SetBoolValue(camera_handle_, "ReverseY", p.invert_image);
  MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", p.exposure_time.count());
  MV_CC_SetFloatValue(camera_handle_, "Gain", p.gain);
  MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
  MV_CC_SetBayerCvtQuality(camera_handle_, 2);
  MV_CC_StartGrabbing(camera_handle_);
  return true;
}

void ImageCapturer::uninit_camera() {
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
  }
}

bool ImageCapturer::is_rgb_pixel_type(MvGvspPixelType t) {
  switch (t) {
  case PixelType_Gvsp_BGR8_Packed:
  case PixelType_Gvsp_YUV422_Packed:
  case PixelType_Gvsp_YUV422_YUYV_Packed:
  case PixelType_Gvsp_BayerGR8:
  case PixelType_Gvsp_BayerRG8:
  case PixelType_Gvsp_BayerGB8:
  case PixelType_Gvsp_BayerBG8:
    return true;
  default:
    return false;
  }
}

} // namespace hikcamera
