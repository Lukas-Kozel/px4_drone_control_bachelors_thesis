// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/CameraImageCaptured.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/camera_image_captured__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_CameraImageCaptured_file_url
{
public:
  explicit Init_CameraImageCaptured_file_url(::mavros_msgs::msg::CameraImageCaptured & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::CameraImageCaptured file_url(::mavros_msgs::msg::CameraImageCaptured::_file_url_type arg)
  {
    msg_.file_url = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

class Init_CameraImageCaptured_capture_result
{
public:
  explicit Init_CameraImageCaptured_capture_result(::mavros_msgs::msg::CameraImageCaptured & msg)
  : msg_(msg)
  {}
  Init_CameraImageCaptured_file_url capture_result(::mavros_msgs::msg::CameraImageCaptured::_capture_result_type arg)
  {
    msg_.capture_result = std::move(arg);
    return Init_CameraImageCaptured_file_url(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

class Init_CameraImageCaptured_image_index
{
public:
  explicit Init_CameraImageCaptured_image_index(::mavros_msgs::msg::CameraImageCaptured & msg)
  : msg_(msg)
  {}
  Init_CameraImageCaptured_capture_result image_index(::mavros_msgs::msg::CameraImageCaptured::_image_index_type arg)
  {
    msg_.image_index = std::move(arg);
    return Init_CameraImageCaptured_capture_result(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

class Init_CameraImageCaptured_relative_alt
{
public:
  explicit Init_CameraImageCaptured_relative_alt(::mavros_msgs::msg::CameraImageCaptured & msg)
  : msg_(msg)
  {}
  Init_CameraImageCaptured_image_index relative_alt(::mavros_msgs::msg::CameraImageCaptured::_relative_alt_type arg)
  {
    msg_.relative_alt = std::move(arg);
    return Init_CameraImageCaptured_image_index(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

class Init_CameraImageCaptured_geo
{
public:
  explicit Init_CameraImageCaptured_geo(::mavros_msgs::msg::CameraImageCaptured & msg)
  : msg_(msg)
  {}
  Init_CameraImageCaptured_relative_alt geo(::mavros_msgs::msg::CameraImageCaptured::_geo_type arg)
  {
    msg_.geo = std::move(arg);
    return Init_CameraImageCaptured_relative_alt(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

class Init_CameraImageCaptured_orientation
{
public:
  explicit Init_CameraImageCaptured_orientation(::mavros_msgs::msg::CameraImageCaptured & msg)
  : msg_(msg)
  {}
  Init_CameraImageCaptured_geo orientation(::mavros_msgs::msg::CameraImageCaptured::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_CameraImageCaptured_geo(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

class Init_CameraImageCaptured_header
{
public:
  Init_CameraImageCaptured_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraImageCaptured_orientation header(::mavros_msgs::msg::CameraImageCaptured::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CameraImageCaptured_orientation(msg_);
  }

private:
  ::mavros_msgs::msg::CameraImageCaptured msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::CameraImageCaptured>()
{
  return mavros_msgs::msg::builder::Init_CameraImageCaptured_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__BUILDER_HPP_
