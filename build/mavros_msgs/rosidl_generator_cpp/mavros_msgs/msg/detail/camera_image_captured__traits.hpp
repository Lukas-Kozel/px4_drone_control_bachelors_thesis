// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/CameraImageCaptured.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/camera_image_captured__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'geo'
#include "geographic_msgs/msg/detail/geo_point__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CameraImageCaptured & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
    out << ", ";
  }

  // member: geo
  {
    out << "geo: ";
    to_flow_style_yaml(msg.geo, out);
    out << ", ";
  }

  // member: relative_alt
  {
    out << "relative_alt: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_alt, out);
    out << ", ";
  }

  // member: image_index
  {
    out << "image_index: ";
    rosidl_generator_traits::value_to_yaml(msg.image_index, out);
    out << ", ";
  }

  // member: capture_result
  {
    out << "capture_result: ";
    rosidl_generator_traits::value_to_yaml(msg.capture_result, out);
    out << ", ";
  }

  // member: file_url
  {
    out << "file_url: ";
    rosidl_generator_traits::value_to_yaml(msg.file_url, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraImageCaptured & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }

  // member: geo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geo:\n";
    to_block_style_yaml(msg.geo, out, indentation + 2);
  }

  // member: relative_alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative_alt: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_alt, out);
    out << "\n";
  }

  // member: image_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image_index: ";
    rosidl_generator_traits::value_to_yaml(msg.image_index, out);
    out << "\n";
  }

  // member: capture_result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "capture_result: ";
    rosidl_generator_traits::value_to_yaml(msg.capture_result, out);
    out << "\n";
  }

  // member: file_url
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_url: ";
    rosidl_generator_traits::value_to_yaml(msg.file_url, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraImageCaptured & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::msg::CameraImageCaptured & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::CameraImageCaptured & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::CameraImageCaptured>()
{
  return "mavros_msgs::msg::CameraImageCaptured";
}

template<>
inline const char * name<mavros_msgs::msg::CameraImageCaptured>()
{
  return "mavros_msgs/msg/CameraImageCaptured";
}

template<>
struct has_fixed_size<mavros_msgs::msg::CameraImageCaptured>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::CameraImageCaptured>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::CameraImageCaptured>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__TRAITS_HPP_
