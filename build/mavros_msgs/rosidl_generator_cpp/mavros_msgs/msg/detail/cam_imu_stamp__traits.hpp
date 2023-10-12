// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/CamIMUStamp.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/cam_imu_stamp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'frame_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CamIMUStamp & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame_stamp
  {
    out << "frame_stamp: ";
    to_flow_style_yaml(msg.frame_stamp, out);
    out << ", ";
  }

  // member: frame_seq_id
  {
    out << "frame_seq_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_seq_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CamIMUStamp & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_stamp:\n";
    to_block_style_yaml(msg.frame_stamp, out, indentation + 2);
  }

  // member: frame_seq_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_seq_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_seq_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CamIMUStamp & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::CamIMUStamp & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::CamIMUStamp & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::CamIMUStamp>()
{
  return "mavros_msgs::msg::CamIMUStamp";
}

template<>
inline const char * name<mavros_msgs::msg::CamIMUStamp>()
{
  return "mavros_msgs/msg/CamIMUStamp";
}

template<>
struct has_fixed_size<mavros_msgs::msg::CamIMUStamp>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::CamIMUStamp>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<mavros_msgs::msg::CamIMUStamp>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__TRAITS_HPP_
