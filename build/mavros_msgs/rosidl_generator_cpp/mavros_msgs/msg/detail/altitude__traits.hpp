// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/Altitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ALTITUDE__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ALTITUDE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/altitude__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Altitude & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: monotonic
  {
    out << "monotonic: ";
    rosidl_generator_traits::value_to_yaml(msg.monotonic, out);
    out << ", ";
  }

  // member: amsl
  {
    out << "amsl: ";
    rosidl_generator_traits::value_to_yaml(msg.amsl, out);
    out << ", ";
  }

  // member: local
  {
    out << "local: ";
    rosidl_generator_traits::value_to_yaml(msg.local, out);
    out << ", ";
  }

  // member: relative
  {
    out << "relative: ";
    rosidl_generator_traits::value_to_yaml(msg.relative, out);
    out << ", ";
  }

  // member: terrain
  {
    out << "terrain: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain, out);
    out << ", ";
  }

  // member: bottom_clearance
  {
    out << "bottom_clearance: ";
    rosidl_generator_traits::value_to_yaml(msg.bottom_clearance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Altitude & msg,
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

  // member: monotonic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "monotonic: ";
    rosidl_generator_traits::value_to_yaml(msg.monotonic, out);
    out << "\n";
  }

  // member: amsl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "amsl: ";
    rosidl_generator_traits::value_to_yaml(msg.amsl, out);
    out << "\n";
  }

  // member: local
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local: ";
    rosidl_generator_traits::value_to_yaml(msg.local, out);
    out << "\n";
  }

  // member: relative
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative: ";
    rosidl_generator_traits::value_to_yaml(msg.relative, out);
    out << "\n";
  }

  // member: terrain
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "terrain: ";
    rosidl_generator_traits::value_to_yaml(msg.terrain, out);
    out << "\n";
  }

  // member: bottom_clearance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bottom_clearance: ";
    rosidl_generator_traits::value_to_yaml(msg.bottom_clearance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Altitude & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::Altitude & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::Altitude & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::Altitude>()
{
  return "mavros_msgs::msg::Altitude";
}

template<>
inline const char * name<mavros_msgs::msg::Altitude>()
{
  return "mavros_msgs/msg/Altitude";
}

template<>
struct has_fixed_size<mavros_msgs::msg::Altitude>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::Altitude>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::Altitude>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__ALTITUDE__TRAITS_HPP_
