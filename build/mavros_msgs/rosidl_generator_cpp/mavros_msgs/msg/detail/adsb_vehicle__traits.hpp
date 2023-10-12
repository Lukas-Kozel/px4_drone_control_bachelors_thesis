// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/ADSBVehicle.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/adsb_vehicle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'tslc'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ADSBVehicle & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: icao_address
  {
    out << "icao_address: ";
    rosidl_generator_traits::value_to_yaml(msg.icao_address, out);
    out << ", ";
  }

  // member: callsign
  {
    out << "callsign: ";
    rosidl_generator_traits::value_to_yaml(msg.callsign, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: hor_velocity
  {
    out << "hor_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.hor_velocity, out);
    out << ", ";
  }

  // member: ver_velocity
  {
    out << "ver_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.ver_velocity, out);
    out << ", ";
  }

  // member: altitude_type
  {
    out << "altitude_type: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude_type, out);
    out << ", ";
  }

  // member: emitter_type
  {
    out << "emitter_type: ";
    rosidl_generator_traits::value_to_yaml(msg.emitter_type, out);
    out << ", ";
  }

  // member: tslc
  {
    out << "tslc: ";
    to_flow_style_yaml(msg.tslc, out);
    out << ", ";
  }

  // member: flags
  {
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << ", ";
  }

  // member: squawk
  {
    out << "squawk: ";
    rosidl_generator_traits::value_to_yaml(msg.squawk, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ADSBVehicle & msg,
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

  // member: icao_address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "icao_address: ";
    rosidl_generator_traits::value_to_yaml(msg.icao_address, out);
    out << "\n";
  }

  // member: callsign
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "callsign: ";
    rosidl_generator_traits::value_to_yaml(msg.callsign, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: hor_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hor_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.hor_velocity, out);
    out << "\n";
  }

  // member: ver_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ver_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.ver_velocity, out);
    out << "\n";
  }

  // member: altitude_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude_type: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude_type, out);
    out << "\n";
  }

  // member: emitter_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emitter_type: ";
    rosidl_generator_traits::value_to_yaml(msg.emitter_type, out);
    out << "\n";
  }

  // member: tslc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tslc:\n";
    to_block_style_yaml(msg.tslc, out, indentation + 2);
  }

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << "\n";
  }

  // member: squawk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "squawk: ";
    rosidl_generator_traits::value_to_yaml(msg.squawk, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ADSBVehicle & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::ADSBVehicle & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::ADSBVehicle & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::ADSBVehicle>()
{
  return "mavros_msgs::msg::ADSBVehicle";
}

template<>
inline const char * name<mavros_msgs::msg::ADSBVehicle>()
{
  return "mavros_msgs/msg/ADSBVehicle";
}

template<>
struct has_fixed_size<mavros_msgs::msg::ADSBVehicle>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::ADSBVehicle>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::ADSBVehicle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__TRAITS_HPP_
