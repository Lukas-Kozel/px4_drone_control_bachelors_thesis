// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ADSBVehicle.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/adsb_vehicle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ADSBVehicle_squawk
{
public:
  explicit Init_ADSBVehicle_squawk(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ADSBVehicle squawk(::mavros_msgs::msg::ADSBVehicle::_squawk_type arg)
  {
    msg_.squawk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_flags
{
public:
  explicit Init_ADSBVehicle_flags(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_squawk flags(::mavros_msgs::msg::ADSBVehicle::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_ADSBVehicle_squawk(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_tslc
{
public:
  explicit Init_ADSBVehicle_tslc(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_flags tslc(::mavros_msgs::msg::ADSBVehicle::_tslc_type arg)
  {
    msg_.tslc = std::move(arg);
    return Init_ADSBVehicle_flags(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_emitter_type
{
public:
  explicit Init_ADSBVehicle_emitter_type(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_tslc emitter_type(::mavros_msgs::msg::ADSBVehicle::_emitter_type_type arg)
  {
    msg_.emitter_type = std::move(arg);
    return Init_ADSBVehicle_tslc(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_altitude_type
{
public:
  explicit Init_ADSBVehicle_altitude_type(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_emitter_type altitude_type(::mavros_msgs::msg::ADSBVehicle::_altitude_type_type arg)
  {
    msg_.altitude_type = std::move(arg);
    return Init_ADSBVehicle_emitter_type(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_ver_velocity
{
public:
  explicit Init_ADSBVehicle_ver_velocity(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_altitude_type ver_velocity(::mavros_msgs::msg::ADSBVehicle::_ver_velocity_type arg)
  {
    msg_.ver_velocity = std::move(arg);
    return Init_ADSBVehicle_altitude_type(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_hor_velocity
{
public:
  explicit Init_ADSBVehicle_hor_velocity(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_ver_velocity hor_velocity(::mavros_msgs::msg::ADSBVehicle::_hor_velocity_type arg)
  {
    msg_.hor_velocity = std::move(arg);
    return Init_ADSBVehicle_ver_velocity(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_heading
{
public:
  explicit Init_ADSBVehicle_heading(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_hor_velocity heading(::mavros_msgs::msg::ADSBVehicle::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_ADSBVehicle_hor_velocity(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_altitude
{
public:
  explicit Init_ADSBVehicle_altitude(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_heading altitude(::mavros_msgs::msg::ADSBVehicle::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_ADSBVehicle_heading(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_longitude
{
public:
  explicit Init_ADSBVehicle_longitude(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_altitude longitude(::mavros_msgs::msg::ADSBVehicle::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_ADSBVehicle_altitude(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_latitude
{
public:
  explicit Init_ADSBVehicle_latitude(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_longitude latitude(::mavros_msgs::msg::ADSBVehicle::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_ADSBVehicle_longitude(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_callsign
{
public:
  explicit Init_ADSBVehicle_callsign(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_latitude callsign(::mavros_msgs::msg::ADSBVehicle::_callsign_type arg)
  {
    msg_.callsign = std::move(arg);
    return Init_ADSBVehicle_latitude(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_icao_address
{
public:
  explicit Init_ADSBVehicle_icao_address(::mavros_msgs::msg::ADSBVehicle & msg)
  : msg_(msg)
  {}
  Init_ADSBVehicle_callsign icao_address(::mavros_msgs::msg::ADSBVehicle::_icao_address_type arg)
  {
    msg_.icao_address = std::move(arg);
    return Init_ADSBVehicle_callsign(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

class Init_ADSBVehicle_header
{
public:
  Init_ADSBVehicle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ADSBVehicle_icao_address header(::mavros_msgs::msg::ADSBVehicle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ADSBVehicle_icao_address(msg_);
  }

private:
  ::mavros_msgs::msg::ADSBVehicle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ADSBVehicle>()
{
  return mavros_msgs::msg::builder::Init_ADSBVehicle_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__BUILDER_HPP_
