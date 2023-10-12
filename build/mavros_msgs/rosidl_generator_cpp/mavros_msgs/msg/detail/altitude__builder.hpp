// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Altitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ALTITUDE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ALTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/altitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Altitude_bottom_clearance
{
public:
  explicit Init_Altitude_bottom_clearance(::mavros_msgs::msg::Altitude & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Altitude bottom_clearance(::mavros_msgs::msg::Altitude::_bottom_clearance_type arg)
  {
    msg_.bottom_clearance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

class Init_Altitude_terrain
{
public:
  explicit Init_Altitude_terrain(::mavros_msgs::msg::Altitude & msg)
  : msg_(msg)
  {}
  Init_Altitude_bottom_clearance terrain(::mavros_msgs::msg::Altitude::_terrain_type arg)
  {
    msg_.terrain = std::move(arg);
    return Init_Altitude_bottom_clearance(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

class Init_Altitude_relative
{
public:
  explicit Init_Altitude_relative(::mavros_msgs::msg::Altitude & msg)
  : msg_(msg)
  {}
  Init_Altitude_terrain relative(::mavros_msgs::msg::Altitude::_relative_type arg)
  {
    msg_.relative = std::move(arg);
    return Init_Altitude_terrain(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

class Init_Altitude_local
{
public:
  explicit Init_Altitude_local(::mavros_msgs::msg::Altitude & msg)
  : msg_(msg)
  {}
  Init_Altitude_relative local(::mavros_msgs::msg::Altitude::_local_type arg)
  {
    msg_.local = std::move(arg);
    return Init_Altitude_relative(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

class Init_Altitude_amsl
{
public:
  explicit Init_Altitude_amsl(::mavros_msgs::msg::Altitude & msg)
  : msg_(msg)
  {}
  Init_Altitude_local amsl(::mavros_msgs::msg::Altitude::_amsl_type arg)
  {
    msg_.amsl = std::move(arg);
    return Init_Altitude_local(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

class Init_Altitude_monotonic
{
public:
  explicit Init_Altitude_monotonic(::mavros_msgs::msg::Altitude & msg)
  : msg_(msg)
  {}
  Init_Altitude_amsl monotonic(::mavros_msgs::msg::Altitude::_monotonic_type arg)
  {
    msg_.monotonic = std::move(arg);
    return Init_Altitude_amsl(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

class Init_Altitude_header
{
public:
  Init_Altitude_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Altitude_monotonic header(::mavros_msgs::msg::Altitude::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Altitude_monotonic(msg_);
  }

private:
  ::mavros_msgs::msg::Altitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Altitude>()
{
  return mavros_msgs::msg::builder::Init_Altitude_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ALTITUDE__BUILDER_HPP_
