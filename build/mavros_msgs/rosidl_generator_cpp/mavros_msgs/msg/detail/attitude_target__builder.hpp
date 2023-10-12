// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/AttitudeTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/attitude_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_AttitudeTarget_thrust
{
public:
  explicit Init_AttitudeTarget_thrust(::mavros_msgs::msg::AttitudeTarget & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::AttitudeTarget thrust(::mavros_msgs::msg::AttitudeTarget::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::AttitudeTarget msg_;
};

class Init_AttitudeTarget_body_rate
{
public:
  explicit Init_AttitudeTarget_body_rate(::mavros_msgs::msg::AttitudeTarget & msg)
  : msg_(msg)
  {}
  Init_AttitudeTarget_thrust body_rate(::mavros_msgs::msg::AttitudeTarget::_body_rate_type arg)
  {
    msg_.body_rate = std::move(arg);
    return Init_AttitudeTarget_thrust(msg_);
  }

private:
  ::mavros_msgs::msg::AttitudeTarget msg_;
};

class Init_AttitudeTarget_orientation
{
public:
  explicit Init_AttitudeTarget_orientation(::mavros_msgs::msg::AttitudeTarget & msg)
  : msg_(msg)
  {}
  Init_AttitudeTarget_body_rate orientation(::mavros_msgs::msg::AttitudeTarget::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_AttitudeTarget_body_rate(msg_);
  }

private:
  ::mavros_msgs::msg::AttitudeTarget msg_;
};

class Init_AttitudeTarget_type_mask
{
public:
  explicit Init_AttitudeTarget_type_mask(::mavros_msgs::msg::AttitudeTarget & msg)
  : msg_(msg)
  {}
  Init_AttitudeTarget_orientation type_mask(::mavros_msgs::msg::AttitudeTarget::_type_mask_type arg)
  {
    msg_.type_mask = std::move(arg);
    return Init_AttitudeTarget_orientation(msg_);
  }

private:
  ::mavros_msgs::msg::AttitudeTarget msg_;
};

class Init_AttitudeTarget_header
{
public:
  Init_AttitudeTarget_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AttitudeTarget_type_mask header(::mavros_msgs::msg::AttitudeTarget::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AttitudeTarget_type_mask(msg_);
  }

private:
  ::mavros_msgs::msg::AttitudeTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::AttitudeTarget>()
{
  return mavros_msgs::msg::builder::Init_AttitudeTarget_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__BUILDER_HPP_
