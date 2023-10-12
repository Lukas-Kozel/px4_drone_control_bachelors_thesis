// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ActuatorControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ACTUATOR_CONTROL__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ACTUATOR_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/actuator_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ActuatorControl_controls
{
public:
  explicit Init_ActuatorControl_controls(::mavros_msgs::msg::ActuatorControl & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ActuatorControl controls(::mavros_msgs::msg::ActuatorControl::_controls_type arg)
  {
    msg_.controls = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ActuatorControl msg_;
};

class Init_ActuatorControl_group_mix
{
public:
  explicit Init_ActuatorControl_group_mix(::mavros_msgs::msg::ActuatorControl & msg)
  : msg_(msg)
  {}
  Init_ActuatorControl_controls group_mix(::mavros_msgs::msg::ActuatorControl::_group_mix_type arg)
  {
    msg_.group_mix = std::move(arg);
    return Init_ActuatorControl_controls(msg_);
  }

private:
  ::mavros_msgs::msg::ActuatorControl msg_;
};

class Init_ActuatorControl_header
{
public:
  Init_ActuatorControl_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActuatorControl_group_mix header(::mavros_msgs::msg::ActuatorControl::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ActuatorControl_group_mix(msg_);
  }

private:
  ::mavros_msgs::msg::ActuatorControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ActuatorControl>()
{
  return mavros_msgs::msg::builder::Init_ActuatorControl_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ACTUATOR_CONTROL__BUILDER_HPP_
