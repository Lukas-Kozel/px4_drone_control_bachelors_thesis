// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/CellularStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CELLULAR_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CELLULAR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/cellular_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_CellularStatus_lac
{
public:
  explicit Init_CellularStatus_lac(::mavros_msgs::msg::CellularStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::CellularStatus lac(::mavros_msgs::msg::CellularStatus::_lac_type arg)
  {
    msg_.lac = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

class Init_CellularStatus_mnc
{
public:
  explicit Init_CellularStatus_mnc(::mavros_msgs::msg::CellularStatus & msg)
  : msg_(msg)
  {}
  Init_CellularStatus_lac mnc(::mavros_msgs::msg::CellularStatus::_mnc_type arg)
  {
    msg_.mnc = std::move(arg);
    return Init_CellularStatus_lac(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

class Init_CellularStatus_mcc
{
public:
  explicit Init_CellularStatus_mcc(::mavros_msgs::msg::CellularStatus & msg)
  : msg_(msg)
  {}
  Init_CellularStatus_mnc mcc(::mavros_msgs::msg::CellularStatus::_mcc_type arg)
  {
    msg_.mcc = std::move(arg);
    return Init_CellularStatus_mnc(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

class Init_CellularStatus_quality
{
public:
  explicit Init_CellularStatus_quality(::mavros_msgs::msg::CellularStatus & msg)
  : msg_(msg)
  {}
  Init_CellularStatus_mcc quality(::mavros_msgs::msg::CellularStatus::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_CellularStatus_mcc(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

class Init_CellularStatus_type
{
public:
  explicit Init_CellularStatus_type(::mavros_msgs::msg::CellularStatus & msg)
  : msg_(msg)
  {}
  Init_CellularStatus_quality type(::mavros_msgs::msg::CellularStatus::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_CellularStatus_quality(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

class Init_CellularStatus_failure_reason
{
public:
  explicit Init_CellularStatus_failure_reason(::mavros_msgs::msg::CellularStatus & msg)
  : msg_(msg)
  {}
  Init_CellularStatus_type failure_reason(::mavros_msgs::msg::CellularStatus::_failure_reason_type arg)
  {
    msg_.failure_reason = std::move(arg);
    return Init_CellularStatus_type(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

class Init_CellularStatus_status
{
public:
  Init_CellularStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CellularStatus_failure_reason status(::mavros_msgs::msg::CellularStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_CellularStatus_failure_reason(msg_);
  }

private:
  ::mavros_msgs::msg::CellularStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::CellularStatus>()
{
  return mavros_msgs::msg::builder::Init_CellularStatus_status();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__CELLULAR_STATUS__BUILDER_HPP_
