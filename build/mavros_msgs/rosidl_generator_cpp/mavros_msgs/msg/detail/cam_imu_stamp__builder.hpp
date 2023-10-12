// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/CamIMUStamp.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/cam_imu_stamp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_CamIMUStamp_frame_seq_id
{
public:
  explicit Init_CamIMUStamp_frame_seq_id(::mavros_msgs::msg::CamIMUStamp & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::CamIMUStamp frame_seq_id(::mavros_msgs::msg::CamIMUStamp::_frame_seq_id_type arg)
  {
    msg_.frame_seq_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::CamIMUStamp msg_;
};

class Init_CamIMUStamp_frame_stamp
{
public:
  Init_CamIMUStamp_frame_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CamIMUStamp_frame_seq_id frame_stamp(::mavros_msgs::msg::CamIMUStamp::_frame_stamp_type arg)
  {
    msg_.frame_stamp = std::move(arg);
    return Init_CamIMUStamp_frame_seq_id(msg_);
  }

private:
  ::mavros_msgs::msg::CamIMUStamp msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::CamIMUStamp>()
{
  return mavros_msgs::msg::builder::Init_CamIMUStamp_frame_stamp();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__BUILDER_HPP_
