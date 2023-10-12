// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/CamIMUStamp.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'frame_stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__CamIMUStamp __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__CamIMUStamp __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CamIMUStamp_
{
  using Type = CamIMUStamp_<ContainerAllocator>;

  explicit CamIMUStamp_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_seq_id = 0l;
    }
  }

  explicit CamIMUStamp_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_seq_id = 0l;
    }
  }

  // field types and members
  using _frame_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _frame_stamp_type frame_stamp;
  using _frame_seq_id_type =
    int32_t;
  _frame_seq_id_type frame_seq_id;

  // setters for named parameter idiom
  Type & set__frame_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->frame_stamp = _arg;
    return *this;
  }
  Type & set__frame_seq_id(
    const int32_t & _arg)
  {
    this->frame_seq_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::CamIMUStamp_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::CamIMUStamp_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CamIMUStamp_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CamIMUStamp_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__CamIMUStamp
    std::shared_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__CamIMUStamp
    std::shared_ptr<mavros_msgs::msg::CamIMUStamp_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CamIMUStamp_ & other) const
  {
    if (this->frame_stamp != other.frame_stamp) {
      return false;
    }
    if (this->frame_seq_id != other.frame_seq_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const CamIMUStamp_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CamIMUStamp_

// alias to use template instance with default allocator
using CamIMUStamp =
  mavros_msgs::msg::CamIMUStamp_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__STRUCT_HPP_
