// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/Altitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ALTITUDE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ALTITUDE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__Altitude __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__Altitude __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Altitude_
{
  using Type = Altitude_<ContainerAllocator>;

  explicit Altitude_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->monotonic = 0.0f;
      this->amsl = 0.0f;
      this->local = 0.0f;
      this->relative = 0.0f;
      this->terrain = 0.0f;
      this->bottom_clearance = 0.0f;
    }
  }

  explicit Altitude_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->monotonic = 0.0f;
      this->amsl = 0.0f;
      this->local = 0.0f;
      this->relative = 0.0f;
      this->terrain = 0.0f;
      this->bottom_clearance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _monotonic_type =
    float;
  _monotonic_type monotonic;
  using _amsl_type =
    float;
  _amsl_type amsl;
  using _local_type =
    float;
  _local_type local;
  using _relative_type =
    float;
  _relative_type relative;
  using _terrain_type =
    float;
  _terrain_type terrain;
  using _bottom_clearance_type =
    float;
  _bottom_clearance_type bottom_clearance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__monotonic(
    const float & _arg)
  {
    this->monotonic = _arg;
    return *this;
  }
  Type & set__amsl(
    const float & _arg)
  {
    this->amsl = _arg;
    return *this;
  }
  Type & set__local(
    const float & _arg)
  {
    this->local = _arg;
    return *this;
  }
  Type & set__relative(
    const float & _arg)
  {
    this->relative = _arg;
    return *this;
  }
  Type & set__terrain(
    const float & _arg)
  {
    this->terrain = _arg;
    return *this;
  }
  Type & set__bottom_clearance(
    const float & _arg)
  {
    this->bottom_clearance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::Altitude_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::Altitude_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Altitude_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::Altitude_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__Altitude
    std::shared_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__Altitude
    std::shared_ptr<mavros_msgs::msg::Altitude_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Altitude_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->monotonic != other.monotonic) {
      return false;
    }
    if (this->amsl != other.amsl) {
      return false;
    }
    if (this->local != other.local) {
      return false;
    }
    if (this->relative != other.relative) {
      return false;
    }
    if (this->terrain != other.terrain) {
      return false;
    }
    if (this->bottom_clearance != other.bottom_clearance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Altitude_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Altitude_

// alias to use template instance with default allocator
using Altitude =
  mavros_msgs::msg::Altitude_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ALTITUDE__STRUCT_HPP_
