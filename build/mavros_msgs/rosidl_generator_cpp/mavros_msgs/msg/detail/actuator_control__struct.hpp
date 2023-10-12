// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ActuatorControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ACTUATOR_CONTROL__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ACTUATOR_CONTROL__STRUCT_HPP_

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
# define DEPRECATED__mavros_msgs__msg__ActuatorControl __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ActuatorControl __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ActuatorControl_
{
  using Type = ActuatorControl_<ContainerAllocator>;

  explicit ActuatorControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->group_mix = 0;
      std::fill<typename std::array<float, 8>::iterator, float>(this->controls.begin(), this->controls.end(), 0.0f);
    }
  }

  explicit ActuatorControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    controls(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->group_mix = 0;
      std::fill<typename std::array<float, 8>::iterator, float>(this->controls.begin(), this->controls.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _group_mix_type =
    uint8_t;
  _group_mix_type group_mix;
  using _controls_type =
    std::array<float, 8>;
  _controls_type controls;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__group_mix(
    const uint8_t & _arg)
  {
    this->group_mix = _arg;
    return *this;
  }
  Type & set__controls(
    const std::array<float, 8> & _arg)
  {
    this->controls = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t PX4_MIX_FLIGHT_CONTROL =
    0u;
  static constexpr uint8_t PX4_MIX_FLIGHT_CONTROL_VTOL_ALT =
    1u;
  static constexpr uint8_t PX4_MIX_PAYLOAD =
    2u;
  static constexpr uint8_t PX4_MIX_MANUAL_PASSTHROUGH =
    3u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ActuatorControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ActuatorControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ActuatorControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ActuatorControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ActuatorControl
    std::shared_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ActuatorControl
    std::shared_ptr<mavros_msgs::msg::ActuatorControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ActuatorControl_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->group_mix != other.group_mix) {
      return false;
    }
    if (this->controls != other.controls) {
      return false;
    }
    return true;
  }
  bool operator!=(const ActuatorControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ActuatorControl_

// alias to use template instance with default allocator
using ActuatorControl =
  mavros_msgs::msg::ActuatorControl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ActuatorControl_<ContainerAllocator>::PX4_MIX_FLIGHT_CONTROL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ActuatorControl_<ContainerAllocator>::PX4_MIX_FLIGHT_CONTROL_VTOL_ALT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ActuatorControl_<ContainerAllocator>::PX4_MIX_PAYLOAD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ActuatorControl_<ContainerAllocator>::PX4_MIX_MANUAL_PASSTHROUGH;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ACTUATOR_CONTROL__STRUCT_HPP_
