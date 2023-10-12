// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/AttitudeTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__STRUCT_HPP_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'body_rate'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__AttitudeTarget __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__AttitudeTarget __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AttitudeTarget_
{
  using Type = AttitudeTarget_<ContainerAllocator>;

  explicit AttitudeTarget_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    orientation(_init),
    body_rate(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type_mask = 0;
      this->thrust = 0.0f;
    }
  }

  explicit AttitudeTarget_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    orientation(_alloc, _init),
    body_rate(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type_mask = 0;
      this->thrust = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _type_mask_type =
    uint8_t;
  _type_mask_type type_mask;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _body_rate_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _body_rate_type body_rate;
  using _thrust_type =
    float;
  _thrust_type thrust;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__type_mask(
    const uint8_t & _arg)
  {
    this->type_mask = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__body_rate(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->body_rate = _arg;
    return *this;
  }
  Type & set__thrust(
    const float & _arg)
  {
    this->thrust = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IGNORE_ROLL_RATE =
    1u;
  static constexpr uint8_t IGNORE_PITCH_RATE =
    2u;
  static constexpr uint8_t IGNORE_YAW_RATE =
    4u;
  static constexpr uint8_t IGNORE_THRUST =
    64u;
  static constexpr uint8_t IGNORE_ATTITUDE =
    128u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::AttitudeTarget_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::AttitudeTarget_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::AttitudeTarget_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::AttitudeTarget_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__AttitudeTarget
    std::shared_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__AttitudeTarget
    std::shared_ptr<mavros_msgs::msg::AttitudeTarget_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AttitudeTarget_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->type_mask != other.type_mask) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->body_rate != other.body_rate) {
      return false;
    }
    if (this->thrust != other.thrust) {
      return false;
    }
    return true;
  }
  bool operator!=(const AttitudeTarget_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AttitudeTarget_

// alias to use template instance with default allocator
using AttitudeTarget =
  mavros_msgs::msg::AttitudeTarget_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AttitudeTarget_<ContainerAllocator>::IGNORE_ROLL_RATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AttitudeTarget_<ContainerAllocator>::IGNORE_PITCH_RATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AttitudeTarget_<ContainerAllocator>::IGNORE_YAW_RATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AttitudeTarget_<ContainerAllocator>::IGNORE_THRUST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AttitudeTarget_<ContainerAllocator>::IGNORE_ATTITUDE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__STRUCT_HPP_
