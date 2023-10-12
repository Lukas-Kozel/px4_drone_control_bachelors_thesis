// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/CameraImageCaptured.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__STRUCT_HPP_

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
// Member 'geo'
#include "geographic_msgs/msg/detail/geo_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__CameraImageCaptured __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__CameraImageCaptured __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraImageCaptured_
{
  using Type = CameraImageCaptured_<ContainerAllocator>;

  explicit CameraImageCaptured_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    orientation(_init),
    geo(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->relative_alt = 0.0f;
      this->image_index = 0l;
      this->capture_result = 0;
      this->file_url = "";
    }
  }

  explicit CameraImageCaptured_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    orientation(_alloc, _init),
    geo(_alloc, _init),
    file_url(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->relative_alt = 0.0f;
      this->image_index = 0l;
      this->capture_result = 0;
      this->file_url = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _geo_type =
    geographic_msgs::msg::GeoPoint_<ContainerAllocator>;
  _geo_type geo;
  using _relative_alt_type =
    float;
  _relative_alt_type relative_alt;
  using _image_index_type =
    int32_t;
  _image_index_type image_index;
  using _capture_result_type =
    int8_t;
  _capture_result_type capture_result;
  using _file_url_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_url_type file_url;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__geo(
    const geographic_msgs::msg::GeoPoint_<ContainerAllocator> & _arg)
  {
    this->geo = _arg;
    return *this;
  }
  Type & set__relative_alt(
    const float & _arg)
  {
    this->relative_alt = _arg;
    return *this;
  }
  Type & set__image_index(
    const int32_t & _arg)
  {
    this->image_index = _arg;
    return *this;
  }
  Type & set__capture_result(
    const int8_t & _arg)
  {
    this->capture_result = _arg;
    return *this;
  }
  Type & set__file_url(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_url = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__CameraImageCaptured
    std::shared_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__CameraImageCaptured
    std::shared_ptr<mavros_msgs::msg::CameraImageCaptured_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraImageCaptured_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->geo != other.geo) {
      return false;
    }
    if (this->relative_alt != other.relative_alt) {
      return false;
    }
    if (this->image_index != other.image_index) {
      return false;
    }
    if (this->capture_result != other.capture_result) {
      return false;
    }
    if (this->file_url != other.file_url) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraImageCaptured_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraImageCaptured_

// alias to use template instance with default allocator
using CameraImageCaptured =
  mavros_msgs::msg::CameraImageCaptured_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__STRUCT_HPP_
