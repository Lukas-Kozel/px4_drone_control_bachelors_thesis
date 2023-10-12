// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/ADSBVehicle.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__STRUCT_HPP_

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
// Member 'tslc'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__ADSBVehicle __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__ADSBVehicle __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ADSBVehicle_
{
  using Type = ADSBVehicle_<ContainerAllocator>;

  explicit ADSBVehicle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    tslc(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->icao_address = 0ul;
      this->callsign = "";
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0f;
      this->heading = 0.0f;
      this->hor_velocity = 0.0f;
      this->ver_velocity = 0.0f;
      this->altitude_type = 0;
      this->emitter_type = 0;
      this->flags = 0;
      this->squawk = 0;
    }
  }

  explicit ADSBVehicle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    callsign(_alloc),
    tslc(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->icao_address = 0ul;
      this->callsign = "";
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0f;
      this->heading = 0.0f;
      this->hor_velocity = 0.0f;
      this->ver_velocity = 0.0f;
      this->altitude_type = 0;
      this->emitter_type = 0;
      this->flags = 0;
      this->squawk = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _icao_address_type =
    uint32_t;
  _icao_address_type icao_address;
  using _callsign_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _callsign_type callsign;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _heading_type =
    float;
  _heading_type heading;
  using _hor_velocity_type =
    float;
  _hor_velocity_type hor_velocity;
  using _ver_velocity_type =
    float;
  _ver_velocity_type ver_velocity;
  using _altitude_type_type =
    uint8_t;
  _altitude_type_type altitude_type;
  using _emitter_type_type =
    uint8_t;
  _emitter_type_type emitter_type;
  using _tslc_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _tslc_type tslc;
  using _flags_type =
    uint16_t;
  _flags_type flags;
  using _squawk_type =
    uint16_t;
  _squawk_type squawk;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__icao_address(
    const uint32_t & _arg)
  {
    this->icao_address = _arg;
    return *this;
  }
  Type & set__callsign(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->callsign = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__hor_velocity(
    const float & _arg)
  {
    this->hor_velocity = _arg;
    return *this;
  }
  Type & set__ver_velocity(
    const float & _arg)
  {
    this->ver_velocity = _arg;
    return *this;
  }
  Type & set__altitude_type(
    const uint8_t & _arg)
  {
    this->altitude_type = _arg;
    return *this;
  }
  Type & set__emitter_type(
    const uint8_t & _arg)
  {
    this->emitter_type = _arg;
    return *this;
  }
  Type & set__tslc(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->tslc = _arg;
    return *this;
  }
  Type & set__flags(
    const uint16_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__squawk(
    const uint16_t & _arg)
  {
    this->squawk = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ALT_PRESSURE_QNH =
    0u;
  static constexpr uint8_t ALT_GEOMETRIC =
    1u;
  static constexpr uint8_t EMITTER_NO_INFO =
    0u;
  static constexpr uint8_t EMITTER_LIGHT =
    1u;
  static constexpr uint8_t EMITTER_SMALL =
    2u;
  static constexpr uint8_t EMITTER_LARGE =
    3u;
  static constexpr uint8_t EMITTER_HIGH_VORTEX_LARGE =
    4u;
  static constexpr uint8_t EMITTER_HEAVY =
    5u;
  static constexpr uint8_t EMITTER_HIGHLY_MANUV =
    6u;
  static constexpr uint8_t EMITTER_ROTOCRAFT =
    7u;
  static constexpr uint8_t EMITTER_UNASSIGNED =
    8u;
  static constexpr uint8_t EMITTER_GLIDER =
    9u;
  static constexpr uint8_t EMITTER_LIGHTER_AIR =
    10u;
  static constexpr uint8_t EMITTER_PARACHUTE =
    11u;
  static constexpr uint8_t EMITTER_ULTRA_LIGHT =
    12u;
  static constexpr uint8_t EMITTER_UNASSIGNED2 =
    13u;
  static constexpr uint8_t EMITTER_UAV =
    14u;
  static constexpr uint8_t EMITTER_SPACE =
    15u;
  static constexpr uint8_t EMITTER_UNASSGINED3 =
    16u;
  static constexpr uint8_t EMITTER_EMERGENCY_SURFACE =
    17u;
  static constexpr uint8_t EMITTER_SERVICE_SURFACE =
    18u;
  static constexpr uint8_t EMITTER_POINT_OBSTACLE =
    19u;
  static constexpr uint16_t FLAG_VALID_COORDS =
    1u;
  static constexpr uint16_t FLAG_VALID_ALTITUDE =
    2u;
  static constexpr uint16_t FLAG_VALID_HEADING =
    4u;
  static constexpr uint16_t FLAG_VALID_VELOCITY =
    8u;
  static constexpr uint16_t FLAG_VALID_CALLSIGN =
    16u;
  static constexpr uint16_t FLAG_VALID_SQUAWK =
    32u;
  static constexpr uint16_t FLAG_SIMULATED =
    64u;
  static constexpr uint16_t FLAG_VERTICAL_VELOCITY_VALID =
    128u;
  static constexpr uint16_t FLAG_BARO_VALID =
    256u;
  static constexpr uint16_t FLAG_SOURCE_UAT =
    32768u;

  // pointer types
  using RawPtr =
    mavros_msgs::msg::ADSBVehicle_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::ADSBVehicle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ADSBVehicle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::ADSBVehicle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__ADSBVehicle
    std::shared_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__ADSBVehicle
    std::shared_ptr<mavros_msgs::msg::ADSBVehicle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ADSBVehicle_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->icao_address != other.icao_address) {
      return false;
    }
    if (this->callsign != other.callsign) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->hor_velocity != other.hor_velocity) {
      return false;
    }
    if (this->ver_velocity != other.ver_velocity) {
      return false;
    }
    if (this->altitude_type != other.altitude_type) {
      return false;
    }
    if (this->emitter_type != other.emitter_type) {
      return false;
    }
    if (this->tslc != other.tslc) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    if (this->squawk != other.squawk) {
      return false;
    }
    return true;
  }
  bool operator!=(const ADSBVehicle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ADSBVehicle_

// alias to use template instance with default allocator
using ADSBVehicle =
  mavros_msgs::msg::ADSBVehicle_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::ALT_PRESSURE_QNH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::ALT_GEOMETRIC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_NO_INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_LIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_SMALL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_LARGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_HIGH_VORTEX_LARGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_HEAVY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_HIGHLY_MANUV;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_ROTOCRAFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_UNASSIGNED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_GLIDER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_LIGHTER_AIR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_PARACHUTE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_ULTRA_LIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_UNASSIGNED2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_UAV;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_SPACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_UNASSGINED3;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_EMERGENCY_SURFACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_SERVICE_SURFACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ADSBVehicle_<ContainerAllocator>::EMITTER_POINT_OBSTACLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VALID_COORDS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VALID_ALTITUDE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VALID_HEADING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VALID_VELOCITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VALID_CALLSIGN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VALID_SQUAWK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_SIMULATED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_VERTICAL_VELOCITY_VALID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_BARO_VALID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ADSBVehicle_<ContainerAllocator>::FLAG_SOURCE_UAT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ADSB_VEHICLE__STRUCT_HPP_
