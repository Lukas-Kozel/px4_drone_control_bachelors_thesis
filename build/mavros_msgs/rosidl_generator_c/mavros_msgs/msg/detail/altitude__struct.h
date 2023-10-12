// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/Altitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ALTITUDE__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ALTITUDE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Altitude in the package mavros_msgs.
/**
  * Altitude information
  *
  * https://mavlink.io/en/messages/common.html#ALTITUDE
 */
typedef struct mavros_msgs__msg__Altitude
{
  std_msgs__msg__Header header;
  float monotonic;
  float amsl;
  float local;
  float relative;
  float terrain;
  float bottom_clearance;
} mavros_msgs__msg__Altitude;

// Struct for a sequence of mavros_msgs__msg__Altitude.
typedef struct mavros_msgs__msg__Altitude__Sequence
{
  mavros_msgs__msg__Altitude * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__Altitude__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ALTITUDE__STRUCT_H_
