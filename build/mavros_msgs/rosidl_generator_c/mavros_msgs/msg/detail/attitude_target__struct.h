// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/AttitudeTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IGNORE_ROLL_RATE'.
/**
  * body_rate.x
 */
enum
{
  mavros_msgs__msg__AttitudeTarget__IGNORE_ROLL_RATE = 1
};

/// Constant 'IGNORE_PITCH_RATE'.
/**
  * body_rate.y
 */
enum
{
  mavros_msgs__msg__AttitudeTarget__IGNORE_PITCH_RATE = 2
};

/// Constant 'IGNORE_YAW_RATE'.
/**
  * body_rate.z
 */
enum
{
  mavros_msgs__msg__AttitudeTarget__IGNORE_YAW_RATE = 4
};

/// Constant 'IGNORE_THRUST'.
enum
{
  mavros_msgs__msg__AttitudeTarget__IGNORE_THRUST = 64
};

/// Constant 'IGNORE_ATTITUDE'.
/**
  * orientation field
 */
enum
{
  mavros_msgs__msg__AttitudeTarget__IGNORE_ATTITUDE = 128
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'body_rate'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/AttitudeTarget in the package mavros_msgs.
/**
  * Message for SET_ATTITUDE_TARGET
  *
  * Some complex system requires all feautures that mavlink
  * message provide. See issue #402, #418.
 */
typedef struct mavros_msgs__msg__AttitudeTarget
{
  std_msgs__msg__Header header;
  uint8_t type_mask;
  geometry_msgs__msg__Quaternion orientation;
  geometry_msgs__msg__Vector3 body_rate;
  float thrust;
} mavros_msgs__msg__AttitudeTarget;

// Struct for a sequence of mavros_msgs__msg__AttitudeTarget.
typedef struct mavros_msgs__msg__AttitudeTarget__Sequence
{
  mavros_msgs__msg__AttitudeTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__AttitudeTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__ATTITUDE_TARGET__STRUCT_H_
