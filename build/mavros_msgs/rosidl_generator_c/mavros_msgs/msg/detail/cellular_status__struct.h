// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/CellularStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CELLULAR_STATUS__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__CELLULAR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CellularStatus in the package mavros_msgs.
/**
  * Follows https://mavlink.io/en/messages/common.html#CELLULAR_STATUS specification
 */
typedef struct mavros_msgs__msg__CellularStatus
{
  uint8_t status;
  uint8_t failure_reason;
  uint8_t type;
  uint8_t quality;
  uint16_t mcc;
  uint16_t mnc;
  uint16_t lac;
} mavros_msgs__msg__CellularStatus;

// Struct for a sequence of mavros_msgs__msg__CellularStatus.
typedef struct mavros_msgs__msg__CellularStatus__Sequence
{
  mavros_msgs__msg__CellularStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__CellularStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__CELLULAR_STATUS__STRUCT_H_
