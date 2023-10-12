// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/CamIMUStamp.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/CamIMUStamp in the package mavros_msgs.
/**
  * IMU-Camera synchronisation data
 */
typedef struct mavros_msgs__msg__CamIMUStamp
{
  /// Timestamp when the camera was triggered
  builtin_interfaces__msg__Time frame_stamp;
  /// Sequence number of the image frame
  int32_t frame_seq_id;
} mavros_msgs__msg__CamIMUStamp;

// Struct for a sequence of mavros_msgs__msg__CamIMUStamp.
typedef struct mavros_msgs__msg__CamIMUStamp__Sequence
{
  mavros_msgs__msg__CamIMUStamp * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__CamIMUStamp__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__CAM_IMU_STAMP__STRUCT_H_
