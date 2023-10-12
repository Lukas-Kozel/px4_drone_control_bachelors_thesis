// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mavros_msgs:msg/CameraImageCaptured.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__STRUCT_H_
#define MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__STRUCT_H_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'geo'
#include "geographic_msgs/msg/detail/geo_point__struct.h"
// Member 'file_url'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CameraImageCaptured in the package mavros_msgs.
/**
  * MAVLink message: CAMERA_IMAGE_CAPTURED
  * https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED
 */
typedef struct mavros_msgs__msg__CameraImageCaptured
{
  std_msgs__msg__Header header;
  /// Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
  geometry_msgs__msg__Quaternion orientation;
  geographic_msgs__msg__GeoPoint geo;
  /// mm Altitude above ground
  float relative_alt;
  /// Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)
  int32_t image_index;
  /// Boolean indicating success (1) or failure (0) while capturing this image.
  int8_t capture_result;
  /// URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
  rosidl_runtime_c__String file_url;
} mavros_msgs__msg__CameraImageCaptured;

// Struct for a sequence of mavros_msgs__msg__CameraImageCaptured.
typedef struct mavros_msgs__msg__CameraImageCaptured__Sequence
{
  mavros_msgs__msg__CameraImageCaptured * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mavros_msgs__msg__CameraImageCaptured__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__CAMERA_IMAGE_CAPTURED__STRUCT_H_
