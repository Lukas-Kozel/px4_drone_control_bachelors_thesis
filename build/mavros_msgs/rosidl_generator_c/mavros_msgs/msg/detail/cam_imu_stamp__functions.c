// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/CamIMUStamp.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/cam_imu_stamp__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `frame_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
mavros_msgs__msg__CamIMUStamp__init(mavros_msgs__msg__CamIMUStamp * msg)
{
  if (!msg) {
    return false;
  }
  // frame_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->frame_stamp)) {
    mavros_msgs__msg__CamIMUStamp__fini(msg);
    return false;
  }
  // frame_seq_id
  return true;
}

void
mavros_msgs__msg__CamIMUStamp__fini(mavros_msgs__msg__CamIMUStamp * msg)
{
  if (!msg) {
    return;
  }
  // frame_stamp
  builtin_interfaces__msg__Time__fini(&msg->frame_stamp);
  // frame_seq_id
}

bool
mavros_msgs__msg__CamIMUStamp__are_equal(const mavros_msgs__msg__CamIMUStamp * lhs, const mavros_msgs__msg__CamIMUStamp * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->frame_stamp), &(rhs->frame_stamp)))
  {
    return false;
  }
  // frame_seq_id
  if (lhs->frame_seq_id != rhs->frame_seq_id) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__CamIMUStamp__copy(
  const mavros_msgs__msg__CamIMUStamp * input,
  mavros_msgs__msg__CamIMUStamp * output)
{
  if (!input || !output) {
    return false;
  }
  // frame_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->frame_stamp), &(output->frame_stamp)))
  {
    return false;
  }
  // frame_seq_id
  output->frame_seq_id = input->frame_seq_id;
  return true;
}

mavros_msgs__msg__CamIMUStamp *
mavros_msgs__msg__CamIMUStamp__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__CamIMUStamp * msg = (mavros_msgs__msg__CamIMUStamp *)allocator.allocate(sizeof(mavros_msgs__msg__CamIMUStamp), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__CamIMUStamp));
  bool success = mavros_msgs__msg__CamIMUStamp__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__CamIMUStamp__destroy(mavros_msgs__msg__CamIMUStamp * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__CamIMUStamp__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__CamIMUStamp__Sequence__init(mavros_msgs__msg__CamIMUStamp__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__CamIMUStamp * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__CamIMUStamp *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__CamIMUStamp), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__CamIMUStamp__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__CamIMUStamp__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mavros_msgs__msg__CamIMUStamp__Sequence__fini(mavros_msgs__msg__CamIMUStamp__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mavros_msgs__msg__CamIMUStamp__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mavros_msgs__msg__CamIMUStamp__Sequence *
mavros_msgs__msg__CamIMUStamp__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__CamIMUStamp__Sequence * array = (mavros_msgs__msg__CamIMUStamp__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__CamIMUStamp__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__CamIMUStamp__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__CamIMUStamp__Sequence__destroy(mavros_msgs__msg__CamIMUStamp__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__CamIMUStamp__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__CamIMUStamp__Sequence__are_equal(const mavros_msgs__msg__CamIMUStamp__Sequence * lhs, const mavros_msgs__msg__CamIMUStamp__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__CamIMUStamp__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__CamIMUStamp__Sequence__copy(
  const mavros_msgs__msg__CamIMUStamp__Sequence * input,
  mavros_msgs__msg__CamIMUStamp__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__CamIMUStamp);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__CamIMUStamp * data =
      (mavros_msgs__msg__CamIMUStamp *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__CamIMUStamp__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__CamIMUStamp__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__CamIMUStamp__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
