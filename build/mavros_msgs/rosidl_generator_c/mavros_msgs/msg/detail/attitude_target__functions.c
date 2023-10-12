// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/AttitudeTarget.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/attitude_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `body_rate`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mavros_msgs__msg__AttitudeTarget__init(mavros_msgs__msg__AttitudeTarget * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__AttitudeTarget__fini(msg);
    return false;
  }
  // type_mask
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    mavros_msgs__msg__AttitudeTarget__fini(msg);
    return false;
  }
  // body_rate
  if (!geometry_msgs__msg__Vector3__init(&msg->body_rate)) {
    mavros_msgs__msg__AttitudeTarget__fini(msg);
    return false;
  }
  // thrust
  return true;
}

void
mavros_msgs__msg__AttitudeTarget__fini(mavros_msgs__msg__AttitudeTarget * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // type_mask
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // body_rate
  geometry_msgs__msg__Vector3__fini(&msg->body_rate);
  // thrust
}

bool
mavros_msgs__msg__AttitudeTarget__are_equal(const mavros_msgs__msg__AttitudeTarget * lhs, const mavros_msgs__msg__AttitudeTarget * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // type_mask
  if (lhs->type_mask != rhs->type_mask) {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // body_rate
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->body_rate), &(rhs->body_rate)))
  {
    return false;
  }
  // thrust
  if (lhs->thrust != rhs->thrust) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__AttitudeTarget__copy(
  const mavros_msgs__msg__AttitudeTarget * input,
  mavros_msgs__msg__AttitudeTarget * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // type_mask
  output->type_mask = input->type_mask;
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // body_rate
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->body_rate), &(output->body_rate)))
  {
    return false;
  }
  // thrust
  output->thrust = input->thrust;
  return true;
}

mavros_msgs__msg__AttitudeTarget *
mavros_msgs__msg__AttitudeTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__AttitudeTarget * msg = (mavros_msgs__msg__AttitudeTarget *)allocator.allocate(sizeof(mavros_msgs__msg__AttitudeTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__AttitudeTarget));
  bool success = mavros_msgs__msg__AttitudeTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__AttitudeTarget__destroy(mavros_msgs__msg__AttitudeTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__AttitudeTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__AttitudeTarget__Sequence__init(mavros_msgs__msg__AttitudeTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__AttitudeTarget * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__AttitudeTarget *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__AttitudeTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__AttitudeTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__AttitudeTarget__fini(&data[i - 1]);
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
mavros_msgs__msg__AttitudeTarget__Sequence__fini(mavros_msgs__msg__AttitudeTarget__Sequence * array)
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
      mavros_msgs__msg__AttitudeTarget__fini(&array->data[i]);
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

mavros_msgs__msg__AttitudeTarget__Sequence *
mavros_msgs__msg__AttitudeTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__AttitudeTarget__Sequence * array = (mavros_msgs__msg__AttitudeTarget__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__AttitudeTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__AttitudeTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__AttitudeTarget__Sequence__destroy(mavros_msgs__msg__AttitudeTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__AttitudeTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__AttitudeTarget__Sequence__are_equal(const mavros_msgs__msg__AttitudeTarget__Sequence * lhs, const mavros_msgs__msg__AttitudeTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__AttitudeTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__AttitudeTarget__Sequence__copy(
  const mavros_msgs__msg__AttitudeTarget__Sequence * input,
  mavros_msgs__msg__AttitudeTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__AttitudeTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__AttitudeTarget * data =
      (mavros_msgs__msg__AttitudeTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__AttitudeTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__AttitudeTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__AttitudeTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
