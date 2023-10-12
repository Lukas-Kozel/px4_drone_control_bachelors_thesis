// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mavros_msgs:msg/ADSBVehicle.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/adsb_vehicle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `callsign`
#include "rosidl_runtime_c/string_functions.h"
// Member `tslc`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
mavros_msgs__msg__ADSBVehicle__init(mavros_msgs__msg__ADSBVehicle * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mavros_msgs__msg__ADSBVehicle__fini(msg);
    return false;
  }
  // icao_address
  // callsign
  if (!rosidl_runtime_c__String__init(&msg->callsign)) {
    mavros_msgs__msg__ADSBVehicle__fini(msg);
    return false;
  }
  // latitude
  // longitude
  // altitude
  // heading
  // hor_velocity
  // ver_velocity
  // altitude_type
  // emitter_type
  // tslc
  if (!builtin_interfaces__msg__Duration__init(&msg->tslc)) {
    mavros_msgs__msg__ADSBVehicle__fini(msg);
    return false;
  }
  // flags
  // squawk
  return true;
}

void
mavros_msgs__msg__ADSBVehicle__fini(mavros_msgs__msg__ADSBVehicle * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // icao_address
  // callsign
  rosidl_runtime_c__String__fini(&msg->callsign);
  // latitude
  // longitude
  // altitude
  // heading
  // hor_velocity
  // ver_velocity
  // altitude_type
  // emitter_type
  // tslc
  builtin_interfaces__msg__Duration__fini(&msg->tslc);
  // flags
  // squawk
}

bool
mavros_msgs__msg__ADSBVehicle__are_equal(const mavros_msgs__msg__ADSBVehicle * lhs, const mavros_msgs__msg__ADSBVehicle * rhs)
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
  // icao_address
  if (lhs->icao_address != rhs->icao_address) {
    return false;
  }
  // callsign
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->callsign), &(rhs->callsign)))
  {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // hor_velocity
  if (lhs->hor_velocity != rhs->hor_velocity) {
    return false;
  }
  // ver_velocity
  if (lhs->ver_velocity != rhs->ver_velocity) {
    return false;
  }
  // altitude_type
  if (lhs->altitude_type != rhs->altitude_type) {
    return false;
  }
  // emitter_type
  if (lhs->emitter_type != rhs->emitter_type) {
    return false;
  }
  // tslc
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->tslc), &(rhs->tslc)))
  {
    return false;
  }
  // flags
  if (lhs->flags != rhs->flags) {
    return false;
  }
  // squawk
  if (lhs->squawk != rhs->squawk) {
    return false;
  }
  return true;
}

bool
mavros_msgs__msg__ADSBVehicle__copy(
  const mavros_msgs__msg__ADSBVehicle * input,
  mavros_msgs__msg__ADSBVehicle * output)
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
  // icao_address
  output->icao_address = input->icao_address;
  // callsign
  if (!rosidl_runtime_c__String__copy(
      &(input->callsign), &(output->callsign)))
  {
    return false;
  }
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // heading
  output->heading = input->heading;
  // hor_velocity
  output->hor_velocity = input->hor_velocity;
  // ver_velocity
  output->ver_velocity = input->ver_velocity;
  // altitude_type
  output->altitude_type = input->altitude_type;
  // emitter_type
  output->emitter_type = input->emitter_type;
  // tslc
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->tslc), &(output->tslc)))
  {
    return false;
  }
  // flags
  output->flags = input->flags;
  // squawk
  output->squawk = input->squawk;
  return true;
}

mavros_msgs__msg__ADSBVehicle *
mavros_msgs__msg__ADSBVehicle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ADSBVehicle * msg = (mavros_msgs__msg__ADSBVehicle *)allocator.allocate(sizeof(mavros_msgs__msg__ADSBVehicle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mavros_msgs__msg__ADSBVehicle));
  bool success = mavros_msgs__msg__ADSBVehicle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mavros_msgs__msg__ADSBVehicle__destroy(mavros_msgs__msg__ADSBVehicle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mavros_msgs__msg__ADSBVehicle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mavros_msgs__msg__ADSBVehicle__Sequence__init(mavros_msgs__msg__ADSBVehicle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ADSBVehicle * data = NULL;

  if (size) {
    data = (mavros_msgs__msg__ADSBVehicle *)allocator.zero_allocate(size, sizeof(mavros_msgs__msg__ADSBVehicle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mavros_msgs__msg__ADSBVehicle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mavros_msgs__msg__ADSBVehicle__fini(&data[i - 1]);
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
mavros_msgs__msg__ADSBVehicle__Sequence__fini(mavros_msgs__msg__ADSBVehicle__Sequence * array)
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
      mavros_msgs__msg__ADSBVehicle__fini(&array->data[i]);
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

mavros_msgs__msg__ADSBVehicle__Sequence *
mavros_msgs__msg__ADSBVehicle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mavros_msgs__msg__ADSBVehicle__Sequence * array = (mavros_msgs__msg__ADSBVehicle__Sequence *)allocator.allocate(sizeof(mavros_msgs__msg__ADSBVehicle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mavros_msgs__msg__ADSBVehicle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mavros_msgs__msg__ADSBVehicle__Sequence__destroy(mavros_msgs__msg__ADSBVehicle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mavros_msgs__msg__ADSBVehicle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mavros_msgs__msg__ADSBVehicle__Sequence__are_equal(const mavros_msgs__msg__ADSBVehicle__Sequence * lhs, const mavros_msgs__msg__ADSBVehicle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mavros_msgs__msg__ADSBVehicle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mavros_msgs__msg__ADSBVehicle__Sequence__copy(
  const mavros_msgs__msg__ADSBVehicle__Sequence * input,
  mavros_msgs__msg__ADSBVehicle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mavros_msgs__msg__ADSBVehicle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mavros_msgs__msg__ADSBVehicle * data =
      (mavros_msgs__msg__ADSBVehicle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mavros_msgs__msg__ADSBVehicle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mavros_msgs__msg__ADSBVehicle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mavros_msgs__msg__ADSBVehicle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
