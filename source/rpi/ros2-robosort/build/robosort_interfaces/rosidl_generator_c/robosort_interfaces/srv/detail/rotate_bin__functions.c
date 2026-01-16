// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robosort_interfaces:srv/RotateBin.idl
// generated code does not contain a copyright notice
#include "robosort_interfaces/srv/detail/rotate_bin__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
robosort_interfaces__srv__RotateBin_Request__init(robosort_interfaces__srv__RotateBin_Request * msg)
{
  if (!msg) {
    return false;
  }
  // compartment_number
  return true;
}

void
robosort_interfaces__srv__RotateBin_Request__fini(robosort_interfaces__srv__RotateBin_Request * msg)
{
  if (!msg) {
    return;
  }
  // compartment_number
}

bool
robosort_interfaces__srv__RotateBin_Request__are_equal(const robosort_interfaces__srv__RotateBin_Request * lhs, const robosort_interfaces__srv__RotateBin_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // compartment_number
  if (lhs->compartment_number != rhs->compartment_number) {
    return false;
  }
  return true;
}

bool
robosort_interfaces__srv__RotateBin_Request__copy(
  const robosort_interfaces__srv__RotateBin_Request * input,
  robosort_interfaces__srv__RotateBin_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // compartment_number
  output->compartment_number = input->compartment_number;
  return true;
}

robosort_interfaces__srv__RotateBin_Request *
robosort_interfaces__srv__RotateBin_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Request * msg = (robosort_interfaces__srv__RotateBin_Request *)allocator.allocate(sizeof(robosort_interfaces__srv__RotateBin_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robosort_interfaces__srv__RotateBin_Request));
  bool success = robosort_interfaces__srv__RotateBin_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robosort_interfaces__srv__RotateBin_Request__destroy(robosort_interfaces__srv__RotateBin_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robosort_interfaces__srv__RotateBin_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robosort_interfaces__srv__RotateBin_Request__Sequence__init(robosort_interfaces__srv__RotateBin_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Request * data = NULL;

  if (size) {
    data = (robosort_interfaces__srv__RotateBin_Request *)allocator.zero_allocate(size, sizeof(robosort_interfaces__srv__RotateBin_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robosort_interfaces__srv__RotateBin_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robosort_interfaces__srv__RotateBin_Request__fini(&data[i - 1]);
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
robosort_interfaces__srv__RotateBin_Request__Sequence__fini(robosort_interfaces__srv__RotateBin_Request__Sequence * array)
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
      robosort_interfaces__srv__RotateBin_Request__fini(&array->data[i]);
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

robosort_interfaces__srv__RotateBin_Request__Sequence *
robosort_interfaces__srv__RotateBin_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Request__Sequence * array = (robosort_interfaces__srv__RotateBin_Request__Sequence *)allocator.allocate(sizeof(robosort_interfaces__srv__RotateBin_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robosort_interfaces__srv__RotateBin_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robosort_interfaces__srv__RotateBin_Request__Sequence__destroy(robosort_interfaces__srv__RotateBin_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robosort_interfaces__srv__RotateBin_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robosort_interfaces__srv__RotateBin_Request__Sequence__are_equal(const robosort_interfaces__srv__RotateBin_Request__Sequence * lhs, const robosort_interfaces__srv__RotateBin_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robosort_interfaces__srv__RotateBin_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robosort_interfaces__srv__RotateBin_Request__Sequence__copy(
  const robosort_interfaces__srv__RotateBin_Request__Sequence * input,
  robosort_interfaces__srv__RotateBin_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robosort_interfaces__srv__RotateBin_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robosort_interfaces__srv__RotateBin_Request * data =
      (robosort_interfaces__srv__RotateBin_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robosort_interfaces__srv__RotateBin_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robosort_interfaces__srv__RotateBin_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robosort_interfaces__srv__RotateBin_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
robosort_interfaces__srv__RotateBin_Response__init(robosort_interfaces__srv__RotateBin_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    robosort_interfaces__srv__RotateBin_Response__fini(msg);
    return false;
  }
  return true;
}

void
robosort_interfaces__srv__RotateBin_Response__fini(robosort_interfaces__srv__RotateBin_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
robosort_interfaces__srv__RotateBin_Response__are_equal(const robosort_interfaces__srv__RotateBin_Response * lhs, const robosort_interfaces__srv__RotateBin_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
robosort_interfaces__srv__RotateBin_Response__copy(
  const robosort_interfaces__srv__RotateBin_Response * input,
  robosort_interfaces__srv__RotateBin_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

robosort_interfaces__srv__RotateBin_Response *
robosort_interfaces__srv__RotateBin_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Response * msg = (robosort_interfaces__srv__RotateBin_Response *)allocator.allocate(sizeof(robosort_interfaces__srv__RotateBin_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robosort_interfaces__srv__RotateBin_Response));
  bool success = robosort_interfaces__srv__RotateBin_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robosort_interfaces__srv__RotateBin_Response__destroy(robosort_interfaces__srv__RotateBin_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robosort_interfaces__srv__RotateBin_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robosort_interfaces__srv__RotateBin_Response__Sequence__init(robosort_interfaces__srv__RotateBin_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Response * data = NULL;

  if (size) {
    data = (robosort_interfaces__srv__RotateBin_Response *)allocator.zero_allocate(size, sizeof(robosort_interfaces__srv__RotateBin_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robosort_interfaces__srv__RotateBin_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robosort_interfaces__srv__RotateBin_Response__fini(&data[i - 1]);
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
robosort_interfaces__srv__RotateBin_Response__Sequence__fini(robosort_interfaces__srv__RotateBin_Response__Sequence * array)
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
      robosort_interfaces__srv__RotateBin_Response__fini(&array->data[i]);
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

robosort_interfaces__srv__RotateBin_Response__Sequence *
robosort_interfaces__srv__RotateBin_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Response__Sequence * array = (robosort_interfaces__srv__RotateBin_Response__Sequence *)allocator.allocate(sizeof(robosort_interfaces__srv__RotateBin_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robosort_interfaces__srv__RotateBin_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robosort_interfaces__srv__RotateBin_Response__Sequence__destroy(robosort_interfaces__srv__RotateBin_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robosort_interfaces__srv__RotateBin_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robosort_interfaces__srv__RotateBin_Response__Sequence__are_equal(const robosort_interfaces__srv__RotateBin_Response__Sequence * lhs, const robosort_interfaces__srv__RotateBin_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robosort_interfaces__srv__RotateBin_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robosort_interfaces__srv__RotateBin_Response__Sequence__copy(
  const robosort_interfaces__srv__RotateBin_Response__Sequence * input,
  robosort_interfaces__srv__RotateBin_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robosort_interfaces__srv__RotateBin_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robosort_interfaces__srv__RotateBin_Response * data =
      (robosort_interfaces__srv__RotateBin_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robosort_interfaces__srv__RotateBin_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robosort_interfaces__srv__RotateBin_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robosort_interfaces__srv__RotateBin_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__functions.h"

bool
robosort_interfaces__srv__RotateBin_Event__init(robosort_interfaces__srv__RotateBin_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robosort_interfaces__srv__RotateBin_Event__fini(msg);
    return false;
  }
  // request
  if (!robosort_interfaces__srv__RotateBin_Request__Sequence__init(&msg->request, 0)) {
    robosort_interfaces__srv__RotateBin_Event__fini(msg);
    return false;
  }
  // response
  if (!robosort_interfaces__srv__RotateBin_Response__Sequence__init(&msg->response, 0)) {
    robosort_interfaces__srv__RotateBin_Event__fini(msg);
    return false;
  }
  return true;
}

void
robosort_interfaces__srv__RotateBin_Event__fini(robosort_interfaces__srv__RotateBin_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robosort_interfaces__srv__RotateBin_Request__Sequence__fini(&msg->request);
  // response
  robosort_interfaces__srv__RotateBin_Response__Sequence__fini(&msg->response);
}

bool
robosort_interfaces__srv__RotateBin_Event__are_equal(const robosort_interfaces__srv__RotateBin_Event * lhs, const robosort_interfaces__srv__RotateBin_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robosort_interfaces__srv__RotateBin_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robosort_interfaces__srv__RotateBin_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robosort_interfaces__srv__RotateBin_Event__copy(
  const robosort_interfaces__srv__RotateBin_Event * input,
  robosort_interfaces__srv__RotateBin_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robosort_interfaces__srv__RotateBin_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robosort_interfaces__srv__RotateBin_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robosort_interfaces__srv__RotateBin_Event *
robosort_interfaces__srv__RotateBin_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Event * msg = (robosort_interfaces__srv__RotateBin_Event *)allocator.allocate(sizeof(robosort_interfaces__srv__RotateBin_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robosort_interfaces__srv__RotateBin_Event));
  bool success = robosort_interfaces__srv__RotateBin_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robosort_interfaces__srv__RotateBin_Event__destroy(robosort_interfaces__srv__RotateBin_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robosort_interfaces__srv__RotateBin_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robosort_interfaces__srv__RotateBin_Event__Sequence__init(robosort_interfaces__srv__RotateBin_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Event * data = NULL;

  if (size) {
    data = (robosort_interfaces__srv__RotateBin_Event *)allocator.zero_allocate(size, sizeof(robosort_interfaces__srv__RotateBin_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robosort_interfaces__srv__RotateBin_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robosort_interfaces__srv__RotateBin_Event__fini(&data[i - 1]);
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
robosort_interfaces__srv__RotateBin_Event__Sequence__fini(robosort_interfaces__srv__RotateBin_Event__Sequence * array)
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
      robosort_interfaces__srv__RotateBin_Event__fini(&array->data[i]);
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

robosort_interfaces__srv__RotateBin_Event__Sequence *
robosort_interfaces__srv__RotateBin_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robosort_interfaces__srv__RotateBin_Event__Sequence * array = (robosort_interfaces__srv__RotateBin_Event__Sequence *)allocator.allocate(sizeof(robosort_interfaces__srv__RotateBin_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robosort_interfaces__srv__RotateBin_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robosort_interfaces__srv__RotateBin_Event__Sequence__destroy(robosort_interfaces__srv__RotateBin_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robosort_interfaces__srv__RotateBin_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robosort_interfaces__srv__RotateBin_Event__Sequence__are_equal(const robosort_interfaces__srv__RotateBin_Event__Sequence * lhs, const robosort_interfaces__srv__RotateBin_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robosort_interfaces__srv__RotateBin_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robosort_interfaces__srv__RotateBin_Event__Sequence__copy(
  const robosort_interfaces__srv__RotateBin_Event__Sequence * input,
  robosort_interfaces__srv__RotateBin_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robosort_interfaces__srv__RotateBin_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robosort_interfaces__srv__RotateBin_Event * data =
      (robosort_interfaces__srv__RotateBin_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robosort_interfaces__srv__RotateBin_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robosort_interfaces__srv__RotateBin_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robosort_interfaces__srv__RotateBin_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
