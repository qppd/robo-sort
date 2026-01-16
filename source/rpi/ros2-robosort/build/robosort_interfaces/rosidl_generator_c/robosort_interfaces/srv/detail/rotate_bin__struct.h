// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robosort_interfaces:srv/RotateBin.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/rotate_bin.h"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__ROTATE_BIN__STRUCT_H_
#define ROBOSORT_INTERFACES__SRV__DETAIL__ROTATE_BIN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RotateBin in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__RotateBin_Request
{
  /// Compartment 0-3 (0=paper, 1=plastic, 2=metal, 3=other)
  int32_t compartment_number;
} robosort_interfaces__srv__RotateBin_Request;

// Struct for a sequence of robosort_interfaces__srv__RotateBin_Request.
typedef struct robosort_interfaces__srv__RotateBin_Request__Sequence
{
  robosort_interfaces__srv__RotateBin_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__RotateBin_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RotateBin in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__RotateBin_Response
{
  bool success;
  rosidl_runtime_c__String message;
} robosort_interfaces__srv__RotateBin_Response;

// Struct for a sequence of robosort_interfaces__srv__RotateBin_Response.
typedef struct robosort_interfaces__srv__RotateBin_Response__Sequence
{
  robosort_interfaces__srv__RotateBin_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__RotateBin_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robosort_interfaces__srv__RotateBin_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robosort_interfaces__srv__RotateBin_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/RotateBin in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__RotateBin_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robosort_interfaces__srv__RotateBin_Request__Sequence request;
  robosort_interfaces__srv__RotateBin_Response__Sequence response;
} robosort_interfaces__srv__RotateBin_Event;

// Struct for a sequence of robosort_interfaces__srv__RotateBin_Event.
typedef struct robosort_interfaces__srv__RotateBin_Event__Sequence
{
  robosort_interfaces__srv__RotateBin_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__RotateBin_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__ROTATE_BIN__STRUCT_H_
