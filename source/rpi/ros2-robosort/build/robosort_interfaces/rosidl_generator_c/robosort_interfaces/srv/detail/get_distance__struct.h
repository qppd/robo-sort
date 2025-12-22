// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robosort_interfaces:srv/GetDistance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/get_distance.h"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__STRUCT_H_
#define ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetDistance in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__GetDistance_Request
{
  /// Sensor ID (1-4)
  int32_t sensor_id;
} robosort_interfaces__srv__GetDistance_Request;

// Struct for a sequence of robosort_interfaces__srv__GetDistance_Request.
typedef struct robosort_interfaces__srv__GetDistance_Request__Sequence
{
  robosort_interfaces__srv__GetDistance_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__GetDistance_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/GetDistance in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__GetDistance_Response
{
  /// Distance in cm
  float distance;
  bool success;
} robosort_interfaces__srv__GetDistance_Response;

// Struct for a sequence of robosort_interfaces__srv__GetDistance_Response.
typedef struct robosort_interfaces__srv__GetDistance_Response__Sequence
{
  robosort_interfaces__srv__GetDistance_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__GetDistance_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robosort_interfaces__srv__GetDistance_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robosort_interfaces__srv__GetDistance_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetDistance in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__GetDistance_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robosort_interfaces__srv__GetDistance_Request__Sequence request;
  robosort_interfaces__srv__GetDistance_Response__Sequence response;
} robosort_interfaces__srv__GetDistance_Event;

// Struct for a sequence of robosort_interfaces__srv__GetDistance_Event.
typedef struct robosort_interfaces__srv__GetDistance_Event__Sequence
{
  robosort_interfaces__srv__GetDistance_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__GetDistance_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__STRUCT_H_
