// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robosort_interfaces:srv/SetServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/set_servo.h"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__SET_SERVO__STRUCT_H_
#define ROBOSORT_INTERFACES__SRV__DETAIL__SET_SERVO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetServo in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__SetServo_Request
{
  /// Servo number (0-5): 0-4 for arm, 5 for lifter
  int32_t servo_num;
  /// Angle in degrees (0-180)
  int32_t angle;
} robosort_interfaces__srv__SetServo_Request;

// Struct for a sequence of robosort_interfaces__srv__SetServo_Request.
typedef struct robosort_interfaces__srv__SetServo_Request__Sequence
{
  robosort_interfaces__srv__SetServo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__SetServo_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetServo in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__SetServo_Response
{
  bool success;
  rosidl_runtime_c__String message;
} robosort_interfaces__srv__SetServo_Response;

// Struct for a sequence of robosort_interfaces__srv__SetServo_Response.
typedef struct robosort_interfaces__srv__SetServo_Response__Sequence
{
  robosort_interfaces__srv__SetServo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__SetServo_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robosort_interfaces__srv__SetServo_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robosort_interfaces__srv__SetServo_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetServo in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__SetServo_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robosort_interfaces__srv__SetServo_Request__Sequence request;
  robosort_interfaces__srv__SetServo_Response__Sequence response;
} robosort_interfaces__srv__SetServo_Event;

// Struct for a sequence of robosort_interfaces__srv__SetServo_Event.
typedef struct robosort_interfaces__srv__SetServo_Event__Sequence
{
  robosort_interfaces__srv__SetServo_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__SetServo_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__SET_SERVO__STRUCT_H_
