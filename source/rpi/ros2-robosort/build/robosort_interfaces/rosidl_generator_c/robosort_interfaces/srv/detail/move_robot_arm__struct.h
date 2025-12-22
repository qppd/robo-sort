// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robosort_interfaces:srv/MoveRobotArm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/move_robot_arm.h"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__MOVE_ROBOT_ARM__STRUCT_H_
#define ROBOSORT_INTERFACES__SRV__DETAIL__MOVE_ROBOT_ARM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_angles'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MoveRobotArm in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__MoveRobotArm_Request
{
  /// Joint angles [base, shoulder, elbow, wrist, gripper, lifter]
  rosidl_runtime_c__float__Sequence joint_angles;
} robosort_interfaces__srv__MoveRobotArm_Request;

// Struct for a sequence of robosort_interfaces__srv__MoveRobotArm_Request.
typedef struct robosort_interfaces__srv__MoveRobotArm_Request__Sequence
{
  robosort_interfaces__srv__MoveRobotArm_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__MoveRobotArm_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveRobotArm in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__MoveRobotArm_Response
{
  bool success;
  rosidl_runtime_c__String message;
} robosort_interfaces__srv__MoveRobotArm_Response;

// Struct for a sequence of robosort_interfaces__srv__MoveRobotArm_Response.
typedef struct robosort_interfaces__srv__MoveRobotArm_Response__Sequence
{
  robosort_interfaces__srv__MoveRobotArm_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__MoveRobotArm_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robosort_interfaces__srv__MoveRobotArm_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robosort_interfaces__srv__MoveRobotArm_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveRobotArm in the package robosort_interfaces.
typedef struct robosort_interfaces__srv__MoveRobotArm_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robosort_interfaces__srv__MoveRobotArm_Request__Sequence request;
  robosort_interfaces__srv__MoveRobotArm_Response__Sequence response;
} robosort_interfaces__srv__MoveRobotArm_Event;

// Struct for a sequence of robosort_interfaces__srv__MoveRobotArm_Event.
typedef struct robosort_interfaces__srv__MoveRobotArm_Event__Sequence
{
  robosort_interfaces__srv__MoveRobotArm_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robosort_interfaces__srv__MoveRobotArm_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__MOVE_ROBOT_ARM__STRUCT_H_
