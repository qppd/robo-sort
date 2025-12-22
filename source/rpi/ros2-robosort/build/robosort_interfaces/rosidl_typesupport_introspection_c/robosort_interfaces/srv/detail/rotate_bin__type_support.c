// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robosort_interfaces:srv/RotateBin.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robosort_interfaces/srv/detail/rotate_bin__rosidl_typesupport_introspection_c.h"
#include "robosort_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robosort_interfaces/srv/detail/rotate_bin__functions.h"
#include "robosort_interfaces/srv/detail/rotate_bin__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robosort_interfaces__srv__RotateBin_Request__init(message_memory);
}

void robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_fini_function(void * message_memory)
{
  robosort_interfaces__srv__RotateBin_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_member_array[1] = {
  {
    "compartment_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robosort_interfaces__srv__RotateBin_Request, compartment_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_members = {
  "robosort_interfaces__srv",  // message namespace
  "RotateBin_Request",  // message name
  1,  // number of fields
  sizeof(robosort_interfaces__srv__RotateBin_Request),
  false,  // has_any_key_member_
  robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_member_array,  // message members
  robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_type_support_handle = {
  0,
  &robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_members,
  get_message_typesupport_handle_function,
  &robosort_interfaces__srv__RotateBin_Request__get_type_hash,
  &robosort_interfaces__srv__RotateBin_Request__get_type_description,
  &robosort_interfaces__srv__RotateBin_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robosort_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Request)() {
  if (!robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_type_support_handle.typesupport_identifier) {
    robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robosort_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__functions.h"
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robosort_interfaces__srv__RotateBin_Response__init(message_memory);
}

void robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_fini_function(void * message_memory)
{
  robosort_interfaces__srv__RotateBin_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robosort_interfaces__srv__RotateBin_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robosort_interfaces__srv__RotateBin_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_members = {
  "robosort_interfaces__srv",  // message namespace
  "RotateBin_Response",  // message name
  2,  // number of fields
  sizeof(robosort_interfaces__srv__RotateBin_Response),
  false,  // has_any_key_member_
  robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_member_array,  // message members
  robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle = {
  0,
  &robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_members,
  get_message_typesupport_handle_function,
  &robosort_interfaces__srv__RotateBin_Response__get_type_hash,
  &robosort_interfaces__srv__RotateBin_Response__get_type_description,
  &robosort_interfaces__srv__RotateBin_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robosort_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Response)() {
  if (!robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle.typesupport_identifier) {
    robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robosort_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__functions.h"
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "robosort_interfaces/srv/rotate_bin.h"
// Member `request`
// Member `response`
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robosort_interfaces__srv__RotateBin_Event__init(message_memory);
}

void robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_fini_function(void * message_memory)
{
  robosort_interfaces__srv__RotateBin_Event__fini(message_memory);
}

size_t robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__size_function__RotateBin_Event__request(
  const void * untyped_member)
{
  const robosort_interfaces__srv__RotateBin_Request__Sequence * member =
    (const robosort_interfaces__srv__RotateBin_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_const_function__RotateBin_Event__request(
  const void * untyped_member, size_t index)
{
  const robosort_interfaces__srv__RotateBin_Request__Sequence * member =
    (const robosort_interfaces__srv__RotateBin_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_function__RotateBin_Event__request(
  void * untyped_member, size_t index)
{
  robosort_interfaces__srv__RotateBin_Request__Sequence * member =
    (robosort_interfaces__srv__RotateBin_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__fetch_function__RotateBin_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robosort_interfaces__srv__RotateBin_Request * item =
    ((const robosort_interfaces__srv__RotateBin_Request *)
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_const_function__RotateBin_Event__request(untyped_member, index));
  robosort_interfaces__srv__RotateBin_Request * value =
    (robosort_interfaces__srv__RotateBin_Request *)(untyped_value);
  *value = *item;
}

void robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__assign_function__RotateBin_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robosort_interfaces__srv__RotateBin_Request * item =
    ((robosort_interfaces__srv__RotateBin_Request *)
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_function__RotateBin_Event__request(untyped_member, index));
  const robosort_interfaces__srv__RotateBin_Request * value =
    (const robosort_interfaces__srv__RotateBin_Request *)(untyped_value);
  *item = *value;
}

bool robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__resize_function__RotateBin_Event__request(
  void * untyped_member, size_t size)
{
  robosort_interfaces__srv__RotateBin_Request__Sequence * member =
    (robosort_interfaces__srv__RotateBin_Request__Sequence *)(untyped_member);
  robosort_interfaces__srv__RotateBin_Request__Sequence__fini(member);
  return robosort_interfaces__srv__RotateBin_Request__Sequence__init(member, size);
}

size_t robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__size_function__RotateBin_Event__response(
  const void * untyped_member)
{
  const robosort_interfaces__srv__RotateBin_Response__Sequence * member =
    (const robosort_interfaces__srv__RotateBin_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_const_function__RotateBin_Event__response(
  const void * untyped_member, size_t index)
{
  const robosort_interfaces__srv__RotateBin_Response__Sequence * member =
    (const robosort_interfaces__srv__RotateBin_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_function__RotateBin_Event__response(
  void * untyped_member, size_t index)
{
  robosort_interfaces__srv__RotateBin_Response__Sequence * member =
    (robosort_interfaces__srv__RotateBin_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__fetch_function__RotateBin_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robosort_interfaces__srv__RotateBin_Response * item =
    ((const robosort_interfaces__srv__RotateBin_Response *)
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_const_function__RotateBin_Event__response(untyped_member, index));
  robosort_interfaces__srv__RotateBin_Response * value =
    (robosort_interfaces__srv__RotateBin_Response *)(untyped_value);
  *value = *item;
}

void robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__assign_function__RotateBin_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robosort_interfaces__srv__RotateBin_Response * item =
    ((robosort_interfaces__srv__RotateBin_Response *)
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_function__RotateBin_Event__response(untyped_member, index));
  const robosort_interfaces__srv__RotateBin_Response * value =
    (const robosort_interfaces__srv__RotateBin_Response *)(untyped_value);
  *item = *value;
}

bool robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__resize_function__RotateBin_Event__response(
  void * untyped_member, size_t size)
{
  robosort_interfaces__srv__RotateBin_Response__Sequence * member =
    (robosort_interfaces__srv__RotateBin_Response__Sequence *)(untyped_member);
  robosort_interfaces__srv__RotateBin_Response__Sequence__fini(member);
  return robosort_interfaces__srv__RotateBin_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robosort_interfaces__srv__RotateBin_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robosort_interfaces__srv__RotateBin_Event, request),  // bytes offset in struct
    NULL,  // default value
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__size_function__RotateBin_Event__request,  // size() function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_const_function__RotateBin_Event__request,  // get_const(index) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_function__RotateBin_Event__request,  // get(index) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__fetch_function__RotateBin_Event__request,  // fetch(index, &value) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__assign_function__RotateBin_Event__request,  // assign(index, value) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__resize_function__RotateBin_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robosort_interfaces__srv__RotateBin_Event, response),  // bytes offset in struct
    NULL,  // default value
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__size_function__RotateBin_Event__response,  // size() function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_const_function__RotateBin_Event__response,  // get_const(index) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__get_function__RotateBin_Event__response,  // get(index) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__fetch_function__RotateBin_Event__response,  // fetch(index, &value) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__assign_function__RotateBin_Event__response,  // assign(index, value) function pointer
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__resize_function__RotateBin_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_members = {
  "robosort_interfaces__srv",  // message namespace
  "RotateBin_Event",  // message name
  3,  // number of fields
  sizeof(robosort_interfaces__srv__RotateBin_Event),
  false,  // has_any_key_member_
  robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_member_array,  // message members
  robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_type_support_handle = {
  0,
  &robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_members,
  get_message_typesupport_handle_function,
  &robosort_interfaces__srv__RotateBin_Event__get_type_hash,
  &robosort_interfaces__srv__RotateBin_Event__get_type_description,
  &robosort_interfaces__srv__RotateBin_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robosort_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Event)() {
  robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Request)();
  robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Response)();
  if (!robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_type_support_handle.typesupport_identifier) {
    robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robosort_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robosort_interfaces/srv/detail/rotate_bin__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_members = {
  "robosort_interfaces__srv",  // service namespace
  "RotateBin",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_Request_message_type_support_handle,
  NULL,  // response message
  // robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle
  NULL  // event_message
  // robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle
};


static rosidl_service_type_support_t robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_type_support_handle = {
  0,
  &robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_members,
  get_service_typesupport_handle_function,
  &robosort_interfaces__srv__RotateBin_Request__rosidl_typesupport_introspection_c__RotateBin_Request_message_type_support_handle,
  &robosort_interfaces__srv__RotateBin_Response__rosidl_typesupport_introspection_c__RotateBin_Response_message_type_support_handle,
  &robosort_interfaces__srv__RotateBin_Event__rosidl_typesupport_introspection_c__RotateBin_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robosort_interfaces,
    srv,
    RotateBin
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robosort_interfaces,
    srv,
    RotateBin
  ),
  &robosort_interfaces__srv__RotateBin__get_type_hash,
  &robosort_interfaces__srv__RotateBin__get_type_description,
  &robosort_interfaces__srv__RotateBin__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robosort_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin)(void) {
  if (!robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_type_support_handle.typesupport_identifier) {
    robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robosort_interfaces, srv, RotateBin_Event)()->data;
  }

  return &robosort_interfaces__srv__detail__rotate_bin__rosidl_typesupport_introspection_c__RotateBin_service_type_support_handle;
}
