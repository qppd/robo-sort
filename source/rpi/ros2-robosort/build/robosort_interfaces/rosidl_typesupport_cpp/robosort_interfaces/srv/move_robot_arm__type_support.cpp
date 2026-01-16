// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from robosort_interfaces:srv/MoveRobotArm.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robosort_interfaces/srv/detail/move_robot_arm__functions.h"
#include "robosort_interfaces/srv/detail/move_robot_arm__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robosort_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveRobotArm_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveRobotArm_Request_type_support_ids_t;

static const _MoveRobotArm_Request_type_support_ids_t _MoveRobotArm_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveRobotArm_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveRobotArm_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveRobotArm_Request_type_support_symbol_names_t _MoveRobotArm_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robosort_interfaces, srv, MoveRobotArm_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robosort_interfaces, srv, MoveRobotArm_Request)),
  }
};

typedef struct _MoveRobotArm_Request_type_support_data_t
{
  void * data[2];
} _MoveRobotArm_Request_type_support_data_t;

static _MoveRobotArm_Request_type_support_data_t _MoveRobotArm_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveRobotArm_Request_message_typesupport_map = {
  2,
  "robosort_interfaces",
  &_MoveRobotArm_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MoveRobotArm_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MoveRobotArm_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveRobotArm_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveRobotArm_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robosort_interfaces__srv__MoveRobotArm_Request__get_type_hash,
  &robosort_interfaces__srv__MoveRobotArm_Request__get_type_description,
  &robosort_interfaces__srv__MoveRobotArm_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Request>()
{
  return &::robosort_interfaces::srv::rosidl_typesupport_cpp::MoveRobotArm_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robosort_interfaces, srv, MoveRobotArm_Request)() {
  return get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robosort_interfaces/srv/detail/move_robot_arm__functions.h"
// already included above
// #include "robosort_interfaces/srv/detail/move_robot_arm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robosort_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveRobotArm_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveRobotArm_Response_type_support_ids_t;

static const _MoveRobotArm_Response_type_support_ids_t _MoveRobotArm_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveRobotArm_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveRobotArm_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveRobotArm_Response_type_support_symbol_names_t _MoveRobotArm_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robosort_interfaces, srv, MoveRobotArm_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robosort_interfaces, srv, MoveRobotArm_Response)),
  }
};

typedef struct _MoveRobotArm_Response_type_support_data_t
{
  void * data[2];
} _MoveRobotArm_Response_type_support_data_t;

static _MoveRobotArm_Response_type_support_data_t _MoveRobotArm_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveRobotArm_Response_message_typesupport_map = {
  2,
  "robosort_interfaces",
  &_MoveRobotArm_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MoveRobotArm_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MoveRobotArm_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveRobotArm_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveRobotArm_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robosort_interfaces__srv__MoveRobotArm_Response__get_type_hash,
  &robosort_interfaces__srv__MoveRobotArm_Response__get_type_description,
  &robosort_interfaces__srv__MoveRobotArm_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Response>()
{
  return &::robosort_interfaces::srv::rosidl_typesupport_cpp::MoveRobotArm_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robosort_interfaces, srv, MoveRobotArm_Response)() {
  return get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robosort_interfaces/srv/detail/move_robot_arm__functions.h"
// already included above
// #include "robosort_interfaces/srv/detail/move_robot_arm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robosort_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveRobotArm_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveRobotArm_Event_type_support_ids_t;

static const _MoveRobotArm_Event_type_support_ids_t _MoveRobotArm_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveRobotArm_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveRobotArm_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveRobotArm_Event_type_support_symbol_names_t _MoveRobotArm_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robosort_interfaces, srv, MoveRobotArm_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robosort_interfaces, srv, MoveRobotArm_Event)),
  }
};

typedef struct _MoveRobotArm_Event_type_support_data_t
{
  void * data[2];
} _MoveRobotArm_Event_type_support_data_t;

static _MoveRobotArm_Event_type_support_data_t _MoveRobotArm_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveRobotArm_Event_message_typesupport_map = {
  2,
  "robosort_interfaces",
  &_MoveRobotArm_Event_message_typesupport_ids.typesupport_identifier[0],
  &_MoveRobotArm_Event_message_typesupport_symbol_names.symbol_name[0],
  &_MoveRobotArm_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveRobotArm_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveRobotArm_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robosort_interfaces__srv__MoveRobotArm_Event__get_type_hash,
  &robosort_interfaces__srv__MoveRobotArm_Event__get_type_description,
  &robosort_interfaces__srv__MoveRobotArm_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Event>()
{
  return &::robosort_interfaces::srv::rosidl_typesupport_cpp::MoveRobotArm_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robosort_interfaces, srv, MoveRobotArm_Event)() {
  return get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "robosort_interfaces/srv/detail/move_robot_arm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robosort_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveRobotArm_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveRobotArm_type_support_ids_t;

static const _MoveRobotArm_type_support_ids_t _MoveRobotArm_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveRobotArm_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveRobotArm_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveRobotArm_type_support_symbol_names_t _MoveRobotArm_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robosort_interfaces, srv, MoveRobotArm)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robosort_interfaces, srv, MoveRobotArm)),
  }
};

typedef struct _MoveRobotArm_type_support_data_t
{
  void * data[2];
} _MoveRobotArm_type_support_data_t;

static _MoveRobotArm_type_support_data_t _MoveRobotArm_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveRobotArm_service_typesupport_map = {
  2,
  "robosort_interfaces",
  &_MoveRobotArm_service_typesupport_ids.typesupport_identifier[0],
  &_MoveRobotArm_service_typesupport_symbol_names.symbol_name[0],
  &_MoveRobotArm_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MoveRobotArm_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveRobotArm_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robosort_interfaces::srv::MoveRobotArm_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<robosort_interfaces::srv::MoveRobotArm>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<robosort_interfaces::srv::MoveRobotArm>,
  &robosort_interfaces__srv__MoveRobotArm__get_type_hash,
  &robosort_interfaces__srv__MoveRobotArm__get_type_description,
  &robosort_interfaces__srv__MoveRobotArm__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robosort_interfaces::srv::MoveRobotArm>()
{
  return &::robosort_interfaces::srv::rosidl_typesupport_cpp::MoveRobotArm_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robosort_interfaces, srv, MoveRobotArm)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robosort_interfaces::srv::MoveRobotArm>();
}

#ifdef __cplusplus
}
#endif
