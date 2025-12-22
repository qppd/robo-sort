// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robosort_interfaces:srv/GetDistance.idl
// generated code does not contain a copyright notice

#include "robosort_interfaces/srv/detail/get_distance__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robosort_interfaces
const rosidl_type_hash_t *
robosort_interfaces__srv__GetDistance__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd8, 0x3f, 0xec, 0x9d, 0xa9, 0xac, 0x0d, 0x55,
      0xfe, 0x5f, 0x91, 0xea, 0x3c, 0xc2, 0xce, 0x4e,
      0x6b, 0xba, 0x89, 0x96, 0x75, 0xc6, 0xc2, 0xa1,
      0x26, 0x4c, 0x8f, 0x8d, 0x1f, 0xd8, 0x0a, 0x85,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robosort_interfaces
const rosidl_type_hash_t *
robosort_interfaces__srv__GetDistance_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5b, 0x14, 0xfe, 0xfc, 0x86, 0xde, 0x2a, 0xd7,
      0x48, 0xd1, 0x95, 0xa7, 0xdc, 0x77, 0x91, 0x10,
      0xce, 0x55, 0x25, 0x08, 0xd9, 0xea, 0xa5, 0x3a,
      0xe0, 0x48, 0x8d, 0xd6, 0xa5, 0x88, 0x85, 0x56,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robosort_interfaces
const rosidl_type_hash_t *
robosort_interfaces__srv__GetDistance_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0x07, 0xc1, 0xde, 0x16, 0x9b, 0xef, 0xcf,
      0x4d, 0x69, 0x96, 0xf1, 0x69, 0x38, 0xd1, 0xa8,
      0x2b, 0x35, 0xa6, 0x6e, 0xcc, 0x03, 0x6f, 0x72,
      0x3f, 0x50, 0x6f, 0x6f, 0x9a, 0x6d, 0xce, 0xe7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robosort_interfaces
const rosidl_type_hash_t *
robosort_interfaces__srv__GetDistance_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9d, 0xbf, 0x3f, 0x2f, 0x37, 0x09, 0x18, 0x19,
      0xc2, 0x92, 0x42, 0xef, 0x85, 0x8e, 0xd8, 0x5a,
      0xf0, 0x9a, 0x02, 0xc1, 0x01, 0xa1, 0xd1, 0x3d,
      0x0e, 0x9d, 0x81, 0x5e, 0x35, 0x25, 0x27, 0x28,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char robosort_interfaces__srv__GetDistance__TYPE_NAME[] = "robosort_interfaces/srv/GetDistance";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robosort_interfaces__srv__GetDistance_Event__TYPE_NAME[] = "robosort_interfaces/srv/GetDistance_Event";
static char robosort_interfaces__srv__GetDistance_Request__TYPE_NAME[] = "robosort_interfaces/srv/GetDistance_Request";
static char robosort_interfaces__srv__GetDistance_Response__TYPE_NAME[] = "robosort_interfaces/srv/GetDistance_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robosort_interfaces__srv__GetDistance__FIELD_NAME__request_message[] = "request_message";
static char robosort_interfaces__srv__GetDistance__FIELD_NAME__response_message[] = "response_message";
static char robosort_interfaces__srv__GetDistance__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robosort_interfaces__srv__GetDistance__FIELDS[] = {
  {
    {robosort_interfaces__srv__GetDistance__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robosort_interfaces__srv__GetDistance_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robosort_interfaces__srv__GetDistance_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robosort_interfaces__srv__GetDistance_Event__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robosort_interfaces__srv__GetDistance__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Event__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Response__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robosort_interfaces__srv__GetDistance__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robosort_interfaces__srv__GetDistance__TYPE_NAME, 35, 35},
      {robosort_interfaces__srv__GetDistance__FIELDS, 3, 3},
    },
    {robosort_interfaces__srv__GetDistance__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robosort_interfaces__srv__GetDistance_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robosort_interfaces__srv__GetDistance_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robosort_interfaces__srv__GetDistance_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robosort_interfaces__srv__GetDistance_Request__FIELD_NAME__sensor_id[] = "sensor_id";

static rosidl_runtime_c__type_description__Field robosort_interfaces__srv__GetDistance_Request__FIELDS[] = {
  {
    {robosort_interfaces__srv__GetDistance_Request__FIELD_NAME__sensor_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robosort_interfaces__srv__GetDistance_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robosort_interfaces__srv__GetDistance_Request__TYPE_NAME, 43, 43},
      {robosort_interfaces__srv__GetDistance_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robosort_interfaces__srv__GetDistance_Response__FIELD_NAME__distance[] = "distance";
static char robosort_interfaces__srv__GetDistance_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field robosort_interfaces__srv__GetDistance_Response__FIELDS[] = {
  {
    {robosort_interfaces__srv__GetDistance_Response__FIELD_NAME__distance, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robosort_interfaces__srv__GetDistance_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robosort_interfaces__srv__GetDistance_Response__TYPE_NAME, 44, 44},
      {robosort_interfaces__srv__GetDistance_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robosort_interfaces__srv__GetDistance_Event__FIELD_NAME__info[] = "info";
static char robosort_interfaces__srv__GetDistance_Event__FIELD_NAME__request[] = "request";
static char robosort_interfaces__srv__GetDistance_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robosort_interfaces__srv__GetDistance_Event__FIELDS[] = {
  {
    {robosort_interfaces__srv__GetDistance_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robosort_interfaces__srv__GetDistance_Request__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robosort_interfaces__srv__GetDistance_Response__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robosort_interfaces__srv__GetDistance_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Request__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {robosort_interfaces__srv__GetDistance_Response__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robosort_interfaces__srv__GetDistance_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robosort_interfaces__srv__GetDistance_Event__TYPE_NAME, 41, 41},
      {robosort_interfaces__srv__GetDistance_Event__FIELDS, 3, 3},
    },
    {robosort_interfaces__srv__GetDistance_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robosort_interfaces__srv__GetDistance_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robosort_interfaces__srv__GetDistance_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Service to get distance from ultrasonic sensor\n"
  "int32 sensor_id  # Sensor ID (1-4)\n"
  "---\n"
  "float32 distance  # Distance in cm\n"
  "bool success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robosort_interfaces__srv__GetDistance__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robosort_interfaces__srv__GetDistance__TYPE_NAME, 35, 35},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 136, 136},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robosort_interfaces__srv__GetDistance_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robosort_interfaces__srv__GetDistance_Request__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robosort_interfaces__srv__GetDistance_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robosort_interfaces__srv__GetDistance_Response__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robosort_interfaces__srv__GetDistance_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robosort_interfaces__srv__GetDistance_Event__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robosort_interfaces__srv__GetDistance__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robosort_interfaces__srv__GetDistance__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robosort_interfaces__srv__GetDistance_Event__get_individual_type_description_source(NULL);
    sources[3] = *robosort_interfaces__srv__GetDistance_Request__get_individual_type_description_source(NULL);
    sources[4] = *robosort_interfaces__srv__GetDistance_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robosort_interfaces__srv__GetDistance_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robosort_interfaces__srv__GetDistance_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robosort_interfaces__srv__GetDistance_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robosort_interfaces__srv__GetDistance_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robosort_interfaces__srv__GetDistance_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robosort_interfaces__srv__GetDistance_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robosort_interfaces__srv__GetDistance_Request__get_individual_type_description_source(NULL);
    sources[3] = *robosort_interfaces__srv__GetDistance_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
