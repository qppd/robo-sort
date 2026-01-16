// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robosort_interfaces:srv/MoveRobotArm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/move_robot_arm.hpp"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__MOVE_ROBOT_ARM__TRAITS_HPP_
#define ROBOSORT_INTERFACES__SRV__DETAIL__MOVE_ROBOT_ARM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robosort_interfaces/srv/detail/move_robot_arm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robosort_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveRobotArm_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_angles
  {
    if (msg.joint_angles.size() == 0) {
      out << "joint_angles: []";
    } else {
      out << "joint_angles: [";
      size_t pending_items = msg.joint_angles.size();
      for (auto item : msg.joint_angles) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveRobotArm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_angles.size() == 0) {
      out << "joint_angles: []\n";
    } else {
      out << "joint_angles:\n";
      for (auto item : msg.joint_angles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveRobotArm_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robosort_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robosort_interfaces::srv::MoveRobotArm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robosort_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robosort_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robosort_interfaces::srv::MoveRobotArm_Request & msg)
{
  return robosort_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robosort_interfaces::srv::MoveRobotArm_Request>()
{
  return "robosort_interfaces::srv::MoveRobotArm_Request";
}

template<>
inline const char * name<robosort_interfaces::srv::MoveRobotArm_Request>()
{
  return "robosort_interfaces/srv/MoveRobotArm_Request";
}

template<>
struct has_fixed_size<robosort_interfaces::srv::MoveRobotArm_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robosort_interfaces::srv::MoveRobotArm_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robosort_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveRobotArm_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveRobotArm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveRobotArm_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robosort_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robosort_interfaces::srv::MoveRobotArm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robosort_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robosort_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robosort_interfaces::srv::MoveRobotArm_Response & msg)
{
  return robosort_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robosort_interfaces::srv::MoveRobotArm_Response>()
{
  return "robosort_interfaces::srv::MoveRobotArm_Response";
}

template<>
inline const char * name<robosort_interfaces::srv::MoveRobotArm_Response>()
{
  return "robosort_interfaces/srv/MoveRobotArm_Response";
}

template<>
struct has_fixed_size<robosort_interfaces::srv::MoveRobotArm_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robosort_interfaces::srv::MoveRobotArm_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace robosort_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveRobotArm_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveRobotArm_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveRobotArm_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robosort_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robosort_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robosort_interfaces::srv::MoveRobotArm_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  robosort_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robosort_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robosort_interfaces::srv::MoveRobotArm_Event & msg)
{
  return robosort_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robosort_interfaces::srv::MoveRobotArm_Event>()
{
  return "robosort_interfaces::srv::MoveRobotArm_Event";
}

template<>
inline const char * name<robosort_interfaces::srv::MoveRobotArm_Event>()
{
  return "robosort_interfaces/srv/MoveRobotArm_Event";
}

template<>
struct has_fixed_size<robosort_interfaces::srv::MoveRobotArm_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Event>
  : std::integral_constant<bool, has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Request>::value && has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<robosort_interfaces::srv::MoveRobotArm_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robosort_interfaces::srv::MoveRobotArm>()
{
  return "robosort_interfaces::srv::MoveRobotArm";
}

template<>
inline const char * name<robosort_interfaces::srv::MoveRobotArm>()
{
  return "robosort_interfaces/srv/MoveRobotArm";
}

template<>
struct has_fixed_size<robosort_interfaces::srv::MoveRobotArm>
  : std::integral_constant<
    bool,
    has_fixed_size<robosort_interfaces::srv::MoveRobotArm_Request>::value &&
    has_fixed_size<robosort_interfaces::srv::MoveRobotArm_Response>::value
  >
{
};

template<>
struct has_bounded_size<robosort_interfaces::srv::MoveRobotArm>
  : std::integral_constant<
    bool,
    has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Request>::value &&
    has_bounded_size<robosort_interfaces::srv::MoveRobotArm_Response>::value
  >
{
};

template<>
struct is_service<robosort_interfaces::srv::MoveRobotArm>
  : std::true_type
{
};

template<>
struct is_service_request<robosort_interfaces::srv::MoveRobotArm_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robosort_interfaces::srv::MoveRobotArm_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__MOVE_ROBOT_ARM__TRAITS_HPP_
