// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robosort_interfaces:srv/SetServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/set_servo.hpp"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__SET_SERVO__BUILDER_HPP_
#define ROBOSORT_INTERFACES__SRV__DETAIL__SET_SERVO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robosort_interfaces/srv/detail/set_servo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetServo_Request_angle
{
public:
  explicit Init_SetServo_Request_angle(::robosort_interfaces::srv::SetServo_Request & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::SetServo_Request angle(::robosort_interfaces::srv::SetServo_Request::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Request msg_;
};

class Init_SetServo_Request_servo_num
{
public:
  Init_SetServo_Request_servo_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetServo_Request_angle servo_num(::robosort_interfaces::srv::SetServo_Request::_servo_num_type arg)
  {
    msg_.servo_num = std::move(arg);
    return Init_SetServo_Request_angle(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::SetServo_Request>()
{
  return robosort_interfaces::srv::builder::Init_SetServo_Request_servo_num();
}

}  // namespace robosort_interfaces


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetServo_Response_message
{
public:
  explicit Init_SetServo_Response_message(::robosort_interfaces::srv::SetServo_Response & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::SetServo_Response message(::robosort_interfaces::srv::SetServo_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Response msg_;
};

class Init_SetServo_Response_success
{
public:
  Init_SetServo_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetServo_Response_message success(::robosort_interfaces::srv::SetServo_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetServo_Response_message(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::SetServo_Response>()
{
  return robosort_interfaces::srv::builder::Init_SetServo_Response_success();
}

}  // namespace robosort_interfaces


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetServo_Event_response
{
public:
  explicit Init_SetServo_Event_response(::robosort_interfaces::srv::SetServo_Event & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::SetServo_Event response(::robosort_interfaces::srv::SetServo_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Event msg_;
};

class Init_SetServo_Event_request
{
public:
  explicit Init_SetServo_Event_request(::robosort_interfaces::srv::SetServo_Event & msg)
  : msg_(msg)
  {}
  Init_SetServo_Event_response request(::robosort_interfaces::srv::SetServo_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetServo_Event_response(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Event msg_;
};

class Init_SetServo_Event_info
{
public:
  Init_SetServo_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetServo_Event_request info(::robosort_interfaces::srv::SetServo_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetServo_Event_request(msg_);
  }

private:
  ::robosort_interfaces::srv::SetServo_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::SetServo_Event>()
{
  return robosort_interfaces::srv::builder::Init_SetServo_Event_info();
}

}  // namespace robosort_interfaces

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__SET_SERVO__BUILDER_HPP_
