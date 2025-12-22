// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robosort_interfaces:srv/GetDistance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/get_distance.hpp"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__BUILDER_HPP_
#define ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robosort_interfaces/srv/detail/get_distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDistance_Request_sensor_id
{
public:
  Init_GetDistance_Request_sensor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robosort_interfaces::srv::GetDistance_Request sensor_id(::robosort_interfaces::srv::GetDistance_Request::_sensor_id_type arg)
  {
    msg_.sensor_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::GetDistance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::GetDistance_Request>()
{
  return robosort_interfaces::srv::builder::Init_GetDistance_Request_sensor_id();
}

}  // namespace robosort_interfaces


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDistance_Response_success
{
public:
  explicit Init_GetDistance_Response_success(::robosort_interfaces::srv::GetDistance_Response & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::GetDistance_Response success(::robosort_interfaces::srv::GetDistance_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::GetDistance_Response msg_;
};

class Init_GetDistance_Response_distance
{
public:
  Init_GetDistance_Response_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDistance_Response_success distance(::robosort_interfaces::srv::GetDistance_Response::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_GetDistance_Response_success(msg_);
  }

private:
  ::robosort_interfaces::srv::GetDistance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::GetDistance_Response>()
{
  return robosort_interfaces::srv::builder::Init_GetDistance_Response_distance();
}

}  // namespace robosort_interfaces


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetDistance_Event_response
{
public:
  explicit Init_GetDistance_Event_response(::robosort_interfaces::srv::GetDistance_Event & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::GetDistance_Event response(::robosort_interfaces::srv::GetDistance_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::GetDistance_Event msg_;
};

class Init_GetDistance_Event_request
{
public:
  explicit Init_GetDistance_Event_request(::robosort_interfaces::srv::GetDistance_Event & msg)
  : msg_(msg)
  {}
  Init_GetDistance_Event_response request(::robosort_interfaces::srv::GetDistance_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetDistance_Event_response(msg_);
  }

private:
  ::robosort_interfaces::srv::GetDistance_Event msg_;
};

class Init_GetDistance_Event_info
{
public:
  Init_GetDistance_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDistance_Event_request info(::robosort_interfaces::srv::GetDistance_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetDistance_Event_request(msg_);
  }

private:
  ::robosort_interfaces::srv::GetDistance_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::GetDistance_Event>()
{
  return robosort_interfaces::srv::builder::Init_GetDistance_Event_info();
}

}  // namespace robosort_interfaces

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__BUILDER_HPP_
