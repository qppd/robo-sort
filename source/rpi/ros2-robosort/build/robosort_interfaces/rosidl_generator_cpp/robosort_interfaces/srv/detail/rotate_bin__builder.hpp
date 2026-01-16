// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robosort_interfaces:srv/RotateBin.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/rotate_bin.hpp"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__ROTATE_BIN__BUILDER_HPP_
#define ROBOSORT_INTERFACES__SRV__DETAIL__ROTATE_BIN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robosort_interfaces/srv/detail/rotate_bin__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_RotateBin_Request_compartment_number
{
public:
  Init_RotateBin_Request_compartment_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robosort_interfaces::srv::RotateBin_Request compartment_number(::robosort_interfaces::srv::RotateBin_Request::_compartment_number_type arg)
  {
    msg_.compartment_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::RotateBin_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::RotateBin_Request>()
{
  return robosort_interfaces::srv::builder::Init_RotateBin_Request_compartment_number();
}

}  // namespace robosort_interfaces


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_RotateBin_Response_message
{
public:
  explicit Init_RotateBin_Response_message(::robosort_interfaces::srv::RotateBin_Response & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::RotateBin_Response message(::robosort_interfaces::srv::RotateBin_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::RotateBin_Response msg_;
};

class Init_RotateBin_Response_success
{
public:
  Init_RotateBin_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RotateBin_Response_message success(::robosort_interfaces::srv::RotateBin_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RotateBin_Response_message(msg_);
  }

private:
  ::robosort_interfaces::srv::RotateBin_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::RotateBin_Response>()
{
  return robosort_interfaces::srv::builder::Init_RotateBin_Response_success();
}

}  // namespace robosort_interfaces


namespace robosort_interfaces
{

namespace srv
{

namespace builder
{

class Init_RotateBin_Event_response
{
public:
  explicit Init_RotateBin_Event_response(::robosort_interfaces::srv::RotateBin_Event & msg)
  : msg_(msg)
  {}
  ::robosort_interfaces::srv::RotateBin_Event response(::robosort_interfaces::srv::RotateBin_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robosort_interfaces::srv::RotateBin_Event msg_;
};

class Init_RotateBin_Event_request
{
public:
  explicit Init_RotateBin_Event_request(::robosort_interfaces::srv::RotateBin_Event & msg)
  : msg_(msg)
  {}
  Init_RotateBin_Event_response request(::robosort_interfaces::srv::RotateBin_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RotateBin_Event_response(msg_);
  }

private:
  ::robosort_interfaces::srv::RotateBin_Event msg_;
};

class Init_RotateBin_Event_info
{
public:
  Init_RotateBin_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RotateBin_Event_request info(::robosort_interfaces::srv::RotateBin_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RotateBin_Event_request(msg_);
  }

private:
  ::robosort_interfaces::srv::RotateBin_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robosort_interfaces::srv::RotateBin_Event>()
{
  return robosort_interfaces::srv::builder::Init_RotateBin_Event_info();
}

}  // namespace robosort_interfaces

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__ROTATE_BIN__BUILDER_HPP_
