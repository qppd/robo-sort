// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robosort_interfaces:srv/GetDistance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robosort_interfaces/srv/get_distance.hpp"


#ifndef ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__STRUCT_HPP_
#define ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robosort_interfaces__srv__GetDistance_Request __attribute__((deprecated))
#else
# define DEPRECATED__robosort_interfaces__srv__GetDistance_Request __declspec(deprecated)
#endif

namespace robosort_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDistance_Request_
{
  using Type = GetDistance_Request_<ContainerAllocator>;

  explicit GetDistance_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_id = 0l;
    }
  }

  explicit GetDistance_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_id = 0l;
    }
  }

  // field types and members
  using _sensor_id_type =
    int32_t;
  _sensor_id_type sensor_id;

  // setters for named parameter idiom
  Type & set__sensor_id(
    const int32_t & _arg)
  {
    this->sensor_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robosort_interfaces__srv__GetDistance_Request
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robosort_interfaces__srv__GetDistance_Request
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDistance_Request_ & other) const
  {
    if (this->sensor_id != other.sensor_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDistance_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDistance_Request_

// alias to use template instance with default allocator
using GetDistance_Request =
  robosort_interfaces::srv::GetDistance_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robosort_interfaces


#ifndef _WIN32
# define DEPRECATED__robosort_interfaces__srv__GetDistance_Response __attribute__((deprecated))
#else
# define DEPRECATED__robosort_interfaces__srv__GetDistance_Response __declspec(deprecated)
#endif

namespace robosort_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDistance_Response_
{
  using Type = GetDistance_Response_<ContainerAllocator>;

  explicit GetDistance_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0f;
      this->success = false;
    }
  }

  explicit GetDistance_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0f;
      this->success = false;
    }
  }

  // field types and members
  using _distance_type =
    float;
  _distance_type distance;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robosort_interfaces__srv__GetDistance_Response
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robosort_interfaces__srv__GetDistance_Response
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDistance_Response_ & other) const
  {
    if (this->distance != other.distance) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDistance_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDistance_Response_

// alias to use template instance with default allocator
using GetDistance_Response =
  robosort_interfaces::srv::GetDistance_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robosort_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robosort_interfaces__srv__GetDistance_Event __attribute__((deprecated))
#else
# define DEPRECATED__robosort_interfaces__srv__GetDistance_Event __declspec(deprecated)
#endif

namespace robosort_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetDistance_Event_
{
  using Type = GetDistance_Event_<ContainerAllocator>;

  explicit GetDistance_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetDistance_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robosort_interfaces::srv::GetDistance_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robosort_interfaces::srv::GetDistance_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robosort_interfaces__srv__GetDistance_Event
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robosort_interfaces__srv__GetDistance_Event
    std::shared_ptr<robosort_interfaces::srv::GetDistance_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetDistance_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetDistance_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetDistance_Event_

// alias to use template instance with default allocator
using GetDistance_Event =
  robosort_interfaces::srv::GetDistance_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robosort_interfaces

namespace robosort_interfaces
{

namespace srv
{

struct GetDistance
{
  using Request = robosort_interfaces::srv::GetDistance_Request;
  using Response = robosort_interfaces::srv::GetDistance_Response;
  using Event = robosort_interfaces::srv::GetDistance_Event;
};

}  // namespace srv

}  // namespace robosort_interfaces

#endif  // ROBOSORT_INTERFACES__SRV__DETAIL__GET_DISTANCE__STRUCT_HPP_
