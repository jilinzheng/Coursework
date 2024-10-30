// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from me416_msgs:msg/MotorSpeedsStamped.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__STRUCT_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__me416_msgs__msg__MotorSpeedsStamped __attribute__((deprecated))
#else
# define DEPRECATED__me416_msgs__msg__MotorSpeedsStamped __declspec(deprecated)
#endif

namespace me416_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSpeedsStamped_
{
  using Type = MotorSpeedsStamped_<ContainerAllocator>;

  explicit MotorSpeedsStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->left = 0.0;
      this->right = 0.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->left = 0.0;
      this->right = 0.0;
    }
  }

  explicit MotorSpeedsStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->left = 0.0;
      this->right = 0.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->left = 0.0;
      this->right = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_type =
    double;
  _left_type left;
  using _right_type =
    double;
  _right_type right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left(
    const double & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const double & _arg)
  {
    this->right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__me416_msgs__msg__MotorSpeedsStamped
    std::shared_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__me416_msgs__msg__MotorSpeedsStamped
    std::shared_ptr<me416_msgs::msg::MotorSpeedsStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSpeedsStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSpeedsStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSpeedsStamped_

// alias to use template instance with default allocator
using MotorSpeedsStamped =
  me416_msgs::msg::MotorSpeedsStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace me416_msgs

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__STRUCT_HPP_
