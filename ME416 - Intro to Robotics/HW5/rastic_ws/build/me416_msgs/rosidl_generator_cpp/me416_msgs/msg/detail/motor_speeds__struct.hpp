// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from me416_msgs:msg/MotorSpeeds.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__STRUCT_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__me416_msgs__msg__MotorSpeeds __attribute__((deprecated))
#else
# define DEPRECATED__me416_msgs__msg__MotorSpeeds __declspec(deprecated)
#endif

namespace me416_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSpeeds_
{
  using Type = MotorSpeeds_<ContainerAllocator>;

  explicit MotorSpeeds_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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

  explicit MotorSpeeds_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
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
  using _left_type =
    double;
  _left_type left;
  using _right_type =
    double;
  _right_type right;

  // setters for named parameter idiom
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
    me416_msgs::msg::MotorSpeeds_<ContainerAllocator> *;
  using ConstRawPtr =
    const me416_msgs::msg::MotorSpeeds_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      me416_msgs::msg::MotorSpeeds_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      me416_msgs::msg::MotorSpeeds_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__me416_msgs__msg__MotorSpeeds
    std::shared_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__me416_msgs__msg__MotorSpeeds
    std::shared_ptr<me416_msgs::msg::MotorSpeeds_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSpeeds_ & other) const
  {
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSpeeds_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSpeeds_

// alias to use template instance with default allocator
using MotorSpeeds =
  me416_msgs::msg::MotorSpeeds_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace me416_msgs

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__STRUCT_HPP_
