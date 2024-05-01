// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from me416_msgs:msg/MotorSpeedsStamped.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__BUILDER_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "me416_msgs/msg/detail/motor_speeds_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace me416_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorSpeedsStamped_right
{
public:
  explicit Init_MotorSpeedsStamped_right(::me416_msgs::msg::MotorSpeedsStamped & msg)
  : msg_(msg)
  {}
  ::me416_msgs::msg::MotorSpeedsStamped right(::me416_msgs::msg::MotorSpeedsStamped::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::me416_msgs::msg::MotorSpeedsStamped msg_;
};

class Init_MotorSpeedsStamped_left
{
public:
  explicit Init_MotorSpeedsStamped_left(::me416_msgs::msg::MotorSpeedsStamped & msg)
  : msg_(msg)
  {}
  Init_MotorSpeedsStamped_right left(::me416_msgs::msg::MotorSpeedsStamped::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_MotorSpeedsStamped_right(msg_);
  }

private:
  ::me416_msgs::msg::MotorSpeedsStamped msg_;
};

class Init_MotorSpeedsStamped_header
{
public:
  Init_MotorSpeedsStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSpeedsStamped_left header(::me416_msgs::msg::MotorSpeedsStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorSpeedsStamped_left(msg_);
  }

private:
  ::me416_msgs::msg::MotorSpeedsStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::me416_msgs::msg::MotorSpeedsStamped>()
{
  return me416_msgs::msg::builder::Init_MotorSpeedsStamped_header();
}

}  // namespace me416_msgs

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__BUILDER_HPP_
