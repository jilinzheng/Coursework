// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from me416_msgs:msg/MotorSpeeds.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__BUILDER_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "me416_msgs/msg/detail/motor_speeds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace me416_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorSpeeds_right
{
public:
  explicit Init_MotorSpeeds_right(::me416_msgs::msg::MotorSpeeds & msg)
  : msg_(msg)
  {}
  ::me416_msgs::msg::MotorSpeeds right(::me416_msgs::msg::MotorSpeeds::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::me416_msgs::msg::MotorSpeeds msg_;
};

class Init_MotorSpeeds_left
{
public:
  Init_MotorSpeeds_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSpeeds_right left(::me416_msgs::msg::MotorSpeeds::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_MotorSpeeds_right(msg_);
  }

private:
  ::me416_msgs::msg::MotorSpeeds msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::me416_msgs::msg::MotorSpeeds>()
{
  return me416_msgs::msg::builder::Init_MotorSpeeds_left();
}

}  // namespace me416_msgs

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__BUILDER_HPP_
