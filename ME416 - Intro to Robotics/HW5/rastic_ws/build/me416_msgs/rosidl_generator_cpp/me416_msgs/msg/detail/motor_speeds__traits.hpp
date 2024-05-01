// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from me416_msgs:msg/MotorSpeeds.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__TRAITS_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "me416_msgs/msg/detail/motor_speeds__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace me416_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSpeeds & msg,
  std::ostream & out)
{
  out << "{";
  // member: left
  {
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorSpeeds & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << "\n";
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSpeeds & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace me416_msgs

namespace rosidl_generator_traits
{

[[deprecated("use me416_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const me416_msgs::msg::MotorSpeeds & msg,
  std::ostream & out, size_t indentation = 0)
{
  me416_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use me416_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const me416_msgs::msg::MotorSpeeds & msg)
{
  return me416_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<me416_msgs::msg::MotorSpeeds>()
{
  return "me416_msgs::msg::MotorSpeeds";
}

template<>
inline const char * name<me416_msgs::msg::MotorSpeeds>()
{
  return "me416_msgs/msg/MotorSpeeds";
}

template<>
struct has_fixed_size<me416_msgs::msg::MotorSpeeds>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<me416_msgs::msg::MotorSpeeds>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<me416_msgs::msg::MotorSpeeds>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__TRAITS_HPP_
