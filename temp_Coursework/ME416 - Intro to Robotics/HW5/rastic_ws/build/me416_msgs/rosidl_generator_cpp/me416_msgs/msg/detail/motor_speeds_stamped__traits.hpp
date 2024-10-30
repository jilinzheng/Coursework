// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from me416_msgs:msg/MotorSpeedsStamped.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__TRAITS_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "me416_msgs/msg/detail/motor_speeds_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace me416_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSpeedsStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

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
  const MotorSpeedsStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

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

inline std::string to_yaml(const MotorSpeedsStamped & msg, bool use_flow_style = false)
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
  const me416_msgs::msg::MotorSpeedsStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  me416_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use me416_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const me416_msgs::msg::MotorSpeedsStamped & msg)
{
  return me416_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<me416_msgs::msg::MotorSpeedsStamped>()
{
  return "me416_msgs::msg::MotorSpeedsStamped";
}

template<>
inline const char * name<me416_msgs::msg::MotorSpeedsStamped>()
{
  return "me416_msgs/msg/MotorSpeedsStamped";
}

template<>
struct has_fixed_size<me416_msgs::msg::MotorSpeedsStamped>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<me416_msgs::msg::MotorSpeedsStamped>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<me416_msgs::msg::MotorSpeedsStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__TRAITS_HPP_
