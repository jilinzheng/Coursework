// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from me416_msgs:msg/MotorSpeedsStamped.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "me416_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "me416_msgs/msg/detail/motor_speeds_stamped__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace me416_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_me416_msgs
cdr_serialize(
  const me416_msgs::msg::MotorSpeedsStamped & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_me416_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  me416_msgs::msg::MotorSpeedsStamped & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_me416_msgs
get_serialized_size(
  const me416_msgs::msg::MotorSpeedsStamped & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_me416_msgs
max_serialized_size_MotorSpeedsStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace me416_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_me416_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, me416_msgs, msg, MotorSpeedsStamped)();

#ifdef __cplusplus
}
#endif

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
