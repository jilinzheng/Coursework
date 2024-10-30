// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from me416_msgs:msg/MotorSpeedsStamped.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__STRUCT_H_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/MotorSpeedsStamped in the package me416_msgs.
typedef struct me416_msgs__msg__MotorSpeedsStamped
{
  std_msgs__msg__Header header;
  double left;
  double right;
} me416_msgs__msg__MotorSpeedsStamped;

// Struct for a sequence of me416_msgs__msg__MotorSpeedsStamped.
typedef struct me416_msgs__msg__MotorSpeedsStamped__Sequence
{
  me416_msgs__msg__MotorSpeedsStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} me416_msgs__msg__MotorSpeedsStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS_STAMPED__STRUCT_H_
