// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from me416_msgs:msg/MotorSpeeds.idl
// generated code does not contain a copyright notice

#ifndef ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__STRUCT_H_
#define ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorSpeeds in the package me416_msgs.
typedef struct me416_msgs__msg__MotorSpeeds
{
  double left;
  double right;
} me416_msgs__msg__MotorSpeeds;

// Struct for a sequence of me416_msgs__msg__MotorSpeeds.
typedef struct me416_msgs__msg__MotorSpeeds__Sequence
{
  me416_msgs__msg__MotorSpeeds * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} me416_msgs__msg__MotorSpeeds__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ME416_MSGS__MSG__DETAIL__MOTOR_SPEEDS__STRUCT_H_
