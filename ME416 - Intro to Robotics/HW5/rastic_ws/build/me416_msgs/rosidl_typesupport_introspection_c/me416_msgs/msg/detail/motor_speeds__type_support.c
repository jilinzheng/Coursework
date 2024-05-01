// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from me416_msgs:msg/MotorSpeeds.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "me416_msgs/msg/detail/motor_speeds__rosidl_typesupport_introspection_c.h"
#include "me416_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "me416_msgs/msg/detail/motor_speeds__functions.h"
#include "me416_msgs/msg/detail/motor_speeds__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  me416_msgs__msg__MotorSpeeds__init(message_memory);
}

void me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_fini_function(void * message_memory)
{
  me416_msgs__msg__MotorSpeeds__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_member_array[2] = {
  {
    "left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(me416_msgs__msg__MotorSpeeds, left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(me416_msgs__msg__MotorSpeeds, right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_members = {
  "me416_msgs__msg",  // message namespace
  "MotorSpeeds",  // message name
  2,  // number of fields
  sizeof(me416_msgs__msg__MotorSpeeds),
  me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_member_array,  // message members
  me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_init_function,  // function to initialize message memory (memory has to be allocated)
  me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_type_support_handle = {
  0,
  &me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_me416_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, me416_msgs, msg, MotorSpeeds)() {
  if (!me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_type_support_handle.typesupport_identifier) {
    me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &me416_msgs__msg__MotorSpeeds__rosidl_typesupport_introspection_c__MotorSpeeds_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
