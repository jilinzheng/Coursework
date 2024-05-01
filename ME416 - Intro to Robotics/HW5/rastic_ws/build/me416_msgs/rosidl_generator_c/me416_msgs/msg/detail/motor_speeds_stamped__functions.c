// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from me416_msgs:msg/MotorSpeedsStamped.idl
// generated code does not contain a copyright notice
#include "me416_msgs/msg/detail/motor_speeds_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
me416_msgs__msg__MotorSpeedsStamped__init(me416_msgs__msg__MotorSpeedsStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    me416_msgs__msg__MotorSpeedsStamped__fini(msg);
    return false;
  }
  // left
  msg->left = 0.0l;
  // right
  msg->right = 0.0l;
  return true;
}

void
me416_msgs__msg__MotorSpeedsStamped__fini(me416_msgs__msg__MotorSpeedsStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // left
  // right
}

bool
me416_msgs__msg__MotorSpeedsStamped__are_equal(const me416_msgs__msg__MotorSpeedsStamped * lhs, const me416_msgs__msg__MotorSpeedsStamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // left
  if (lhs->left != rhs->left) {
    return false;
  }
  // right
  if (lhs->right != rhs->right) {
    return false;
  }
  return true;
}

bool
me416_msgs__msg__MotorSpeedsStamped__copy(
  const me416_msgs__msg__MotorSpeedsStamped * input,
  me416_msgs__msg__MotorSpeedsStamped * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // left
  output->left = input->left;
  // right
  output->right = input->right;
  return true;
}

me416_msgs__msg__MotorSpeedsStamped *
me416_msgs__msg__MotorSpeedsStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  me416_msgs__msg__MotorSpeedsStamped * msg = (me416_msgs__msg__MotorSpeedsStamped *)allocator.allocate(sizeof(me416_msgs__msg__MotorSpeedsStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(me416_msgs__msg__MotorSpeedsStamped));
  bool success = me416_msgs__msg__MotorSpeedsStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
me416_msgs__msg__MotorSpeedsStamped__destroy(me416_msgs__msg__MotorSpeedsStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    me416_msgs__msg__MotorSpeedsStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
me416_msgs__msg__MotorSpeedsStamped__Sequence__init(me416_msgs__msg__MotorSpeedsStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  me416_msgs__msg__MotorSpeedsStamped * data = NULL;

  if (size) {
    data = (me416_msgs__msg__MotorSpeedsStamped *)allocator.zero_allocate(size, sizeof(me416_msgs__msg__MotorSpeedsStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = me416_msgs__msg__MotorSpeedsStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        me416_msgs__msg__MotorSpeedsStamped__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
me416_msgs__msg__MotorSpeedsStamped__Sequence__fini(me416_msgs__msg__MotorSpeedsStamped__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      me416_msgs__msg__MotorSpeedsStamped__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

me416_msgs__msg__MotorSpeedsStamped__Sequence *
me416_msgs__msg__MotorSpeedsStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  me416_msgs__msg__MotorSpeedsStamped__Sequence * array = (me416_msgs__msg__MotorSpeedsStamped__Sequence *)allocator.allocate(sizeof(me416_msgs__msg__MotorSpeedsStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = me416_msgs__msg__MotorSpeedsStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
me416_msgs__msg__MotorSpeedsStamped__Sequence__destroy(me416_msgs__msg__MotorSpeedsStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    me416_msgs__msg__MotorSpeedsStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
me416_msgs__msg__MotorSpeedsStamped__Sequence__are_equal(const me416_msgs__msg__MotorSpeedsStamped__Sequence * lhs, const me416_msgs__msg__MotorSpeedsStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!me416_msgs__msg__MotorSpeedsStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
me416_msgs__msg__MotorSpeedsStamped__Sequence__copy(
  const me416_msgs__msg__MotorSpeedsStamped__Sequence * input,
  me416_msgs__msg__MotorSpeedsStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(me416_msgs__msg__MotorSpeedsStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    me416_msgs__msg__MotorSpeedsStamped * data =
      (me416_msgs__msg__MotorSpeedsStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!me416_msgs__msg__MotorSpeedsStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          me416_msgs__msg__MotorSpeedsStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!me416_msgs__msg__MotorSpeedsStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
