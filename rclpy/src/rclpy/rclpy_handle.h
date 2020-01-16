// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLPY__RCLPY_HANDLE_H_
#define RCLPY__RCLPY_HANDLE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <Python.h>
#include <stddef.h>

typedef struct rclpy_handle_t rclpy_handle_t;

typedef void (* rclpy_handle_destructor_t)(void *);

PyObject *
rclpy_create_handle_capsule(void * ptr, const char * name, rclpy_handle_destructor_t destructor);

void *
rclpy_handle_get_pointer(PyObject * capsule, const char * name);

struct rclpy_handle_t
{
  void * ptr;  // opaque pointer to the wrapped object.
  size_t ref_count;  // Reference count.
  struct rclpy_handle_t ** dependencies;  // array of pointers to dependencies.
  size_t num_of_dependencies;  // size of the array.
  rclpy_handle_destructor_t destructor;  // destructor
};

#ifdef __cplusplus
}
#endif

#endif  // RCLPY__RCLPY_HANDLE_H_
