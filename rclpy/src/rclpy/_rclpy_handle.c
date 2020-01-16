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

#include <Python.h>
#include <stddef.h>

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"
#include "rcutils/types/rcutils_ret.h"

#include "./rclpy_handle.h"

/// Creates a Handle object.
rclpy_handle_t *
_rclpy_create_handle(void * ptr, rclpy_handle_destructor_t destructor)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rclpy_handle_t * handle = allocator.zero_allocate(1, sizeof(rclpy_handle_t), allocator.state);
  if (!handle) {
    goto fail;
  }

  handle->ptr = ptr;
  handle->ref_count++;
  handle->destructor = destructor;

  return handle;
fail:
  allocator.deallocate(handle, allocator.state);
  return NULL;
}

/// Adds a dependency to a handle.
/**
 * The `dependency` handle reference count is incresead.
 * The `dependent` handle stores `dependency` into its dependencies list.
 */
rcutils_ret_t
_rclpy_handle_add_dependency(rclpy_handle_t * dependent, rclpy_handle_t * dependency)
{
  assert(dependent);
  assert(dependency);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  dependent->num_of_dependencies++;
  rclpy_handle_t ** new_dependencies = allocator.reallocate(
    dependent->dependencies,
    dependent->num_of_dependencies * sizeof(rclpy_handle_t *),
    allocator.state);
  if (!new_dependencies) {
    return RCUTILS_RET_ERROR;
  }
  new_dependencies[dependent->num_of_dependencies - 1] = dependency;
  dependent->dependencies = new_dependencies;
  dependency->ref_count++;
  return RCUTILS_RET_OK;
}

/// Decrements the reference count of a handle.
/**
 * The reference count of `handle` is decremented.
 * If it reaches zero:
 * - `rclpy_handle_dec_ref` is called on `handle` dependencies.
 * - `handle` is deallocated.
 */
void
_rclpy_handle_dec_ref(rclpy_handle_t * handle)
{
  if (!handle) {
    return;
  }
  assert(
    (0u != handle->num_of_dependencies && NULL != handle->dependencies) ||
    (0u == handle->num_of_dependencies && NULL == handle->dependencies));

  handle->ref_count--;
  if (handle->ref_count) {
    for (size_t i = 0; i < handle->num_of_dependencies; i++) {
      _rclpy_handle_dec_ref(handle->dependencies[i]);
    }
    if (handle->destructor) {
      handle->destructor(handle->ptr);
    }
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    allocator.deallocate(handle->dependencies, allocator.state);
    allocator.deallocate(handle, allocator.state);
  }
}

void
_rclpy_handle_capsule_destructor(PyObject * capsule)
{
  rclpy_handle_t * handle = PyCapsule_GetPointer(capsule, PyCapsule_GetName(capsule));
  if (!handle) {
    return;
  }
  _rclpy_handle_dec_ref(handle);
}

/// Creates a PyCapsule wrapping a handle object.
PyObject *
rclpy_create_handle_capsule(void * ptr, const char * name, rclpy_handle_destructor_t destructor)
{
  rclpy_handle_t * handle = _rclpy_create_handle(ptr, destructor);
  if (!handle) {
    return NULL;
  }

  return PyCapsule_New(handle, name, _rclpy_handle_capsule_destructor);
}

void *
rclpy_handle_get_pointer(PyObject * capsule, const char * name)
{
  rclpy_handle_t * handle = PyCapsule_GetPointer(capsule, name);
  if (!handle) {
    return NULL;
  }
  return handle->ptr;
}

static PyObject *
rclpy_handle_add_dependency(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * dependent_capsule;
  PyObject * dependency_capsule;
  if (!PyArg_ParseTuple(args, "OO", &dependent_capsule, &dependency_capsule)) {
    return NULL;
  }

  rclpy_handle_t * dependent = PyCapsule_GetPointer(dependent_capsule, PyCapsule_GetName(dependent_capsule));
  rclpy_handle_t * dependency = PyCapsule_GetPointer(dependency_capsule, PyCapsule_GetName(dependency_capsule));


  if (PyErr_Occurred()) {
    return NULL;
  }

  if (RCUTILS_RET_OK != _rclpy_handle_add_dependency(dependent, dependency)) {
    PyErr_Format(PyExc_RuntimeError, "Failed to add dependency to handle");
    return NULL;
  }

  return Py_None;
}

static PyObject *
rclpy_handle_get_name(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * handle_capsule;
  if (!PyArg_ParseTuple(args, "O", &handle_capsule)) {
    return NULL;
  }

  if (PyErr_Occurred()) {
    return NULL;
  }
  return PyUnicode_FromString(PyCapsule_GetName(handle_capsule));
}

static PyObject *
rclpy_handle_dec_ref(PyObject * Py_UNUSED(self), PyObject * args)
{
  PyObject * handle_capsule;
  if (!PyArg_ParseTuple(args, "O", &handle_capsule)) {
    return NULL;
  }

  rclpy_handle_t * handle = PyCapsule_GetPointer(
    handle_capsule, PyCapsule_GetName(handle_capsule));

  if (PyErr_Occurred()) {
    return NULL;
  }

  _rclpy_handle_dec_ref(handle);
  return Py_None;
}

/// Define the public methods of this module
static PyMethodDef rclpy_handle_methods[] = {
  {
    "rclpy_handle_add_dependency", rclpy_handle_add_dependency,
    METH_VARARGS,
    "Add a dependency to a handle."
  },
  {
    "rclpy_handle_dec_ref", rclpy_handle_dec_ref,
    METH_VARARGS,
    "Decrement reference count of a handle."
  },
  {
    "rclpy_handle_get_name", rclpy_handle_get_name,
    METH_VARARGS,
    "Get the handle name."
  },
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(rclpy_handle__doc__,
  "rclpy module for working with Handle objects.");

/// Define the Python module
static struct PyModuleDef _rclpy_handle_module = {
  PyModuleDef_HEAD_INIT,
  "_rclpy_handle",
  rclpy_handle__doc__,
  -1,  /* -1 means that the module keeps state in global variables */
  rclpy_handle_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

/// Init function of this module
PyMODINIT_FUNC PyInit__rclpy_handle(void)
{
  return PyModule_Create(&_rclpy_handle_module);
}
