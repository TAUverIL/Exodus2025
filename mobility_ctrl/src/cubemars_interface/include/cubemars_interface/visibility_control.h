// Copyright 2021 ros2_control Development Team
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef CUBEMARS_INTERFACE__VISIBILITY_CONTROL_H_
#define CUBEMARS_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CUBEMARS_INTERFACE_EXPORT __attribute__((dllexport))
#define CUBEMARS_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define CUBEMARS_INTERFACE_EXPORT __declspec(dllexport)
#define CUBEMARS_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef CUBEMARS_INTERFACE_BUILDING_DLL
#define CUBEMARS_INTERFACE_PUBLIC CUBEMARS_INTERFACE_EXPORT
#else
#define CUBEMARS_INTERFACE_PUBLIC CUBEMARS_INTERFACE_IMPORT
#endif
#define CUBEMARS_INTERFACE_PUBLIC_TYPE CUBEMARS_INTERFACE_PUBLIC
#define CUBEMARS_INTERFACE_LOCAL
#else
#define CUBEMARS_INTERFACE_EXPORT __attribute__((visibility("default")))
#define CUBEMARS_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define CUBEMARS_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define CUBEMARS_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define CUBEMARS_INTERFACE_PUBLIC
#define CUBEMARS_INTERFACE_LOCAL
#endif
#define CUBEMARS_INTERFACE_PUBLIC_TYPE
#endif

#endif  // CUBEMARS_INTERFACE__VISIBILITY_CONTROL_H_
