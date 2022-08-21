// Copyright 2022 OUXT Polaris.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ROS_KVH_1750__DRIVER_COMPONENT_HPP_
#define ROS_KVH_1750__DRIVER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROS_KVH_1750_DRIVER_COMPONENT_EXPORT __attribute__((dllexport))
#define ROS_KVH_1750_DRIVER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define ROS_KVH_1750_DRIVER_COMPONENT_EXPORT __declspec(dllexport)
#define ROS_KVH_1750_DRIVER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef ROS_KVH_1750_DRIVER_COMPONENT_BUILDING_DLL
#define ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC ROS_KVH_1750_DRIVER_COMPONENT_EXPORT
#else
#define ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC ROS_KVH_1750_DRIVER_COMPONENT_IMPORT
#endif
#define ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC_TYPE ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC
#define ROS_KVH_1750_DRIVER_COMPONENT_LOCAL
#else
#define ROS_KVH_1750_DRIVER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define ROS_KVH_1750_DRIVER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define ROS_KVH_1750_DRIVER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC
#define ROS_KVH_1750_DRIVER_COMPONENT_LOCAL
#endif
#define ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#endif  // ROS_KVH_1750__DRIVER_COMPONENT_HPP_