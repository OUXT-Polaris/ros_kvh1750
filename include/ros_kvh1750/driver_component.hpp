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

#include <ros_kvh1750/kvh_plugin.h>
#include <ros_kvh1750/tov_file.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace ros_kvh1750
{
class DriverComponent : public rclcpp::Node
{
public:
  ROS_KVH_1750_DRIVER_COMPONENT_PUBLIC
  DriverComponent(const rclcpp::NodeOptions & options);

private:
  void toRos(
    const kvh::Message & msg, sensor_msgs::msg::Imu & imu, sensor_msgs::msg::Temperature & temp);
  bool is_da_;
  double ahrs_gyro_x_ = 0;
  double ahrs_gyro_y_ = 0;
  double ahrs_gyro_z_ = 0;
  double prev_stamp_ = 0;
  int rate_;
  std::string imu_frame_;
  bool use_delta_angles_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  sensor_msgs::msg::Imu current_imu_;
  sensor_msgs::msg::Temperature current_temp_;
  std::vector<double> ahrs_cov_;
  std::vector<double> ang_cov_;
  std::vector<double> lin_cov_;
  std::shared_ptr<kvh::IOModule> io_module_;
  std::shared_ptr<kvh::IMU1750> imu_;
  // std::shared_ptr<kvh::MessageProcessorBase> plugin_;
};
}  // namespace ros_kvh1750

#endif  // ROS_KVH_1750__DRIVER_COMPONENT_HPP_