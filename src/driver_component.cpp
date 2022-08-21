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

#include <quaternion_operation/quaternion_operation.h>

#include <ros_kvh1750/driver_component.hpp>

namespace ros_kvh1750
{
DriverComponent::DriverComponent(const rclcpp::NodeOptions & options)
: Node("ros_kvh1750_driver", options)
{
}

void DriverComponent::toRos(
  const kvh::Message & msg, sensor_msgs::msg::Imu & imu, sensor_msgs::msg::Temperature & temp)
{
  msg.time(imu.header.stamp.sec, imu.header.stamp.nanosec);

  imu.angular_velocity.x = msg.gyro_x();
  imu.angular_velocity.y = msg.gyro_y();
  imu.angular_velocity.z = msg.gyro_z();
  imu.linear_acceleration.x = msg.accel_x();
  imu.linear_acceleration.y = msg.accel_y();
  imu.linear_acceleration.z = msg.accel_z();

  //scale for ROS if delta angles are enabled
  if (IsDA) {
    Ahrs_gyro_x += msg.gyro_x();
    Ahrs_gyro_y += msg.gyro_y();
    Ahrs_gyro_z += msg.gyro_z();

    imu.angular_velocity.x *= Rate;
    imu.angular_velocity.y *= Rate;
    imu.angular_velocity.z *= Rate;
  } else {
    double current_stamp = imu.header.stamp.sec + imu.header.stamp.nanosec * 1E-9;
    double deltatime;
    if (Prev_stamp) {
      deltatime = current_stamp - Prev_stamp;
    } else {
      deltatime = 1 / Rate;
    }
    Ahrs_gyro_x += msg.gyro_x() * deltatime;
    Ahrs_gyro_y += msg.gyro_y() * deltatime;
    Ahrs_gyro_z += msg.gyro_z() * deltatime;
    Prev_stamp = current_stamp;
  }

  geometry_msgs::msg::Vector3 vec;
  vec.x = Ahrs_gyro_x;
  vec.y = Ahrs_gyro_y;
  vec.z = Ahrs_gyro_z;
  imu.orientation = quaternion_operation::convertEulerAngleToQuaternion(vec);
  temp.header.stamp = imu.header.stamp;
  temp.temperature = msg.temp();
}
}  // namespace ros_kvh1750

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros_kvh1750::DriverComponent)
