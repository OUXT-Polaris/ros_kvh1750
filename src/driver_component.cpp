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
  std::string imu_topic;
  declare_parameter<std::string>("imu_topic", "imu");
  get_parameter<std::string>("imu_topic", imu_topic);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
  std::string temperature_topic;
  declare_parameter<std::string>("temperature_topic", "temp");
  get_parameter<std::string>("temperature_topic", temperature_topic);
  temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(temperature_topic, 1);
  declare_parameter<std::string>("imu_frame", "imu");
  get_parameter<std::string>("imu_frame", imu_frame_);
  declare_parameter<int>("rate", 100);
  get_parameter<int>("rate", rate_);
  declare_parameter<bool>("use_delta_angles", true);
  get_parameter<bool>("use_delta_angles", use_delta_angles_);
  declare_parameter<std::vector<double>>("orientation_covariance", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  get_parameter<std::vector<double>>("orientation_covariance", ahrs_cov_);
  std::copy(ahrs_cov_.begin(), ahrs_cov_.end(), current_imu_.orientation_covariance.begin());
  declare_parameter<std::vector<double>>("linear_covariance", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  get_parameter<std::vector<double>>("linear_covariance", lin_cov_);
  std::copy(lin_cov_.begin(), lin_cov_.end(), current_imu_.linear_acceleration_covariance.begin());
  // plugin_ = std::make_shared<kvh::MessageProcessorBase>();
  // plugin_->set_link_name(imu_frame_);
  current_temp_.header.frame_id = imu_frame_;
  current_imu_.header.frame_id = imu_frame_;
  std::string address, tov_address;
  declare_parameter<std::string>("address", "/dev/ttyS4");
  get_parameter<std::string>("address", address);
  declare_parameter<std::string>("tov_address", "");
  get_parameter<std::string>("tov_address", tov_address);
  declare_parameter<int>("baudrate", 921600);
  uint32_t baud = static_cast<uint32_t>(get_parameter("baudrate").as_int());
  int max_temp;
  declare_parameter<int>("max_temp", kvh::MaxTemp_C);
  get_parameter<int>("max_temp", max_temp);

  io_module_ = std::shared_ptr<kvh::IOModule>(new kvh::TOVFile(address, baud, 100, tov_address));
  imu_ = std::make_shared<kvh::IMU1750>(io_module_);
  imu_->set_temp_limit(max_temp);
  if (!imu_->set_angle_units(use_delta_angles_)) {
    RCLCPP_ERROR(get_logger(), "Could not set angle units.");
  }
  if (rate_ > 0) {
    if (!imu_->set_data_rate(rate_)) {
      RCLCPP_ERROR(get_logger(), "Could not set data rate to %d", rate_);
    }
  }
  imu_->query_data_rate(rate_);
  imu_->query_angle_units(is_da_);
}

void DriverComponent::read()
{
  while (rclcpp::ok()) {
    kvh::Message msg;
    switch (imu_->read(msg)) {
      case kvh::IMU1750::VALID:
        toRos(msg, current_imu_, current_temp_);
        imu_pub_->publish(current_imu_);
        temp_pub_->publish(current_temp_);
        break;
      case kvh::IMU1750::BAD_READ:
      case kvh::IMU1750::BAD_CRC:
        RCLCPP_ERROR(get_logger(), "Bad data from KVH, ignoring.");
        break;
      case kvh::IMU1750::FATAL_ERROR:
        RCLCPP_FATAL(get_logger(), "Lost connection to IMU!");
        break;
      case kvh::IMU1750::OVER_TEMP:
        RCLCPP_FATAL(get_logger(), "IMU is overheating!");
        break;
      case kvh::IMU1750::PARTIAL_READ:
      default:
        break;
    }
  }
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
  if (is_da_) {
    ahrs_gyro_x_ += msg.gyro_x();
    ahrs_gyro_y_ += msg.gyro_y();
    ahrs_gyro_z_ += msg.gyro_z();

    imu.angular_velocity.x *= rate_;
    imu.angular_velocity.y *= rate_;
    imu.angular_velocity.z *= rate_;
  } else {
    double current_stamp = imu.header.stamp.sec + imu.header.stamp.nanosec * 1E-9;
    double deltatime;
    if (prev_stamp_) {
      deltatime = current_stamp - prev_stamp_;
    } else {
      deltatime = 1 / rate_;
    }
    ahrs_gyro_x_ += msg.gyro_x() * deltatime;
    ahrs_gyro_y_ += msg.gyro_y() * deltatime;
    ahrs_gyro_z_ += msg.gyro_z() * deltatime;
    prev_stamp_ = current_stamp;
  }

  geometry_msgs::msg::Vector3 vec;
  vec.x = ahrs_gyro_x_;
  vec.y = ahrs_gyro_y_;
  vec.z = ahrs_gyro_z_;
  imu.orientation = quaternion_operation::convertEulerAngleToQuaternion(vec);
  temp.header.stamp = imu.header.stamp;
  temp.temperature = msg.temp();
}
}  // namespace ros_kvh1750

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros_kvh1750::DriverComponent)
