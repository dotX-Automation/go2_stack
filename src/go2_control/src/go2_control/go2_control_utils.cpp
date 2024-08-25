/**
 * Go2 Control node auxiliary functions.
 *
 * July 28, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <go2_control/go2_control.hpp>

namespace go2_control
{

/**
 * @brief Notifies a Sport mode state update.
 */
void Go2Control::notify_sportmode_state()
{
  {
    std::lock_guard sportmode_state_lock(sportmode_state_lock_);
    bool expected = false;
    if (!sportmode_state_received_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      return;
    }
  }
  sportmode_state_cv_.notify_one();
}

/**
 * @brief Sets the onboard obstacle avoidance state.
 *
 * @param state New state to set.
 */
void Go2Control::set_obstacle_avoidance(bool state)
{
  Request obs_avoidance_req{};
  obs_avoidance_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
  obs_avoidance_req.header.identity.set__api_id(
    unitree::robot::go2::ROBOT_API_ID_OBSTACLES_AVOID_SWITCH_SET);

  nlohmann::json obs_avoidance_j = {{"enable", state}};
  obs_avoidance_req.set__parameter(obs_avoidance_j.dump());

  obstacle_avoidance_request_pub_->publish(obs_avoidance_req);

  RCLCPP_INFO(this->get_logger(), "Obstacle avoidance %s", state ? "activated" : "deactivated");
}

/**
 * @brief Converts a point cloud message into a Eigen matrix.
 *
 * @param msg Point cloud message to convert.
 * @param read_intensities True if the intensities should be read, false otherwise.
 * @param intensities Vector to store the intensities.
 * @return Eigen matrix with the point cloud data.
 */
Eigen::MatrixXd Go2Control::cloud_to_matrix(
  const PointCloud2::SharedPtr msg,
  bool read_intensities,
  const std::shared_ptr<std::vector<float>> intensities)
{
  Eigen::MatrixXd cloud(4, msg->width);

  sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");

  std::shared_ptr<sensor_msgs::PointCloud2Iterator<float>> i_it;
  if (read_intensities) {
    i_it = std::make_shared<sensor_msgs::PointCloud2Iterator<float>>(*msg, "intensity");
  }

  for (size_t i = 0; i < msg->width; i++, ++x_it, ++y_it, ++z_it) {
    cloud(0, i) = double(*x_it);
    cloud(1, i) = double(*y_it);
    cloud(2, i) = double(*z_it);
    cloud(3, i) = 1.0;

    if (read_intensities) {
      intensities->push_back(*(*i_it));
      ++(*i_it);
    }
  }

  return cloud;
}

/**
 * @brief Converts an Eigen matrix into a point cloud message.
 *
 * @param mat Eigen matrix to convert.
 * @param write_intensities True if the intensities should be written, false otherwise.
 * @param intensities Vector with the intensities.
 * @return Point cloud message with the matrix data.
 */
PointCloud2::SharedPtr Go2Control::matrix_to_cloud(
  const Eigen::MatrixXd & mat,
  bool write_intensities,
  const std::shared_ptr<std::vector<float>> intensities)
{
  PointCloud2::SharedPtr msg = std::make_shared<PointCloud2>();

  msg->set__height(1);
  msg->set__width(mat.cols());
  msg->set__is_bigendian(false);
  msg->set__is_dense(true);

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  if (write_intensities) {
    modifier.setPointCloud2Fields(
      4,
      "x", 1, PointField::FLOAT32,
      "y", 1, PointField::FLOAT32,
      "z", 1, PointField::FLOAT32,
      "intensity", 1, PointField::FLOAT32);
  } else {
    modifier.setPointCloud2Fields(
      3,
      "x", 1, PointField::FLOAT32,
      "y", 1, PointField::FLOAT32,
      "z", 1, PointField::FLOAT32);
  }

  sensor_msgs::PointCloud2Iterator<float> x_it(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(*msg, "z");

  std::shared_ptr<sensor_msgs::PointCloud2Iterator<float>> i_it;
  if (write_intensities) {
    i_it = std::make_shared<sensor_msgs::PointCloud2Iterator<float>>(*msg, "intensity");
  }

  for (ssize_t i = 0; i < mat.cols(); i++, ++x_it, ++y_it, ++z_it) {
    *x_it = static_cast<float>(mat(0, i));
    *y_it = static_cast<float>(mat(1, i));
    *z_it = static_cast<float>(mat(2, i));

    if (write_intensities) {
      *(*i_it) = intensities->at(i);
      ++(*i_it);
    }
  }

  return msg;
}

/**
 * @brief Retrieves a transform from the TF tree.
 *
 * @param target_frame Target frame of the transform.
 * @param source_frame Source frame of the transform.
 * @param time Time of the transform.
 * @param tf_timeout_sec Timeout for the transform request [seconds].
 * @return TransformStamped with the requested transform.
 */
TransformStamped Go2Control::get_tf(
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & time,
  double tf_timeout_sec)
{
  TransformStamped tf{};
  rclcpp::Time tf_time = time;

  while (true) {
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame,
        source_frame,
        tf_time,
        tf2::durationFromSec(tf_timeout_sec));
      break;
    } catch (const tf2::ExtrapolationException & e) {
      // Just get the latest
      tf_time = rclcpp::Time{};
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Go2Control::get_tf: TF exception: %s",
        e.what());
    }
  }

  return tf;
}

/**
 * @brief Parses and republishes a battery state message.
 *
 * @param msg Battery state message to parse.
 */
void Go2Control::parse_battery_state(const BmsState & msg)
{
  BatteryState battery_state_msg{};
  battery_state_msg.header.set__frame_id(body_frame_);
  battery_state_msg.header.set__stamp(this->get_clock()->now());

  float nan = std::numeric_limits<float>::quiet_NaN();

  battery_state_msg.set__voltage(static_cast<float>(msg.cell_vol[0]));
  battery_state_msg.set__temperature(nan);
  battery_state_msg.set__current(static_cast<float>(msg.current));
  battery_state_msg.set__charge(nan);
  battery_state_msg.set__capacity(15.0f);
  battery_state_msg.set__design_capacity(15.0f);
  if (msg.cell_vol[0] > 0) {
    battery_state_msg.set__percentage(static_cast<float>(msg.cell_vol[0]) / 29.6f);
  } else {
    battery_state_msg.set__percentage(nan);
  }
  battery_state_msg.set__power_supply_status(BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
  battery_state_msg.set__power_supply_health(BatteryState::POWER_SUPPLY_HEALTH_GOOD);
  battery_state_msg.set__power_supply_technology(BatteryState::POWER_SUPPLY_TECHNOLOGY_LION);
  battery_state_msg.set__present(true);

  for (int i = 0; i < 16; i++) {
    battery_state_msg.cell_voltage.push_back(static_cast<float>(msg.cell_vol[i]));
    battery_state_msg.cell_temperature.push_back(nan);
  }

  battery_state_pub_->publish(battery_state_msg);
}

/**
 * @brief Validates the pose_covariance parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool Go2Control::validate_pose_covariance(const rclcpp::Parameter & p)
{
  if (p.as_double_array().size() != 6) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid pose_covariance parameter size: %ld",
      p.as_double_array().size());
    return false;
  }

  pose_covariance_.resize(6);
  for (size_t i = 0; i < 6; i++) {
    pose_covariance_[i] = p.as_double_array()[i];
  }
  return true;
}

/**
 * @brief Validates the imu_covariance parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool Go2Control::validate_imu_covariance(const rclcpp::Parameter & p)
{
  if (p.as_double_array().size() != 9) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid imu_covariance parameter size: %ld",
      p.as_double_array().size());
    return false;
  }

  imu_covariance_.resize(9);
  for (size_t i = 0; i < 9; i++) {
    imu_covariance_[i] = p.as_double_array()[i];
  }
  return true;
}

/**
 * @brief Validates the twist_covariance parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool Go2Control::validate_twist_covariance(const rclcpp::Parameter & p)
{
  if (p.as_double_array().size() != 6) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid twist_covariance parameter size: %ld",
      p.as_double_array().size());
    return false;
  }

  twist_covariance_.resize(6);
  for (size_t i = 0; i < 6; i++) {
    twist_covariance_[i] = p.as_double_array()[i];
  }
  return true;
}

} // namespace go2_control
