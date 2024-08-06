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
