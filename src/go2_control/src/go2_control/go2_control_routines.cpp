/**
 * Go2 Control node actions routines.
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
 * @brief Arms the robot.
 *
 * @param goal_handle Arm action goal handle.
 */
void Go2Control::arm(const ArmGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Arm::Result>();
  int64_t stabilization_time = operations_stabilization_time_;
  int64_t timeout = operations_timeout_;

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(frame_prefix_ + "base_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  // Check if the robot is already armed
  if (armed_.load(std::memory_order_acquire)) {
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(frame_prefix_ + "base_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    goal_handle->succeed(result);
    operation_lock_.unlock();
    return;
  }

  // Build API request
  Request recoverystand_req{};
  recoverystand_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
  recoverystand_req.header.identity.set__api_id(
    unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND);

  // Send the request and wait for the robot to stabilize
  {
    std::unique_lock sportmode_state_lock(sportmode_state_lock_);
    sportmode_state_received_.store(false, std::memory_order_release);
    sport_request_pub_->publish(recoverystand_req);
    if (!sportmode_state_cv_.wait_for(
        sportmode_state_lock,
        std::chrono::milliseconds(timeout),
        [this]() -> bool {return sportmode_state_received_.load();}))
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for robot operation");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(frame_prefix_ + "base_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Timeout waiting for robot operation");
      goal_handle->abort(result);
      operation_lock_.unlock();
      return;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(stabilization_time));

  operation_lock_.unlock();
  result->result.header.set__stamp(this->get_clock()->now());
  result->result.header.set__frame_id(frame_prefix_ + "base_link");
  result->result.set__result(CommandResultStamped::SUCCESS);
  goal_handle->succeed(result);
}

/**
 * @brief Disarms the robot.
 *
 * @param goal_handle Disarm action goal handle.
 */
void Go2Control::disarm(const DisarmGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Disarm::Result>();
  int64_t stabilization_time = operations_stabilization_time_;
  int64_t timeout = operations_timeout_;

  // Check if some other operation is in progress
  if (!operation_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(frame_prefix_ + "base_link");
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  // Check if the robot is already disarmed
  if (!armed_.load(std::memory_order_acquire)) {
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(frame_prefix_ + "base_link");
    result->result.set__result(CommandResultStamped::SUCCESS);
    goal_handle->succeed(result);
    operation_lock_.unlock();
    return;
  }

  // Build API request for STANDDOWN
  Request standdown_req{};
  standdown_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
  standdown_req.header.identity.set__api_id(unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN);

  // Send the request and wait for the robot to stabilize
  {
    std::unique_lock sportmode_state_lock(sportmode_state_lock_);
    sportmode_state_received_.store(false, std::memory_order_release);
    sport_request_pub_->publish(standdown_req);
    if (!sportmode_state_cv_.wait_for(
        sportmode_state_lock,
        std::chrono::milliseconds(timeout),
        [this]() -> bool {return sportmode_state_received_.load();}))
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for robot operation");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(frame_prefix_ + "base_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Timeout waiting for robot operation");
      goal_handle->abort(result);
      operation_lock_.unlock();
      return;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(stabilization_time));

  // Build API request for DAMP
  Request damp_req{};
  damp_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
  damp_req.header.identity.set__api_id(unitree::robot::go2::ROBOT_SPORT_API_ID_DAMP);

  // Send the request and wait for the robot to stabilize
  {
    std::unique_lock sportmode_state_lock(sportmode_state_lock_);
    sportmode_state_received_.store(false, std::memory_order_release);
    sport_request_pub_->publish(damp_req);
    if (!sportmode_state_cv_.wait_for(
        sportmode_state_lock,
        std::chrono::milliseconds(timeout),
        [this]() -> bool {return sportmode_state_received_.load();}))
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for robot operation");
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(frame_prefix_ + "base_link");
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Timeout waiting for robot operation");
      goal_handle->abort(result);
      operation_lock_.unlock();
      return;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(stabilization_time));

  operation_lock_.unlock();
  result->result.header.set__stamp(this->get_clock()->now());
  result->result.header.set__frame_id(frame_prefix_ + "base_link");
  result->result.set__result(CommandResultStamped::SUCCESS);
  goal_handle->succeed(result);
}

} // namespace go2_control
