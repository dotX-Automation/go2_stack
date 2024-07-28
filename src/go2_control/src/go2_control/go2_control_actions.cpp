/**
 * Go2 Control node actions implementation.
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

#define UNUSED(arg) (void)(arg)

#include <go2_control/go2_control.hpp>

namespace go2_control
{

/**
 * @brief Handles a new Arm goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse Go2Control::handle_arm_goal(
  const rclcpp_action::GoalUUID & uuid,
  ArmGoalSharedPtr goal)
{
  UNUSED(uuid);
  UNUSED(goal);
  RCLCPP_INFO(this->get_logger(), "Received arming request");
  if (kill_switch_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Arming request rejected: kill switch active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Disarm goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse Go2Control::handle_disarm_goal(
  const rclcpp_action::GoalUUID & uuid,
  DisarmGoalSharedPtr goal)
{
  UNUSED(uuid);
  UNUSED(goal);
  if (kill_switch_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Disarming request rejected: kill switch active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Received disarming request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Arm cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse Go2Control::handle_arm_cancel(
  const ArmGoalHandleSharedPtr goal_handle)
{
  // Arming cannot be canceled while in progress
  UNUSED(goal_handle);
  RCLCPP_ERROR(
    this->get_logger(),
    "Arming cancellation request rejected");
  return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief Handles a new Disarm cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse Go2Control::handle_disarm_cancel(
  const DisarmGoalHandleSharedPtr goal_handle)
{
  // Disarming cannot be canceled while in progress
  UNUSED(goal_handle);
  RCLCPP_ERROR(
    this->get_logger(),
    "Disarming cancellation request rejected");
  return rclcpp_action::CancelResponse::REJECT;
}

/**
 * @brief Starts execution of an Arm.
 *
 * @param goal_handle Handle to the goal object.
 */
void Go2Control::handle_arm_accepted(const ArmGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &Go2Control::arm,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Disarm.
 *
 * @param goal_handle Handle to the goal object.
 */
void Go2Control::handle_disarm_accepted(const DisarmGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &Go2Control::disarm,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

} // go2_control
