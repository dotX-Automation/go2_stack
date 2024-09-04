/**
 * Go2 Control node topic subscription callbacks.
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
 * @brief Republishes a velocity setpoint.
 *
 * @param msg Twist message to parse.
 */
void Go2Control::cmd_vel_callback(const Twist::SharedPtr msg)
{
  // Check if we can execute
  if (!armed_.load(std::memory_order_acquire)) {
    return;
  }

  // Check kill switch, if engaged publish zero velocity setpoint to reactively stop the robot
  if (kill_switch_.load(std::memory_order_acquire)) {
    if (!stopped_) {
      nlohmann::json stop_j = {
        {"x", 0.0},
        {"y", 0.0},
        {"z", 0.0}};

      Request stop_req{};
      stop_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
      stop_req.header.identity.set__api_id(unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE);
      stop_req.set__parameter(stop_j.dump());
      sport_request_pub_->publish(stop_req);

      stopped_ = true;
    }
    return;
  } else {
    stopped_ = false;
  }

  // Saturate velocities
  double v_x = fmin(fmax(msg->linear.x, -velocity_control_vhorz_max_), velocity_control_vhorz_max_);
  double v_y = fmin(fmax(msg->linear.y, -velocity_control_vhorz_max_), velocity_control_vhorz_max_);
  double v_yaw =
    fmin(fmax(msg->angular.z, -velocity_control_vyaw_max_), velocity_control_vyaw_max_);

  // Build the velocity setpoint
  nlohmann::json cmd_vel_j = {
    {"x", v_x},
    {"y", v_y},
    {"z", v_yaw}};

  // Build and publish the API request
  Request cmd_vel_req{};
  cmd_vel_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
  cmd_vel_req.header.identity.set__api_id(unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE);
  cmd_vel_req.set__parameter(cmd_vel_j.dump());
  sport_request_pub_->publish(cmd_vel_req);
}

/**
 * @brief Republishes low-level robot state data.
 *
 * @param msg Message to parse.
 */
void Go2Control::lowstate_callback(const LowState::SharedPtr msg)
{
  // Joint state
  JointState joint_state_msg{};
  joint_state_msg.header.set__stamp(this->get_clock()->now());
  joint_state_msg.set__name(
    {
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    });
  joint_state_msg.set__position(
    {
      msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
      msg->motor_state[0].q, msg->motor_state[1].q, msg->motor_state[2].q,
      msg->motor_state[9].q, msg->motor_state[10].q, msg->motor_state[11].q,
      msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q,
    });
  joint_states_pub_->publish(joint_state_msg);

  // Battery state
  parse_battery_state(msg);
}

/**
 * @brief Republishes odometry data.
 *
 * @param msg Odometry message to parse.
 */
void Go2Control::odometry_callback(const Odometry::SharedPtr msg)
{
  // Drop this sample if no initial pose is available
  if (!init_pose_ok_.load(std::memory_order_acquire)) {
    return;
  }

  // Initialize odometry message
  Odometry odom_msg{};
  odom_msg.header.set__stamp(msg->header.stamp);
  odom_msg.header.set__frame_id(local_frame_);
  odom_msg.set__child_frame_id(body_frame_);

  // Set pose compensating initial pose
  Eigen::Isometry3d pose_curr_iso{};
  tf2::fromMsg(msg->pose.pose, pose_curr_iso);
  Eigen::Isometry3d pose_curr_iso_comp = init_pose_inv_iso_ * pose_curr_iso;
  odom_msg.pose.set__pose(tf2::toMsg(pose_curr_iso_comp));

  // Set pose covariance
  int j = 0;
  for (int i = 0; i < 6; i++) {
    odom_msg.pose.covariance[i + j * 6] = pose_covariance_[i];
    j++;
  }

  // Set twist
  odom_msg.twist.set__twist(msg->twist.twist);

  // Set twist covariance
  j = 0;
  for (int i = 0; i < 6; i++) {
    odom_msg.twist.covariance[i + j * 6] = twist_covariance_[i];
    j++;
  }

  // Lock altitude if requested
  if (lock_altitude_) {
    if (armed_.load(std::memory_order_acquire)) {
      odom_msg.pose.pose.position.set__z(0.25);
    } else {
      odom_msg.pose.pose.position.set__z(0.0);
    }
    odom_msg.twist.twist.linear.set__z(0.0);
  }

  // Publish message
  odom_pub_->publish(odom_msg);
}

/**
 * @brief Republishes onboard LiDAR data.
 *
 * @param msg Message to parse.
 */
void Go2Control::point_cloud_callback(const PointCloud2::SharedPtr msg)
{
    // Raw point cloud is already in LiDAR frame
    PointCloud2 point_cloud_msg = *msg;
    point_cloud_msg.header.frame_id = frame_prefix_ + "utlidar";
    point_cloud_pub_->publish(point_cloud_msg);
}

/**
 * @brief Republishes pose data.
 *
 * @param msg Pose message to parse.
 */
void Go2Control::pose_callback(const PoseStamped::SharedPtr msg)
{
  // Store robot pose
  PoseStamped pose_msg_robot = *msg;
  pose_msg_robot.header.frame_id = local_frame_;
  pose_kit::Pose pose_curr{};
  state_lock_.lock();
  if (!init_pose_ok_.load(std::memory_order_acquire)) {
    init_pose_ = pose_kit::Pose(pose_msg_robot);
    init_pose_inv_iso_ = init_pose_.get_isometry().inverse();
    init_pose_ok_.store(true, std::memory_order_release);
  }
  pose_kit::Pose pose_curr_robot(pose_msg_robot);
  Eigen::Isometry3d pose_curr_robot_iso = pose_curr_robot.get_isometry();
  Eigen::Isometry3d pose_curr_iso = init_pose_inv_iso_ * pose_curr_robot_iso;
  if (lock_altitude_) {
    // Lock altitude
    if (armed_.load(std::memory_order_acquire)) {
      pose_curr_iso.translation().z() = 0.25;
    } else {
      pose_curr_iso.translation().z() = 0.0;
    }
  }
  pose_ = pose_kit::Pose(
    pose_curr_iso.translation(),
    Eigen::Quaterniond(pose_curr_iso.rotation()),
    pose_msg_robot.header);
  pose_curr = pose_;
  state_lock_.unlock();

  // Republish the pose with covariance
  PoseWithCovarianceStamped pose_msg = pose_curr.to_pose_with_covariance_stamped();
  pose_msg.set__header(pose_msg_robot.header);
  int j = 0;
  for (int i = 0; i < 6; i++) {
    pose_msg.pose.covariance[i + j * 6] = pose_covariance_[i];
    j++;
  }
  pose_pub_->publish(pose_msg);

  // Publish the tf
  if (publish_tf_) {
    TransformStamped tf_msg{};
    tf_msg.set__header(pose_msg.header);
    tf_msg.set__child_frame_id(body_frame_);
    tf_msg.transform.translation.set__x(pose_msg.pose.pose.position.x);
    tf_msg.transform.translation.set__y(pose_msg.pose.pose.position.y);
    tf_msg.transform.translation.set__z(pose_msg.pose.pose.position.z);
    tf_msg.transform.set__rotation(pose_msg.pose.pose.orientation);
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

/**
 * @brief Parses the Sport Service state.
 *
 * @param msg SportModeState message to parse.
 */
void Go2Control::sportmode_state_callback(const SportModeState::SharedPtr msg)
{
  // Parse and process the mode
  if (msg->mode != sportmode_last_) {
    sportmode_last_ = msg->mode;

    if (msg->mode == static_cast<uint8_t>(SportMode::STANDING)) {
      bool expected = false;
      if (armed_.compare_exchange_strong(
          expected,
          true,
          std::memory_order_release,
          std::memory_order_acquire))
      {
        notify_sportmode_state();
        RCLCPP_WARN(this->get_logger(), "Robot ARMED");
      }
    } else if (msg->mode == static_cast<uint8_t>(SportMode::RECOVERYSTAND)) {
      RCLCPP_INFO(this->get_logger(), "RECOVERY STAND");
    } else if (msg->mode == static_cast<uint8_t>(SportMode::STANDUP)) {
      RCLCPP_INFO(this->get_logger(), "STAND UP (joints locked)");
    } else if (msg->mode == static_cast<uint8_t>(SportMode::STANDDOWN)) {
      notify_sportmode_state();
      RCLCPP_INFO(this->get_logger(), "STAND DOWN");
    } else if (msg->mode == static_cast<uint8_t>(SportMode::DAMP)) {
      bool expected = true;
      if (armed_.compare_exchange_strong(
          expected,
          false,
          std::memory_order_release,
          std::memory_order_acquire))
      {
        notify_sportmode_state();
        RCLCPP_WARN(this->get_logger(), "Robot DISARMED");
      }
    }
  }

  // Parse and publish IMU data
  Imu imu_msg{};
  int j;
  imu_msg.header.set__stamp(this->get_clock()->now());
  imu_msg.header.set__frame_id(frame_prefix_ + "imu");

  imu_msg.orientation.set__w(msg->imu_state.quaternion[0]);
  imu_msg.orientation.set__x(msg->imu_state.quaternion[1]);
  imu_msg.orientation.set__y(msg->imu_state.quaternion[2]);
  imu_msg.orientation.set__z(msg->imu_state.quaternion[3]);
  j = 0;
  for (int i = 0; i < 3; i++) {
    imu_msg.orientation_covariance[i + j * 3] = imu_covariance_[i];
    j++;
  }

  imu_msg.angular_velocity.set__x(msg->imu_state.gyroscope[0]);
  imu_msg.angular_velocity.set__y(msg->imu_state.gyroscope[1]);
  imu_msg.angular_velocity.set__z(msg->imu_state.gyroscope[2]);
  j = 0;
  for (int i = 0; i < 3; i++) {
    imu_msg.angular_velocity_covariance[i + j * 3] = imu_covariance_[i + 3];
    j++;
  }

  imu_msg.linear_acceleration.set__x(msg->imu_state.accelerometer[0]);
  imu_msg.linear_acceleration.set__y(msg->imu_state.accelerometer[1]);
  imu_msg.linear_acceleration.set__z(msg->imu_state.accelerometer[2]);
  j = 0;
  for (int i = 0; i < 3; i++) {
    imu_msg.linear_acceleration_covariance[i + j * 3] = imu_covariance_[i + 6];
    j++;
  }

  imu_pub_->publish(imu_msg);
}

/**
 * @brief Processes inputs from the wireless controller.
 *
 * @param msg WirelessController message to parse.
 */
void Go2Control::wireless_controller_callback(const WirelessController::SharedPtr msg)
{
  if (msg->keys == uint16_t(kill_switch_code_) &&
    !kill_switch_.load(std::memory_order_acquire))
  {
    kill_switch_.store(true, std::memory_order_release);
    RCLCPP_WARN(this->get_logger(), "Kill switch activated");
  } else if (msg->keys == uint16_t(kill_switch_release_code_) &&
    kill_switch_.load(std::memory_order_acquire))
  {
    kill_switch_.store(false, std::memory_order_release);
    RCLCPP_WARN(this->get_logger(), "Kill switch released");
  }
}

} // go2_control
