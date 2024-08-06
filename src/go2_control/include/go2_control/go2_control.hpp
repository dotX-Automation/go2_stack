/**
 * Go2 Control node definition.
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

#ifndef GO2_CONTROL__GO2_CONTROL_HPP
#define GO2_CONTROL__GO2_CONTROL_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <unitree/robot/go2/sport/sport_api.hpp>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <pose_kit/pose.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <dua_interfaces/msg/command_result_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include <dua_interfaces/action/arm.hpp>
#include <dua_interfaces/action/disarm.hpp>

using namespace dua_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace unitree_api::msg;
using namespace unitree_go::msg;

using namespace dua_interfaces::action;

using ArmGoalHandle = rclcpp_action::ServerGoalHandle<Arm>;
using ArmGoalSharedPtr = std::shared_ptr<const Arm::Goal>;
using ArmGoalHandleSharedPtr = std::shared_ptr<ArmGoalHandle>;

using DisarmGoalHandle = rclcpp_action::ServerGoalHandle<Disarm>;
using DisarmGoalSharedPtr = std::shared_ptr<const Disarm::Goal>;
using DisarmGoalHandleSharedPtr = std::shared_ptr<DisarmGoalHandle>;

namespace go2_control
{

/**
 * Low-level robot control module.
 */
class Go2Control : public dua_node::NodeBase
{
public:
  Go2Control(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Go2Control();

private:
  /* Sport mode states encoding. */
  enum class SportMode : uint8_t
  {
    STANDING = 1,     // Permanent, standing idle
    MOVE = 3,         // Temporary, during movement
    STANDDOWN = 5,    // Temporary, during standdown
    STANDUP = 6,      // Permanent, joints locked
    DAMP = 7,         // Permanent, joints dampened and on the ground
    RECOVERYSTAND = 8 // Temporary, during recovery standup
  };

  /* Node initialization routines. */
  void init_atomics();
  void init_cgroups();
  void init_parameters();
  void init_subscriptions();
  void init_tf2();
  void init_publishers();
  void init_actions();

  /* TF2 data. */
  std::string body_frame_;
  std::string local_frame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Topic subscriptions callback groups. */
  rclcpp::CallbackGroup::SharedPtr cmd_vel_callback_group_;
  rclcpp::CallbackGroup::SharedPtr foot_position_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lowstate_callback_group_;
  rclcpp::CallbackGroup::SharedPtr point_cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sportmode_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr wireless_controller_callback_group_;

  /* Topic subscriptions. */
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr foot_position_sub_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<SportModeState>::SharedPtr sportmode_state_sub_;
  rclcpp::Subscription<WirelessController>::SharedPtr wireless_controller_sub_;

  /* Topic subscription callbacks. */
  void cmd_vel_callback(const Twist::SharedPtr msg);
  void foot_position_callback(const PointCloud2::SharedPtr msg);
  void lowstate_callback(const LowState::SharedPtr msg);
  void point_cloud_callback(const PointCloud2::SharedPtr msg);
  void pose_callback(const PoseStamped::SharedPtr msg);
  void sportmode_state_callback(const SportModeState::SharedPtr msg);
  void wireless_controller_callback(const WirelessController::SharedPtr msg);

  /* Topic publishers. */
  rclcpp::Publisher<PointCloud2>::SharedPtr foot_position_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Request>::SharedPtr sport_request_pub_;

  /* Action servers. */
  rclcpp_action::Server<Arm>::SharedPtr arm_server_;
  rclcpp_action::Server<Disarm>::SharedPtr disarm_server_;

  /* Actions goal request handlers. */
  rclcpp_action::GoalResponse handle_arm_goal(
    const rclcpp_action::GoalUUID & uuid,
    ArmGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_disarm_goal(
    const rclcpp_action::GoalUUID & uuid,
    DisarmGoalSharedPtr goal);

  /* Actions cancellation request handlers. */
  rclcpp_action::CancelResponse handle_arm_cancel(
    const ArmGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_disarm_cancel(
    const DisarmGoalHandleSharedPtr goal_handle);

  /* Actions goal acceptance handlers. */
  void handle_arm_accepted(const ArmGoalHandleSharedPtr goal_handle);
  void handle_disarm_accepted(const DisarmGoalHandleSharedPtr goal_handle);

  /* Actions routines. */
  void arm(const ArmGoalHandleSharedPtr goal_handle);
  void disarm(const DisarmGoalHandleSharedPtr goal_handle);

  /* Synchronization primitives. */
  std::mutex operation_lock_;
  std::mutex state_lock_;
  std::mutex sportmode_state_lock_;
  std::condition_variable sportmode_state_cv_;
  std::atomic<bool> sportmode_state_received_;

  /* Internal state variables. */
  std::atomic<bool> armed_;
  std::atomic<bool> kill_switch_;
  pose_kit::Pose pose_;
  uint8_t sportmode_last_ = 0;

  /* Node parameters. */
  std::string frame_prefix_ = "";
  std::vector<double> imu_covariance_ = {};
  int64_t kill_switch_code_ = 0;
  int64_t kill_switch_release_code_ = 0;
  int64_t operations_stabilization_time_ = 0;
  int64_t operations_timeout_ = 0;
  bool pointcloud_deskewed_ = false;
  std::vector<double> pose_covariance_ = {};
  bool publish_tf_ = false;
  double velocity_control_vhorz_max_ = 0.0;
  double velocity_control_vyaw_max_ = 0.0;

  /* Node parameters validators. */
  bool validate_pose_covariance(const rclcpp::Parameter & p);
  bool validate_imu_covariance(const rclcpp::Parameter & p);

  /* Auxiliary routines. */
  void notify_sportmode_state();
};

} // go2_control

#endif // GO2_CONTROL__GO2_CONTROL_HPP
