/**
 * Go2 Control node implementation.
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
 * @brief Go2 Control node constructor.
 *
 * @param node_options Node options.
 */
Go2Control::Go2Control(const rclcpp::NodeOptions & node_options)
: NodeBase("go2_control", node_options, true)
{
  init_atomics();
  init_cgroups();
  init_parameters();
  init_subscriptions();
  init_tf2();
  init_publishers();
  init_services();
  init_actions();

  // Set obstacle avoidance
  set_obstacle_avoidance(this->get_parameter("init_obstacle_avoidance").as_bool());

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Go2 Control node destructor.
 */
Go2Control::~Go2Control()
{
  // Destroy image_transport publishers
  front_image_pub_->shutdown();
  front_image_pub_.reset();
}

/**
 * @brief Routine to initialize atomic members.
 */
void Go2Control::init_atomics()
{
  armed_.store(false, std::memory_order_release);
  kill_switch_.store(false, std::memory_order_release);
  init_pose_ok_.store(false, std::memory_order_release);
  sportmode_state_received_.store(true, std::memory_order_release);
}

/**
 * @brief Routine to initialize callback groups.
 */
void Go2Control::init_cgroups()
{
  // Topic subscriptions
  cmd_vel_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  foot_position_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  front_video_data_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  lowstate_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  point_cloud_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  pose_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  sportmode_state_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  wireless_controller_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void Go2Control::init_subscriptions()
{
  // cmd_vel
  rclcpp::SubscriptionOptions cmd_vel_options{};
  cmd_vel_options.callback_group = cmd_vel_callback_group_;
  cmd_vel_sub_ = this->create_subscription<Twist>(
    "~/cmd_vel",
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &Go2Control::cmd_vel_callback,
      this,
      std::placeholders::_1),
    cmd_vel_options);

  // foot_position
  rclcpp::SubscriptionOptions foot_position_options{};
  foot_position_options.callback_group = foot_position_callback_group_;
  foot_position_sub_ = this->create_subscription<PointCloud2>(
    "/utlidar/foot_position",
    dua_qos::Reliable::get_scan_qos(),
    std::bind(
      &Go2Control::foot_position_callback,
      this,
      std::placeholders::_1),
    foot_position_options);

  // frontvideostream
  rclcpp::SubscriptionOptions front_video_data_options{};
  front_video_data_options.callback_group = front_video_data_callback_group_;
  front_video_data_sub_ = this->create_subscription<Go2FrontVideoData>(
    "/frontvideostream",
    dua_qos::Reliable::get_image_qos(),
    std::bind(
      &Go2Control::front_video_data_callback,
      this,
      std::placeholders::_1),
    front_video_data_options);

  // lowstate
  rclcpp::SubscriptionOptions lowstate_options{};
  lowstate_options.callback_group = lowstate_callback_group_;
  lowstate_sub_ = this->create_subscription<LowState>(
    "/lf/lowstate",
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &Go2Control::lowstate_callback,
      this,
      std::placeholders::_1),
    lowstate_options);

  // cloud
  rclcpp::SubscriptionOptions point_cloud_options{};
  point_cloud_options.callback_group = point_cloud_callback_group_;
  if (pointcloud_deskewed_) {
    point_cloud_sub_ = this->create_subscription<PointCloud2>(
      "/utlidar/cloud_deskewed",
      dua_qos::Reliable::get_scan_qos(),
      std::bind(
        &Go2Control::point_cloud_callback,
        this,
        std::placeholders::_1),
      point_cloud_options);
  } else {
    point_cloud_sub_ = this->create_subscription<PointCloud2>(
      "/utlidar/cloud",
      dua_qos::Reliable::get_scan_qos(),
      std::bind(
        &Go2Control::point_cloud_callback,
        this,
        std::placeholders::_1),
      point_cloud_options);
  }

  // robot_odom
  rclcpp::SubscriptionOptions odom_options{};
  odom_options.callback_group = odom_callback_group_;
  odom_sub_ = this->create_subscription<Odometry>(
    "/utlidar/robot_odom",
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &Go2Control::odometry_callback,
      this,
      std::placeholders::_1),
    odom_options);

  // robot_pose
  rclcpp::SubscriptionOptions pose_options{};
  pose_options.callback_group = pose_callback_group_;
  pose_sub_ = this->create_subscription<PoseStamped>(
    "/utlidar/robot_pose",
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &Go2Control::pose_callback,
      this,
      std::placeholders::_1),
    pose_options);

  // sportmodestate
  rclcpp::SubscriptionOptions sportmode_state_options{};
  sportmode_state_options.callback_group = sportmode_state_callback_group_;
  sportmode_state_sub_ = this->create_subscription<SportModeState>(
    "/lf/sportmodestate",
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &Go2Control::sportmode_state_callback,
      this,
      std::placeholders::_1),
    sportmode_state_options);

  // wirelesscontroller
  rclcpp::SubscriptionOptions wireless_controller_options{};
  wireless_controller_options.callback_group = wireless_controller_callback_group_;
  wireless_controller_sub_ = this->create_subscription<WirelessController>(
    "/wirelesscontroller",
    dua_qos::Reliable::get_datum_qos(),
    std::bind(
      &Go2Control::wireless_controller_callback,
      this,
      std::placeholders::_1),
    wireless_controller_options);
}

/**
 * @brief Routine to initialize TF2 entities.
 */
void Go2Control::init_tf2()
{
  // Initialize frame names
  body_frame_ = frame_prefix_ + "base_link";
  local_frame_ = frame_prefix_ + "odom";

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Initialize TF listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void Go2Control::init_publishers()
{
  // battery_state
  battery_state_pub_ = this->create_publisher<BatteryState>(
    "~/battery_state",
    dua_qos::Reliable::get_datum_qos());

  // foot_position
  foot_position_pub_ = this->create_publisher<PointCloud2>(
    "~/foot_position",
    dua_qos::Reliable::get_scan_qos());

  // imu
  imu_pub_ = this->create_publisher<Imu>(
    "~/imu",
    dua_qos::Reliable::get_datum_qos());

  // joint_states
  joint_states_pub_ = this->create_publisher<JointState>(
    "~/joint_states",
    dua_qos::Reliable::get_datum_qos());

  // obstacles_avoid_request
  obstacle_avoidance_request_pub_ = this->create_publisher<Request>(
    "/api/obstacles_avoid/request",
    dua_qos::Reliable::get_datum_qos());

  // odometry
  odom_pub_ = this->create_publisher<Odometry>(
    "~/odometry",
    dua_qos::Reliable::get_datum_qos());

  // point_cloud
  point_cloud_pub_ = this->create_publisher<PointCloud2>(
    "~/utlidar_cloud",
    dua_qos::Reliable::get_scan_qos());

  // pose
  pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/pose",
    dua_qos::Reliable::get_datum_qos());

  // sport_request
  sport_request_pub_ = this->create_publisher<Request>(
    "/api/sport/request",
    dua_qos::Reliable::get_datum_qos());

  // front/image_rect_color
  front_image_pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      this,
      "~/front/image_rect_color",
      dua_qos::Reliable::get_image_qos().get_rmw_qos_profile()));
}

/**
 * @brief Routine to initialize service servers.
 */
void Go2Control::init_services()
{
  // actions
  actions_srv_ = this->create_service<Action>(
    "~/actions",
    std::bind(
      &Go2Control::actions_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  // obstacle_avoidance
  obstacle_avoidance_srv_ = this->create_service<SetBool>(
    "~/obstacle_avoidance",
    std::bind(
      &Go2Control::obstacle_avoidance_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

/**
 * @brief Routine to initialize action servers.
 */
void Go2Control::init_actions()
{
  // arm
  arm_server_ = rclcpp_action::create_server<Arm>(
    this,
    "~/arm",
    std::bind(
      &Go2Control::handle_arm_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &Go2Control::handle_arm_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &Go2Control::handle_arm_accepted,
      this,
      std::placeholders::_1));

  // disarm
  disarm_server_ = rclcpp_action::create_server<Disarm>(
    this,
    "~/disarm",
    std::bind(
      &Go2Control::handle_disarm_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &Go2Control::handle_disarm_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &Go2Control::handle_disarm_accepted,
      this,
      std::placeholders::_1));
}

} // namespace go2_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(go2_control::Go2Control)
