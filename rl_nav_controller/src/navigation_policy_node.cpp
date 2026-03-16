#include "rl_nav_controller/navigation_policy_node.hpp"
#include "rl_nav_controller/constants.hpp"
#include "rl_nav_controller/quat_utils.hpp"

#include <cmath>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>

using namespace rl_nav_constants;

NavigationPolicyNode::NavigationPolicyNode()
    : Node("navigation_policy_node"),
      waypoint_manager_(this->get_logger())
{
    // Parameters
    this->declare_parameter("use_sim", false);
    use_sim_ = this->get_parameter("use_sim").as_bool();
    min_depth_ = kMinDepth;
    max_depth_ = kMaxDepth;
    control_frequency_ = kControlFrequency;
    odom_ready_ = false;
    arrive_goal_threshold_ = kArriveGoalThreshold;
    last_run_time_ = 0.0;
    system_delay_ = kJoystickTimeout;

    // Load ONNX models
    std::string pkg_path = ament_index_cpp::get_package_share_directory("rl_nav_controller");
    std::filesystem::path vae_path = std::filesystem::path(pkg_path) / "deployment_policies" / "vae_encoder.onnx";
    std::filesystem::path policy_path = std::filesystem::path(pkg_path) / "deployment_policies" / "nav_policy.onnx";
    model_ = std::make_unique<LearningModel>(vae_path, policy_path);

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/path_manager/path_manager_ros/nav_vel", 10);
    base_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/walle_nav/robot_base_vel", 10);
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);

    twist_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vis/twist_cmd_marker", 10);
    goal_vector_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vis/goal_vector_marker", 10);
    moving_goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vis/moving_goal_marker", 10);
    recorded_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("vis/recorded_waypoints_marker", 10);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/dlio/odom_node/odom", 10,
        std::bind(&NavigationPolicyNode::odomCallback, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/depth/depth_registered", 10,
        std::bind(&NavigationPolicyNode::depthCallback, this, std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/rsl_joy", 10,
        std::bind(&NavigationPolicyNode::joyCallback, this, std::placeholders::_1));
    target_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&NavigationPolicyNode::targetPositionCallback, this, std::placeholders::_1));

    // State initialization
    robot_odom_time_ = 0.0;
    is_reset_hidden_state_ = false;
    is_abort_goal_ = false;
    prev_cmd_ = {0.0f, 0.0f, 0.0f};
    resetLastAction();

    joy_linear_x_ = 0.0;
    joy_linear_y_ = 0.0;
    joy_angular_z_ = 0.0;
    cmd_vel_ratio_ = 1.0;
    joy_time_ = std::chrono::steady_clock::now();
    last_trigger_time_ = std::chrono::steady_clock::now();
    joy_active_ = false;

    moving_goal_delta_ = {0.0, 0.0, 0.0};
    smart_joystick_goal_ = {0.0, 0.0, 0.0};
    prev_smart_joystick_goal_ = {0.0, 0.0, 0.0};
    smart_joystick_mode_active_ = false;
    smart_joystick_goal_aborted_ = false;
    latest_joystick_axes_ = {0.0, 0.0, 0.0};

    // Timers
    waypoint_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(kWaypointPublishInterval),
        std::bind(&NavigationPolicyNode::publishRecordedWaypoints, this));
    target_vector_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(kTargetVectorPublishInterval),
        std::bind(&NavigationPolicyNode::publishTargetVector, this));
    smart_joystick_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / kSmartJoystickUpdateFrequency),
        std::bind(&NavigationPolicyNode::updateSmartJoystickGoal, this));

    RCLCPP_INFO(this->get_logger(), "\033[92mNavigation policy node is ready.\033[0m");
}

// ============================================================================
// Callbacks
// ============================================================================

void NavigationPolicyNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_odom_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (map_frame_id_.empty()) {
        map_frame_id_ = msg->header.frame_id;
    }
    if (robot_frame_id_.empty()) {
        robot_frame_id_ = msg->child_frame_id;
    }

    robot_pos_w_ = std::array<double, 3>{
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z};
    robot_orientation_w_ = std::array<double, 4>{
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z};

    linear_vel_w_ = {
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z};
    angular_vel_w_ = {
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z};

    if (use_sim_) {
        linear_vel_ = linear_vel_w_;
        angular_vel_ = angular_vel_w_;
    } else {
        linear_vel_ = convertVelFrame(linear_vel_w_, *robot_orientation_w_);
        angular_vel_ = convertVelFrame(angular_vel_w_, *robot_orientation_w_);
    }

    publishBaseVel(linear_vel_, angular_vel_);

    gravity_vector_ = projectedGravityVector(*robot_orientation_w_);

    if (!odom_ready_) {
        odom_ready_ = true;
    }
}

void NavigationPolicyNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!odom_ready_) {
        RCLCPP_WARN(this->get_logger(), "\033[93mOdometry not ready, skipping depth callback.\033[0m");
        return;
    }

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::Mat depth = cv_ptr->image;

        // Ensure float32
        if (depth.type() != CV_32F) {
            depth.convertTo(depth, CV_32F);
        }

        // Handle NaN, Inf, and out-of-range values
        for (int r = 0; r < depth.rows; r++) {
            float* row = depth.ptr<float>(r);
            for (int c = 0; c < depth.cols; c++) {
                float& v = row[c];
                if (std::isnan(v) || std::isinf(v) || v > max_depth_ || v < min_depth_) {
                    v = 0.0f;
                }
            }
        }
        depth_image_ = depth;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting depth image: %s", e.what());
        return;
    }

    // Control frequency gating
    double interval = 1.0 / control_frequency_;
    if ((robot_odom_time_ - last_run_time_) < interval) {
        return;
    }
    last_run_time_ = robot_odom_time_;

    generateCmdVel();
}

void NavigationPolicyNode::generateCmdVel() {
    // Visualize moving goal
    if (robot_pos_w_ && robot_orientation_w_) {
        vis_manager_.publishMovingGoalMarker(
            moving_goal_delta_, *robot_pos_w_, *robot_orientation_w_,
            robot_odom_time_, map_frame_id_, moving_goal_marker_pub_);
    }

    if (smart_joystick_mode_active_) {
        RCLCPP_INFO(this->get_logger(),
            "Smart joystick mode active - Goal: [%.2f, %.2f, %.2f]",
            smart_joystick_goal_[0], smart_joystick_goal_[1], smart_joystick_goal_[2]);
    }

    bool is_arrived = checkGoalReached(target_pos_w_, robot_pos_w_);
    geometry_msgs::msg::Twist twist;

    if (is_arrived || is_abort_goal_) {
        twist.linear.x = 0.0 + joy_linear_x_;
        twist.linear.y = 0.0 + joy_linear_y_;
        twist.angular.z = 0.0 + joy_angular_z_;
        cmd_vel_pub_->publish(twist);

        if (is_abort_goal_ && target_pos_w_) {
            waypoint_manager_.reAddAbortedWaypoint(*target_pos_w_);
        }

        target_pos_w_.reset();
        is_abort_goal_ = false;
        resetLastAction();
        RCLCPP_INFO(this->get_logger(), "Target position reset.");
    } else {
        // Joystick timeout
        auto now = std::chrono::steady_clock::now();
        double joy_elapsed = std::chrono::duration<double>(now - joy_time_).count();
        if (joy_elapsed > system_delay_) {
            if (joy_active_) {
                cmd_vel_ratio_ = 0.0;
                RCLCPP_WARN(this->get_logger(),
                    "\033[93mJoystick timeout, stopping the robot. Time diff: %.4fs\033[0m", joy_elapsed);
            } else {
                cmd_vel_ratio_ = 1.0;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Joystick cmd_vel ratio: %.4f", cmd_vel_ratio_);

        // Run policy inference
        auto result = model_->predict(
            linear_vel_, angular_vel_, gravity_vector_, last_action_,
            *target_pos_w_, *robot_pos_w_, *robot_orientation_w_,
            depth_image_, is_reset_hidden_state_);

        if (is_reset_hidden_state_) {
            RCLCPP_WARN(this->get_logger(), "\033[93mResetting hidden state.\033[0m");
            is_reset_hidden_state_ = false;
            resetLastAction();
        }

        last_action_ = {
            static_cast<double>(result.raw_action[0]),
            static_cast<double>(result.raw_action[1]),
            static_cast<double>(result.raw_action[2])};

        // Low-pass filter + joystick
        std::array<float, 3> model_cmd = {
            result.cmd_vel[0] * static_cast<float>(cmd_vel_ratio_),
            result.cmd_vel[1] * static_cast<float>(cmd_vel_ratio_) * kLateralVelocityScale,
            result.cmd_vel[2] * static_cast<float>(cmd_vel_ratio_)};

        for (int i = 0; i < 3; i++) {
            prev_cmd_[i] = kLowPassFilterCoef[i] * model_cmd[i] +
                           (1.0f - kLowPassFilterCoef[i]) * prev_cmd_[i];
        }

        twist.linear.x = static_cast<double>(prev_cmd_[0]) + joy_linear_x_;
        twist.linear.y = static_cast<double>(prev_cmd_[1]) + joy_linear_y_;
        twist.angular.z = static_cast<double>(prev_cmd_[2]) + joy_angular_z_;
        cmd_vel_pub_->publish(twist);

        vis_manager_.publishTwistMarker(twist, robot_odom_time_, robot_frame_id_, twist_marker_pub_);
    }

    RCLCPP_INFO(this->get_logger(),
        "Published cmd_vel: linear_x=%.4f, linear_y=%.4f, angular_z=%.4f",
        twist.linear.x, twist.linear.y, twist.angular.z);

    resetJoystick();
}

void NavigationPolicyNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    joy_active_ = true;
    if (msg->axes.size() > 4) {
        cmd_vel_ratio_ = (1.0 + msg->axes[4]) * 1.0;
    }

    if (static_cast<int>(msg->buttons.size()) > kButtonAbort &&
        msg->buttons[kButtonAbort] == 1) {
        is_abort_goal_ = true;
        RCLCPP_WARN(this->get_logger(), "Abort goal");
    }

    auto [dx, dy, dz] = movingXyzWithButtons(msg, kMovingScale);
    moving_goal_delta_[0] += dx;
    moving_goal_delta_[1] += dy;
    moving_goal_delta_[2] += dz;

    auto now = std::chrono::steady_clock::now();
    double cooldown = kTriggerButtonCooldown;
    double since_trigger = std::chrono::duration<double>(now - last_trigger_time_).count();

    if (!is_abort_goal_ && since_trigger > cooldown &&
        static_cast<int>(msg->buttons.size()) > kButtonSendGoal &&
        msg->buttons[kButtonSendGoal] == 1) {
        RCLCPP_WARN(this->get_logger(), "Trigger moving goal");
        publishMovingGoal();
        last_trigger_time_ = now;
    }

    if (since_trigger > cooldown &&
        static_cast<int>(msg->buttons.size()) > kButtonRecordWaypoint &&
        msg->buttons[kButtonRecordWaypoint] == 1) {
        if (robot_pos_w_) {
            waypoint_manager_.recordWaypoint(*robot_pos_w_);
        }
        last_trigger_time_ = now;
    }

    if (since_trigger > cooldown &&
        static_cast<int>(msg->buttons.size()) > kButtonClearWaypoint &&
        msg->buttons[kButtonClearWaypoint] == 1) {
        waypoint_manager_.removeLastWaypoint();
        last_trigger_time_ = now;
    }

    if (!is_abort_goal_ && since_trigger > cooldown &&
        static_cast<int>(msg->buttons.size()) > kButtonTriggerWaypoints &&
        msg->buttons[kButtonTriggerWaypoints] == 1) {
        RCLCPP_WARN(this->get_logger(), "Trigger waypoints");
        if (waypoint_manager_.hasHomeWaypoints()) {
            waypoint_manager_.startHomeWaypointSequence();
        } else if (waypoint_manager_.hasInversedWaypoints()) {
            waypoint_manager_.startInversedWaypointSequence();
        }
        last_trigger_time_ = now;
    }

    if (static_cast<int>(msg->axes.size()) > kJoystickAxisSmart &&
        msg->axes[kJoystickAxisSmart] < -0.5) {
        // Smart joystick mode
        if (!smart_joystick_mode_active_) {
            smart_joystick_mode_active_ = true;
            smart_joystick_goal_aborted_ = false;
        }

        if (!smart_joystick_goal_aborted_ && target_pos_w_) {
            is_abort_goal_ = true;
            smart_joystick_goal_aborted_ = true;
            RCLCPP_INFO(this->get_logger(), "Smart joystick mode: aborting current navigation goal");
        }

        latest_joystick_axes_ = {
            msg->axes[kJoystickAxisLinearX],
            msg->axes[kJoystickAxisLinearY],
            msg->axes[kJoystickAxisLinearZ]};
    } else {
        // Direct joystick control
        if (smart_joystick_mode_active_) {
            smart_joystick_mode_active_ = false;
            is_abort_goal_ = true;
            RCLCPP_INFO(this->get_logger(), "Exiting smart joystick mode: aborting current navigation goal");
        }

        resetSmartJoystickGoal();
        joy_linear_x_ = msg->axes[kJoystickAxisLinearX] * kLinearScale * 1.5;
        joy_linear_y_ = msg->axes[kJoystickAxisLinearY] * kLinearScale;
        joy_angular_z_ = msg->axes[kJoystickAxisAngularZ] * kAngularScale;
    }

    if (static_cast<int>(msg->buttons.size()) > kButtonResetHiddenState &&
        msg->buttons[kButtonResetHiddenState] == 1) {
        is_reset_hidden_state_ = true;
        RCLCPP_WARN(this->get_logger(), "Force Reset hidden state");
    }

    joy_time_ = std::chrono::steady_clock::now();
}

void NavigationPolicyNode::targetPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id != map_frame_id_) {
        RCLCPP_ERROR(this->get_logger(),
            "\033[91mTarget frame_id \"%s\" does not match odometry frame_id \"%s\"\033[0m",
            msg->header.frame_id.c_str(), map_frame_id_.c_str());
        return;
    }

    double goal_z = msg->pose.position.z;
    if (std::abs(goal_z) < 1e-3) {
        if (robot_pos_w_) {
            goal_z = (*robot_pos_w_)[2];
        } else {
            RCLCPP_WARN(this->get_logger(), "Robot position not available, using received z for target.");
        }
    }

    target_pos_w_ = std::array<double, 3>{msg->pose.position.x, msg->pose.position.y, goal_z};
    last_target_pos_ = target_pos_w_;

    RCLCPP_INFO(this->get_logger(),
        "Received target position: [%.2f, %.2f, %.2f], robot height: %.2f",
        (*target_pos_w_)[0], (*target_pos_w_)[1], (*target_pos_w_)[2],
        robot_pos_w_ ? (*robot_pos_w_)[2] : 0.0);
}

// ============================================================================
// Timer callbacks
// ============================================================================

void NavigationPolicyNode::publishRecordedWaypoints() {
    if (!robot_pos_w_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for robot position to initialize.");
        return;
    }

    vis_manager_.publishWaypointsMarker(
        waypoint_manager_.waypointsVisualization(),
        map_frame_id_, this->get_clock(), recorded_waypoints_marker_pub_);

    // Publish home waypoints
    if (waypoint_manager_.isHomeSequenceActive() && waypoint_manager_.hasHomeWaypoints()) {
        if (!target_pos_w_ || checkNearGoal(target_pos_w_, robot_pos_w_)) {
            auto next_wp = waypoint_manager_.getNextWaypointHome();
            RCLCPP_INFO(this->get_logger(), "Publishing next waypoint: [%.2f, %.2f, %.2f]",
                        next_wp[0], next_wp[1], next_wp[2]);
            publishGoal(next_wp);
        } else {
            RCLCPP_INFO(this->get_logger(), "Tracking the current waypoint ...");
        }

        if (!waypoint_manager_.hasHomeWaypoints()) {
            waypoint_manager_.stopHomeWaypointSequence();
        }
    }

    // Publish inversed waypoints
    if (waypoint_manager_.isInversedSequenceActive() && waypoint_manager_.hasInversedWaypoints()) {
        if (!target_pos_w_ || checkNearGoal(target_pos_w_, robot_pos_w_)) {
            auto next_wp = waypoint_manager_.getNextWaypointInversed();
            RCLCPP_INFO(this->get_logger(), "Publishing next inversed waypoint: [%.2f, %.2f, %.2f]",
                        next_wp[0], next_wp[1], next_wp[2]);
            publishGoal(next_wp);
        } else {
            RCLCPP_INFO(this->get_logger(), "Tracking the current waypoint ...");
        }

        if (!waypoint_manager_.hasInversedWaypoints()) {
            waypoint_manager_.stopInversedWaypointSequence();
        }
    }

    waypoint_manager_.resetVisualizationIfComplete();
}

void NavigationPolicyNode::publishTargetVector() {
    if (!robot_pos_w_ || !robot_orientation_w_) return;

    auto tgt = target_pos_w_ ? target_pos_w_ : last_target_pos_;
    if (!tgt) return;

    auto [target_pos_log, target_vec_b] = model_->normalizeTargetPosition(
        *tgt, *robot_pos_w_, *robot_orientation_w_);

    vis_manager_.publishTargetVectorMarker(
        target_vec_b, robot_odom_time_, robot_frame_id_, goal_vector_marker_pub_);
}

void NavigationPolicyNode::updateSmartJoystickGoal() {
    if (!smart_joystick_mode_active_) return;

    generateWaypointUsingJoystick(
        latest_joystick_axes_[0], latest_joystick_axes_[1], latest_joystick_axes_[2]);
    publishSmartJoystickGoal();
}

// ============================================================================
// Helpers
// ============================================================================

std::vector<double> NavigationPolicyNode::convertVelFrame(
    const std::vector<double>& vel_vec,
    const std::array<double, 4>& orientation_w)
{
    // orientation_w is (w, x, y, z)
    Eigen::Quaterniond q(orientation_w[0], orientation_w[1], orientation_w[2], orientation_w[3]);
    Eigen::Vector3d vel(vel_vec[0], vel_vec[1], vel_vec[2]);
    Eigen::Vector3d vel_base = q.conjugate() * vel;
    return {vel_base.x(), vel_base.y(), vel_base.z()};
}

std::vector<double> NavigationPolicyNode::projectedGravityVector(
    const std::array<double, 4>& robot_orientation_w)
{
    Eigen::Quaterniond q(robot_orientation_w[0], robot_orientation_w[1],
                         robot_orientation_w[2], robot_orientation_w[3]);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    Eigen::Vector3d proj = q.conjugate() * gravity;
    double norm = proj.norm() + 1e-6;
    proj /= norm;
    return {proj.x(), proj.y(), proj.z()};
}

void NavigationPolicyNode::publishBaseVel(
    const std::vector<double>& linear_vel,
    const std::vector<double>& angular_vel)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_vel[0];
    twist.linear.y = linear_vel[1];
    twist.angular.z = angular_vel[2];
    base_vel_pub_->publish(twist);
}

bool NavigationPolicyNode::checkGoalReached(
    const std::optional<std::array<double, 3>>& target,
    const std::optional<std::array<double, 3>>& robot)
{
    if (!target || !robot) {
        RCLCPP_WARN(this->get_logger(), "Waiting for target position.");
        return true;
    }
    double dx = (*target)[0] - (*robot)[0];
    double dy = (*target)[1] - (*robot)[1];
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > arrive_goal_threshold_) {
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Arrived at the goal position.");
    return true;
}

bool NavigationPolicyNode::checkNearGoal(
    const std::optional<std::array<double, 3>>& target,
    const std::optional<std::array<double, 3>>& robot)
{
    if (!target || !robot) {
        RCLCPP_WARN(this->get_logger(), "Waiting for target position.");
        return true;
    }
    double dx = (*target)[0] - (*robot)[0];
    double dy = (*target)[1] - (*robot)[1];
    double dist = std::sqrt(dx * dx + dy * dy);
    double threshold = arrive_goal_threshold_ * kNearGoalThresholdMultiplier;
    if (dist > threshold) {
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Near the current goal position.");
    return true;
}

void NavigationPolicyNode::publishGoal(const std::array<double, 3>& goal_pos) {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = rclcpp::Time(static_cast<int64_t>(robot_odom_time_ * 1e9));
    goal_pose.header.frame_id = map_frame_id_;
    goal_pose.pose.position.x = goal_pos[0];
    goal_pose.pose.position.y = goal_pos[1];
    goal_pose.pose.position.z = goal_pos[2];
    goal_pose_pub_->publish(goal_pose);
}

void NavigationPolicyNode::publishMovingGoal() {
    if (!robot_pos_w_ || !robot_orientation_w_) {
        RCLCPP_WARN(this->get_logger(), "Cannot publish moving goal: robot pose/orientation not yet set.");
        return;
    }

    Eigen::Vector3f robot_pos(
        static_cast<float>((*robot_pos_w_)[0]),
        static_cast<float>((*robot_pos_w_)[1]),
        static_cast<float>((*robot_pos_w_)[2]));
    Eigen::Vector4f robot_ori(
        static_cast<float>((*robot_orientation_w_)[0]),
        static_cast<float>((*robot_orientation_w_)[1]),
        static_cast<float>((*robot_orientation_w_)[2]),
        static_cast<float>((*robot_orientation_w_)[3]));
    Eigen::Vector4f robot_yaw_ori = rl_nav_quat::yaw_quat(robot_ori);

    Eigen::Vector3f goal_delta(
        static_cast<float>(moving_goal_delta_[0]),
        static_cast<float>(moving_goal_delta_[1]),
        static_cast<float>(moving_goal_delta_[2]));

    Eigen::Vector3f goal_world = rl_nav_quat::transform_point(goal_delta, robot_pos, robot_yaw_ori);

    publishGoal({
        static_cast<double>(goal_world(0)),
        static_cast<double>(goal_world(1)),
        static_cast<double>(goal_world(2))});

    resetMovingGoal();
}

void NavigationPolicyNode::generateWaypointUsingJoystick(
    double linear_x, double linear_y, double linear_z)
{
    if (!robot_pos_w_ || !robot_orientation_w_) return;

    Eigen::Vector3f goal_offset(
        static_cast<float>(linear_x * kSmartJoystickScale),
        static_cast<float>(linear_y * kSmartJoystickScale),
        static_cast<float>(linear_z * kSmartJoystickScale * kSmartJoystickZScale));

    Eigen::Vector4f robot_ori(
        static_cast<float>((*robot_orientation_w_)[0]),
        static_cast<float>((*robot_orientation_w_)[1]),
        static_cast<float>((*robot_orientation_w_)[2]),
        static_cast<float>((*robot_orientation_w_)[3]));
    Eigen::Vector4f robot_yaw_ori = rl_nav_quat::yaw_quat(robot_ori);

    // Rotation only (zero translation)
    Eigen::Vector3f zero_pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f goal_offset_world = rl_nav_quat::transform_point(goal_offset, zero_pos, robot_yaw_ori);

    std::array<double, 3> target_goal_world = {
        (*robot_pos_w_)[0] + static_cast<double>(goal_offset_world(0)),
        (*robot_pos_w_)[1] + static_cast<double>(goal_offset_world(1)),
        (*robot_pos_w_)[2] + static_cast<double>(goal_offset_world(2))};

    bool is_first = (prev_smart_joystick_goal_ == std::array<double, 3>{0.0, 0.0, 0.0});
    if (is_first) {
        smart_joystick_goal_ = target_goal_world;
    } else {
        double alpha = kSmartJoystickFilterAlpha;
        for (int i = 0; i < 3; i++) {
            smart_joystick_goal_[i] =
                alpha * target_goal_world[i] + (1.0 - alpha) * prev_smart_joystick_goal_[i];
        }
    }

    prev_smart_joystick_goal_ = smart_joystick_goal_;
}

void NavigationPolicyNode::publishSmartJoystickGoal() {
    if (!robot_pos_w_ || !robot_orientation_w_) {
        RCLCPP_WARN(this->get_logger(), "Cannot publish smart joystick goal: robot pose not available");
        return;
    }
    publishGoal(smart_joystick_goal_);
}

std::array<double, 3> NavigationPolicyNode::movingXyzWithButtons(
    const sensor_msgs::msg::Joy::SharedPtr& msg, double scale)
{
    auto btn = [&](int idx) -> bool {
        return static_cast<int>(msg->buttons.size()) > idx && msg->buttons[idx] != 0;
    };

    double dx = scale * (static_cast<double>(btn(kButtonForward)) - static_cast<double>(btn(kButtonBackward)));
    double dy = scale * (static_cast<double>(btn(kButtonLeft)) - static_cast<double>(btn(kButtonRight)));
    double dz = (scale / 5.0) * (static_cast<double>(btn(kButtonUp)) - static_cast<double>(btn(kButtonDown)));
    return {dx, dy, dz};
}

void NavigationPolicyNode::resetLastAction() {
    last_action_ = {0.0, 0.0, 0.0};
    prev_cmd_ = {0.0f, 0.0f, 0.0f};
}

void NavigationPolicyNode::resetJoystick() {
    joy_linear_x_ = 0.0;
    joy_linear_y_ = 0.0;
    joy_angular_z_ = 0.0;
}

void NavigationPolicyNode::resetMovingGoal() {
    moving_goal_delta_ = {0.0, 0.0, 0.0};
    RCLCPP_INFO(this->get_logger(), "Reset moving goal delta.");
}

void NavigationPolicyNode::resetSmartJoystickGoal() {
    smart_joystick_goal_ = {0.0, 0.0, 0.0};
    prev_smart_joystick_goal_ = {0.0, 0.0, 0.0};
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationPolicyNode>();

    RCLCPP_INFO(node->get_logger(),
        "RL Navigation launched in %s mode.",
        node->get_parameter("use_sim").as_bool() ? "SIMULATION" : "REAL-HARDWARE");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
