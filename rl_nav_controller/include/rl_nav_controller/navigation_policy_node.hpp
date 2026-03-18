#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "rl_nav_controller/learning_model.hpp"
#include "rl_nav_controller/waypoint_manager.hpp"
#include "rl_nav_controller/visualization_manager.hpp"

class NavigationPolicyNode : public rclcpp::Node {
public:
    NavigationPolicyNode();

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void targetPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Timer callbacks
    void publishRecordedWaypoints();
    void publishTargetVector();
    void updateSmartJoystickGoal();

    // Core logic
    void generateCmdVel();

    // Helpers
    std::vector<double> convertVelFrame(const std::vector<double>& vel_vec,
                                        const std::array<double, 4>& orientation_w);
    std::vector<double> projectedGravityVector(const std::array<double, 4>& robot_orientation_w);
    void publishBaseVel(const std::vector<double>& linear_vel, const std::vector<double>& angular_vel);
    bool checkGoalReached(const std::optional<std::array<double, 3>>& target,
                          const std::optional<std::array<double, 3>>& robot);
    bool checkNearGoal(const std::optional<std::array<double, 3>>& target,
                       const std::optional<std::array<double, 3>>& robot);
    void publishGoal(const std::array<double, 3>& goal_pos);
    void publishMovingGoal();
    void generateWaypointUsingJoystick(double linear_x, double linear_y, double linear_z);
    void publishSmartJoystickGoal();
    std::array<double, 3> movingXyzWithButtons(const sensor_msgs::msg::Joy::SharedPtr& msg, double scale);
    void resetLastAction();
    void resetJoystick();
    void resetMovingGoal();
    void resetSmartJoystickGoal();

    // Configuration
    bool use_sim_;
    double min_depth_;
    double max_depth_;
    double control_frequency_;
    bool odom_ready_;
    double arrive_goal_threshold_;
    double last_run_time_;
    double system_delay_;

    // Components
    std::unique_ptr<LearningModel> model_;
    WaypointManager waypoint_manager_;
    VisualizationManager vis_manager_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr twist_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_vector_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr moving_goal_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr recorded_waypoints_marker_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr waypoint_timer_;
    rclcpp::TimerBase::SharedPtr target_vector_timer_;
    rclcpp::TimerBase::SharedPtr smart_joystick_timer_;

    // Robot state
    std::string map_frame_id_;
    std::string robot_frame_id_;
    double robot_odom_time_;
    std::optional<std::array<double, 3>> robot_pos_w_;
    std::optional<std::array<double, 4>> robot_orientation_w_;
    std::vector<double> linear_vel_w_;
    std::vector<double> angular_vel_w_;
    std::vector<double> linear_vel_;
    std::vector<double> angular_vel_;
    std::vector<double> gravity_vector_;
    cv::Mat depth_image_;

    // Navigation state
    std::optional<std::array<double, 3>> target_pos_w_;
    std::optional<std::array<double, 3>> last_target_pos_;
    bool is_reset_hidden_state_;
    std::vector<double> last_action_;
    std::array<float, 3> prev_cmd_;
    bool is_abort_goal_;

    // Joystick state
    double joy_linear_x_;
    double joy_linear_y_;
    double joy_angular_z_;
    double cmd_vel_ratio_;
    std::chrono::steady_clock::time_point joy_time_;
    std::chrono::steady_clock::time_point last_trigger_time_;
    bool joy_active_;

    // Moving goal state
    std::array<double, 3> moving_goal_delta_;

    // Smart joystick state
    std::array<double, 3> smart_joystick_goal_;
    std::array<double, 3> prev_smart_joystick_goal_;
    bool smart_joystick_mode_active_;
    bool smart_joystick_goal_aborted_;
    std::array<double, 3> latest_joystick_axes_;
};
