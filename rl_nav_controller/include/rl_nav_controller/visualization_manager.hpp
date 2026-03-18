#pragma once

#include <array>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>

class VisualizationManager {
public:
    using Point3d = std::array<double, 3>;

    void publishTwistMarker(
        const geometry_msgs::msg::Twist& twist_msg,
        double robot_odom_time,
        const std::string& robot_frame_id,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);

    void publishTargetVectorMarker(
        const Eigen::Vector3f& target_vec_b,
        double robot_odom_time,
        const std::string& robot_frame_id,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);

    void publishMovingGoalMarker(
        const Point3d& moving_goal_delta,
        const Point3d& robot_pos_w,
        const std::array<double, 4>& robot_orientation_w,
        double robot_odom_time,
        const std::string& map_frame_id,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);

    void publishWaypointsMarker(
        const std::deque<Point3d>& waypoints,
        const std::string& map_frame_id,
        rclcpp::Clock::SharedPtr clock,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);
};
