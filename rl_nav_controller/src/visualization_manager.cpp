#include "rl_nav_controller/visualization_manager.hpp"
#include "rl_nav_controller/constants.hpp"
#include "rl_nav_controller/quat_utils.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

using visualization_msgs::msg::Marker;

void VisualizationManager::publishTwistMarker(
    const geometry_msgs::msg::Twist& twist_msg,
    double robot_odom_time,
    const std::string& robot_frame_id,
    rclcpp::Publisher<Marker>::SharedPtr publisher)
{
    if (robot_frame_id.empty()) return;

    Marker marker;
    marker.header.stamp = rclcpp::Time(static_cast<int64_t>(robot_odom_time * 1e9));
    marker.header.frame_id = robot_frame_id;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.id = rl_nav_constants::kTwistMarkerId;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0; start.y = 0.0; start.z = 0.0;
    end.x = twist_msg.linear.x * rl_nav_constants::kTwistMarkerScale;
    end.y = twist_msg.linear.y * rl_nav_constants::kTwistMarkerScale;
    end.z = 0.0;
    marker.points = {start, end};

    marker.scale.x = 0.2;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = false;

    publisher->publish(marker);
}

void VisualizationManager::publishTargetVectorMarker(
    const Eigen::Vector3f& target_vec_b,
    double robot_odom_time,
    const std::string& robot_frame_id,
    rclcpp::Publisher<Marker>::SharedPtr publisher)
{
    if (robot_frame_id.empty()) return;

    Marker marker;
    marker.header.stamp = rclcpp::Time(static_cast<int64_t>(robot_odom_time * 1e9));
    marker.header.frame_id = robot_frame_id;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.id = rl_nav_constants::kTargetVectorMarkerId;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0; start.y = 0.0; start.z = 0.0;
    end.x = static_cast<double>(target_vec_b(0));
    end.y = static_cast<double>(target_vec_b(1));
    end.z = static_cast<double>(target_vec_b(2));
    marker.points = {start, end};

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;
    marker.frame_locked = false;

    publisher->publish(marker);
}

void VisualizationManager::publishMovingGoalMarker(
    const Point3d& moving_goal_delta,
    const Point3d& robot_pos_w,
    const std::array<double, 4>& robot_orientation_w,
    double robot_odom_time,
    const std::string& map_frame_id,
    rclcpp::Publisher<Marker>::SharedPtr publisher)
{
    if (map_frame_id.empty()) return;

    Eigen::Vector3f robot_pos(
        static_cast<float>(robot_pos_w[0]),
        static_cast<float>(robot_pos_w[1]),
        static_cast<float>(robot_pos_w[2]));
    Eigen::Vector4f robot_ori(
        static_cast<float>(robot_orientation_w[0]),
        static_cast<float>(robot_orientation_w[1]),
        static_cast<float>(robot_orientation_w[2]),
        static_cast<float>(robot_orientation_w[3]));
    Eigen::Vector4f robot_yaw_ori = rl_nav_quat::yaw_quat(robot_ori);
    Eigen::Vector3f goal_delta(
        static_cast<float>(moving_goal_delta[0]),
        static_cast<float>(moving_goal_delta[1]),
        static_cast<float>(moving_goal_delta[2]));

    Eigen::Vector3f moving_goal_pos_w = rl_nav_quat::transform_point(goal_delta, robot_pos, robot_yaw_ori);

    Marker marker;
    marker.header.stamp = rclcpp::Time(static_cast<int64_t>(robot_odom_time * 1e9));
    marker.header.frame_id = map_frame_id;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.id = rl_nav_constants::kMovingGoalMarkerId;

    geometry_msgs::msg::Point start, end;
    start.x = static_cast<double>(moving_goal_pos_w(0));
    start.y = static_cast<double>(moving_goal_pos_w(1));
    start.z = static_cast<double>(moving_goal_pos_w(2)) + 1.0;
    end.x = static_cast<double>(moving_goal_pos_w(0));
    end.y = static_cast<double>(moving_goal_pos_w(1));
    end.z = static_cast<double>(moving_goal_pos_w(2));
    marker.points = {start, end};

    marker.scale.x = 0.3;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = false;

    publisher->publish(marker);
}

void VisualizationManager::publishWaypointsMarker(
    const std::deque<Point3d>& waypoints,
    const std::string& map_frame_id,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<Marker>::SharedPtr publisher)
{
    if (map_frame_id.empty()) return;

    Marker marker;
    marker.header.stamp = clock->now();
    marker.header.frame_id = map_frame_id;
    marker.ns = "recorded_waypoints";
    marker.id = rl_nav_constants::kWaypointsMarkerId;
    marker.type = Marker::CUBE_LIST;
    marker.action = Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for (const auto& wp : waypoints) {
        geometry_msgs::msg::Point p;
        p.x = wp[0];
        p.y = wp[1];
        p.z = wp[2];
        marker.points.push_back(p);
    }

    publisher->publish(marker);
}
