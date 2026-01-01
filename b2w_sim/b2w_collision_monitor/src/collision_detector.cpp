#include "b2w_collision_monitor/collision_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>

namespace b2w_collision_monitor
{

CollisionDetector::CollisionDetector(const rclcpp::NodeOptions& options)
  : Node("b2w_collision_detector", options)
{
  // Declare and get parameters
  this->declare_parameter("torque_threshold", 200.0);
  this->declare_parameter("cooldown_period_ms", 5000);
  this->declare_parameter("enable_force_torque", true);
  this->declare_parameter("enable_pointcloud", false);
  this->declare_parameter("force_torque_topic", "/collision_plate/force_torque");
  this->declare_parameter("pointcloud_topic", "/collision_cloud");

  torque_threshold_ = this->get_parameter("torque_threshold").as_double();
  cooldown_period_ms_ = this->get_parameter("cooldown_period_ms").as_int();
  enable_force_torque_ = this->get_parameter("enable_force_torque").as_bool();
  enable_pointcloud_ = this->get_parameter("enable_pointcloud").as_bool();

  std::string force_torque_topic = this->get_parameter("force_torque_topic").as_string();
  std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();

  // Initialize last alert time with cooldown in the past
  last_alert_time_ = this->get_clock()->now() - rclcpp::Duration::from_nanoseconds(cooldown_period_ms_ * 1000000L);

  // Subscribe to force-torque sensor if enabled
  if (enable_force_torque_) {
    force_torque_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_torque_topic, 10,
      std::bind(&CollisionDetector::force_torque_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Force-torque collision detection enabled on topic: %s",
                force_torque_topic.c_str());
  }

  // Subscribe to pointcloud if enabled
  if (enable_pointcloud_) {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&CollisionDetector::pointcloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Pointcloud collision detection enabled on topic: %s",
                pointcloud_topic.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "B2W Collision Detector initialized");
  RCLCPP_INFO(this->get_logger(), "Torque threshold: %.2f N⋅m", torque_threshold_);
  RCLCPP_INFO(this->get_logger(), "Cooldown period: %d ms", cooldown_period_ms_);
}

void CollisionDetector::force_torque_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null force-torque message");
    return;
  }

  double tx = msg->wrench.torque.x;
  double ty = msg->wrench.torque.y;
  double tz = msg->wrench.torque.z;

  // Validate sensor data
  if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz)) {
    RCLCPP_WARN(this->get_logger(), "Invalid torque values: tx=%f, ty=%f, tz=%f", tx, ty, tz);
    return;
  }

  double combined_torque = std::sqrt(tx * tx + ty * ty + tz * tz);

  if (!std::isfinite(combined_torque)) {
    RCLCPP_WARN(this->get_logger(), "Invalid combined torque: %f", combined_torque);
    return;
  }

  // Log periodically for debugging
  static int log_counter = 0;
  if (++log_counter % 100 == 0) {
    RCLCPP_DEBUG(this->get_logger(), "Combined torque: %.3f N⋅m (threshold: %.1f)",
                 combined_torque, torque_threshold_);
  }

  // Check collision
  check_collision(combined_torque, tx, ty);
}

void CollisionDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null pointcloud message");
    return;
  }

  // Pointcloud-based collision detection can be implemented here
  // This is a placeholder for future implementation
  RCLCPP_DEBUG(this->get_logger(), "Received pointcloud with %u points", msg->width * msg->height);
}

void CollisionDetector::check_collision(double magnitude, double tx, double ty)
{
  auto current_time = this->get_clock()->now();
  auto time_since_last_alert = current_time - last_alert_time_;

  // Check if threshold exceeded and cooldown period has passed
  if (magnitude > torque_threshold_ &&
      time_since_last_alert.nanoseconds() > (cooldown_period_ms_ * 1000000L)) {

    RCLCPP_WARN(this->get_logger(), "========================================");
    RCLCPP_WARN(this->get_logger(), "COLLISION DETECTED!");
    RCLCPP_WARN(this->get_logger(), "Combined Torque: %.3f N⋅m (threshold: %.1f N⋅m)",
                magnitude, torque_threshold_);
    RCLCPP_WARN(this->get_logger(), "Torque X: %.3f N⋅m", tx);
    RCLCPP_WARN(this->get_logger(), "Torque Y: %.3f N⋅m", ty);
    RCLCPP_WARN(this->get_logger(), "========================================");

    // Update last alert time
    last_alert_time_ = current_time;
  }
}

} // end namespace b2w_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(b2w_collision_monitor::CollisionDetector)
