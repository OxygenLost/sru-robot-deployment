#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace b2w_collision_monitor
{

class CollisionDetector : public rclcpp::Node
{
public:
  explicit CollisionDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callback for force-torque sensor data
  void force_torque_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // Callback for pointcloud-based collision detection (if available)
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Check if collision is detected and log/alert
  void check_collision(double tx, double ty, double tz);

  // ROS 2 subscriptions
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // Collision detection parameters
  double torque_threshold_;
  int cooldown_period_ms_;
  rclcpp::Time last_alert_time_;
  bool enable_force_torque_;
  bool enable_pointcloud_;
};

} // end namespace b2w_collision_monitor

#endif // COLLISION_DETECTOR_H