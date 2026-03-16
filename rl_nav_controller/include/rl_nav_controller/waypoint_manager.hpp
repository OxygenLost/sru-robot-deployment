#pragma once

#include <array>
#include <deque>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class WaypointManager {
public:
    using Point3d = std::array<double, 3>;

    explicit WaypointManager(rclcpp::Logger logger);

    bool recordWaypoint(const Point3d& robot_pos_w);
    bool removeLastWaypoint();

    Point3d getNextWaypointHome();
    Point3d getNextWaypointInversed();

    bool startHomeWaypointSequence();
    bool startInversedWaypointSequence();
    void stopHomeWaypointSequence();
    void stopInversedWaypointSequence();

    bool resetVisualizationIfComplete();
    void reAddAbortedWaypoint(const Point3d& target_pos_w);

    bool hasHomeWaypoints() const { return !recorded_waypoints_.empty(); }
    bool hasInversedWaypoints() const { return !inversed_ordered_waypoints_.empty(); }
    bool isHomeSequenceActive() const { return is_publish_waypoints_home_; }
    bool isInversedSequenceActive() const { return is_publish_waypoints_inversed_; }

    const std::deque<Point3d>& waypointsVisualization() const { return waypoints_visualization_; }

private:
    rclcpp::Logger logger_;
    std::deque<Point3d> recorded_waypoints_;
    std::deque<Point3d> inversed_ordered_waypoints_;
    std::deque<Point3d> waypoints_visualization_;
    bool is_publish_waypoints_home_ = false;
    bool is_publish_waypoints_inversed_ = false;
};
