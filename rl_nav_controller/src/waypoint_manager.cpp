#include "rl_nav_controller/waypoint_manager.hpp"

WaypointManager::WaypointManager(rclcpp::Logger logger)
    : logger_(logger) {}

bool WaypointManager::recordWaypoint(const Point3d& robot_pos_w) {
    if (recorded_waypoints_.empty() && !inversed_ordered_waypoints_.empty()) {
        inversed_ordered_waypoints_.clear();
        waypoints_visualization_.clear();
        RCLCPP_INFO(logger_, "Clear the inversed waypoints, and visualization waypoints.");
    }

    recorded_waypoints_.push_back(robot_pos_w);
    inversed_ordered_waypoints_.push_front(robot_pos_w);
    waypoints_visualization_.push_back(robot_pos_w);
    RCLCPP_INFO(logger_, "Waypoint recorded: [%.2f, %.2f, %.2f], total waypoints: %zu",
                robot_pos_w[0], robot_pos_w[1], robot_pos_w[2], recorded_waypoints_.size());
    return true;
}

bool WaypointManager::removeLastWaypoint() {
    int removed_count = 0;

    if (!recorded_waypoints_.empty()) {
        recorded_waypoints_.pop_back();
        removed_count++;
        RCLCPP_INFO(logger_, "Removed last waypoint (home), total: %zu", recorded_waypoints_.size());
    }

    if (!inversed_ordered_waypoints_.empty()) {
        inversed_ordered_waypoints_.pop_front();
        removed_count++;
        RCLCPP_INFO(logger_, "Removed last waypoint (inversed), total: %zu", inversed_ordered_waypoints_.size());
    }

    if (!waypoints_visualization_.empty()) {
        waypoints_visualization_.pop_back();
        removed_count++;
        RCLCPP_INFO(logger_, "Removed last waypoint (visualization), total: %zu", waypoints_visualization_.size());
    }

    if (removed_count == 0) {
        RCLCPP_WARN(logger_, "No waypoints to remove.");
    }

    return removed_count > 0;
}

WaypointManager::Point3d WaypointManager::getNextWaypointHome() {
    Point3d wp = recorded_waypoints_.back();
    recorded_waypoints_.pop_back();
    return wp;
}

WaypointManager::Point3d WaypointManager::getNextWaypointInversed() {
    Point3d wp = inversed_ordered_waypoints_.back();
    inversed_ordered_waypoints_.pop_back();
    return wp;
}

bool WaypointManager::startHomeWaypointSequence() {
    if (!recorded_waypoints_.empty()) {
        is_publish_waypoints_home_ = true;
        return true;
    }
    return false;
}

bool WaypointManager::startInversedWaypointSequence() {
    if (!inversed_ordered_waypoints_.empty()) {
        is_publish_waypoints_inversed_ = true;
        return true;
    }
    return false;
}

void WaypointManager::stopHomeWaypointSequence() {
    is_publish_waypoints_home_ = false;
}

void WaypointManager::stopInversedWaypointSequence() {
    is_publish_waypoints_inversed_ = false;
}

bool WaypointManager::resetVisualizationIfComplete() {
    if (recorded_waypoints_.empty() && inversed_ordered_waypoints_.empty() && !waypoints_visualization_.empty()) {
        RCLCPP_INFO(logger_, "All waypoints are published, reset visualization.");
        waypoints_visualization_.clear();
        return true;
    }
    return false;
}

void WaypointManager::reAddAbortedWaypoint(const Point3d& target_pos_w) {
    if (is_publish_waypoints_home_) {
        recorded_waypoints_.push_back(target_pos_w);
        RCLCPP_INFO(logger_, "Goal aborted - re-adding current goal as a recorded (home) waypoint.");
    }
    if (is_publish_waypoints_inversed_) {
        inversed_ordered_waypoints_.push_front(target_pos_w);
        RCLCPP_INFO(logger_, "Goal aborted: reinserting the current goal as recorded (inversed) waypoint.");
    }
}
