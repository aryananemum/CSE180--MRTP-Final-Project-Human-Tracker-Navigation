#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Point2D {
  double x;
  double y;
};

struct HumanInfo {
  Point2D original; // original (x, y) in map frame
  bool moved;       // did this human move away from original?
  bool has_new_pos; // have we assigned a new estimated position?
  Point2D new_pos;  // estimated new (x, y)
};

class HumanTrackerNode : public rclcpp::Node {
public:
  HumanTrackerNode()
      : Node("human_tracker"), have_map_(false), have_pose_(false) {
    RCLCPP_INFO(this->get_logger(), "HumanTrackerNode started.");

    // Action Client for Navigation
    nav_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

    // Start mission thread
    mission_thread_ = std::thread(&HumanTrackerNode::missionLoop, this);

    // TODO: replace these with the actual (x, y) of your two humans
    // Read them once from RViz / Gazebo and update here before final
    // submission.
    // Updated coordinates from user
    HumanInfo h1;
    h1.original = {1.0, -1.0};
    h1.moved = false;
    h1.has_new_pos = false;
    h1.new_pos = {0.0, 0.0};

    HumanInfo h2;
    h2.original = {-12.5, 15.0};
    h2.moved = false;
    h2.has_new_pos = false;
    h2.new_pos = {0.0, 0.0};

    humans_.push_back(h1);
    humans_.push_back(h2);

    // Initial Pose Publisher
    init_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

    // Publish initial pose once after a short delay (in a separate thread or
    // timer, but here is fine for startup) We will do it in the mission thread
    // to ensure publisher is ready

    // Subscribers
    // Use Transient Local for map to ensure we get the latching message
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, std::bind(&HumanTrackerNode::mapCallback, this, _1));

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&HumanTrackerNode::poseCallback, this, _1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&HumanTrackerNode::scanCallback, this, _1));

    // Timer: periodically analyze differences (e.g., 1 Hz)
    timer_ = this->create_wall_timer(
        1s, std::bind(&HumanTrackerNode::analyzeEnvironment, this));
  }

private:
  // === Callbacks ===

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = *msg;
    have_map_ = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Received /map (size: %u x %u)", map_.info.width,
                         map_.info.height);
  }

  void poseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = msg->pose.pose;
    have_pose_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!have_map_ || !have_pose_) {
      // Need both map and pose to interpret scan in map frame
      return;
    }

    current_hits_.clear();
    const double angle_min = msg->angle_min;
    const double angle_inc = msg->angle_increment;

    // Robot pose in map frame
    double x_r = current_pose_.position.x;
    double y_r = current_pose_.position.y;

    // Yaw from quaternion
    double qx = current_pose_.orientation.x;
    double qy = current_pose_.orientation.y;
    double qz = current_pose_.orientation.z;
    double qw = current_pose_.orientation.w;
    double yaw =
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < msg->range_min || r > msg->range_max) {
        continue;
      }

      double angle = angle_min + i * angle_inc;

      // 1. Convert (r, theta) to point relative to robot (Local Frame)
      //    (As discussed in class discord: Manual Calculation approach)
      double local_x = r * std::cos(angle);
      double local_y = r * std::sin(angle);

      // 2. Transform from Robot Frame to Global Map Frame
      //    Rotate by yaw, translate by robot position (x_r, y_r)
      double x_hit = x_r + (local_x * std::cos(yaw) - local_y * std::sin(yaw));
      double y_hit = y_r + (local_x * std::sin(yaw) + local_y * std::cos(yaw));

      // Only keep hits that fall inside the known map bounds
      if (pointInMap(x_hit, y_hit)) {
        current_hits_.push_back({x_hit, y_hit});
      }
    }
  }

  // === Analysis ===

  void analyzeEnvironment() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!have_map_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for /map...");
      return;
    }
    if (!have_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for /amcl_pose...");
      return;
    }

    if (current_hits_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No current laser hits recorded yet.");
      return;
    }

    // --- Cluster "new" obstacles (places where map says free) ---

    // --- cluster "new" obstacles (places where map says free) ---
    // 1. Filter out static map points
    std::vector<Point2D> dynamic_points;
    for (const auto &p : current_hits_) {
      int mi, mj;
      if (worldToMap(p.x, p.y, mi, mj)) {
        int8_t occ = getMapValue(mi, mj);
        if (occ > 50) {
          continue; // Wall/Static Obstacle
        }
      }
      dynamic_points.push_back(p);
    }

    // 2. Cluster points (Euclidean Distance)
    const double CLUSTER_TOLERANCE = 0.3; // 0.3m tolerance
    std::vector<std::vector<Point2D>> clusters;
    if (!dynamic_points.empty()) {
      std::vector<Point2D> current_cluster;
      current_cluster.push_back(dynamic_points[0]);

      for (size_t i = 1; i < dynamic_points.size(); ++i) {
        const auto &p = dynamic_points[i];
        const auto &prev = dynamic_points[i - 1];
        double dist =
            std::sqrt(std::pow(p.x - prev.x, 2) + std::pow(p.y - prev.y, 2));

        if (dist < CLUSTER_TOLERANCE) {
          current_cluster.push_back(p);
        } else {
          clusters.push_back(current_cluster);
          current_cluster.clear();
          current_cluster.push_back(p);
        }
      }
      clusters.push_back(current_cluster);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Found %zu dynamic clusters.", clusters.size());

    // --- Check if each human is still at original position ---

    for (size_t h = 0; h < humans_.size(); ++h) {
      auto &human = humans_[h];

      // --- DISTANCE CHECK ---
      // We must be close to the human (e.g. 4.5m) before we even TRY to detect
      // them. If we are too far, we can't see them anyway, so don't update
      // state. INCREASED to 20.0 (approx 4.5m) to ensure we trigger even if
      // path planner stops early
      double dist_to_robot_sq =
          (current_pose_.position.x - human.original.x) *
              (current_pose_.position.x - human.original.x) +
          (current_pose_.position.y - human.original.y) *
              (current_pose_.position.y - human.original.y);

      if (dist_to_robot_sq > 20.0) {
        continue; // Too far, skip
      }

      // (Clustering now done at top level, 'clusters' variable is available
      // here)

      // 3. Analyze Clusters for Human Presence
      bool human_detected_in_cluster = false;

      for (const auto &cluster : clusters) {
        if (cluster.size() < 3)
          continue; // Noise

        // Centroid
        double cx = 0, cy = 0;
        double min_x = 1e9, max_x = -1e9;
        double min_y = 1e9, max_y = -1e9;

        for (const auto &p : cluster) {
          cx += p.x;
          cy += p.y;
          if (p.x < min_x)
            min_x = p.x;
          if (p.x > max_x)
            max_x = p.x;
          if (p.y < min_y)
            min_y = p.y;
          if (p.y > max_y)
            max_y = p.y;
        }
        cx /= cluster.size();
        cy /= cluster.size();

        // Width Check
        double width =
            std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_y - min_y, 2));

        // Distance to Expected Human Location
        double dist_to_human = std::sqrt(std::pow(cx - human.original.x, 2) +
                                         std::pow(cy - human.original.y, 2));

        // LOG EACH CLUSTER if it is somewhat close
        if (dist_to_human < 3.0) {
          RCLCPP_INFO(this->get_logger(),
                      "DEBUG H%zu Cluster: (%.2f, %.2f) Width=%.2f Dist=%.2f",
                      h + 1, cx, cy, width, dist_to_human);
        }

        // STRICTER CHECKS:
        // 1. Radius reduced to 1.5m (was 2.5m)
        // 2. Width reduced to 0.9m (was 1.0m)
        if (dist_to_human < 1.5 && width < 0.9) {
          human_detected_in_cluster = true;
          RCLCPP_INFO(this->get_logger(),
                      "Valid Human Cluster Found: Human %zu", h + 1);
        }
      }

      bool was_moved_before = human.moved;
      human.moved = !human_detected_in_cluster; // If NOT found, they Moved.

      if (!human_detected_in_cluster) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Human %zu NOT found near original position -> MOVED.", h + 1);
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Human %zu FOUND at position.", h + 1);
      }

      if (!human.moved) {
        human.has_new_pos =
            false; // still at original; discard any guessed new pos
      }

      if (human.moved && !was_moved_before) {
        RCLCPP_INFO(this->get_logger(),
                    "Human %zu appears to have moved away from original "
                    "position (%.2f, %.2f).",
                    h + 1, human.original.x, human.original.y);
      }

      if (!human.moved && was_moved_before) {
        RCLCPP_INFO(
            this->get_logger(),
            "Human %zu appears to be back at original position (%.2f, %.2f).",
            h + 1, human.original.x, human.original.y);
      }
    }

    // --- Assign new positions for humans who have moved ---
    // improved: Continuous Averaging & Width Check

    for (size_t h = 0; h < humans_.size(); ++h) {
      auto &human = humans_[h];
      if (!human.moved) {
        continue; // still at original, no need to assign
      }

      if (clusters.empty()) {
        continue;
      }

      // Find the BEST human candidate cluster
      // Criteria:
      // 1. Must be "Human Sized" (Width < 0.9)
      // 2. Closest to the original position (assuming they moved nearby)
      //    OR just closest to robot if we assume they are the only thing there.
      //    Let's stick to "Closest to Original" as a heuristic, but ONLY valid
      //    ones.

      double best_d2 = std::numeric_limits<double>::infinity();
      Point2D best_center{0.0, 0.0};
      bool found_candidate = false;

      for (const auto &cluster : clusters) {
        if (cluster.size() < 3)
          continue;

        // Calculate Cluster Properties again (Centroid & Width)
        double cx = 0, cy = 0, min_x = 1e9, max_x = -1e9, min_y = 1e9,
               max_y = -1e9;
        for (const auto &p : cluster) {
          cx += p.x;
          cy += p.y;
          if (p.x < min_x)
            min_x = p.x;
          if (p.x > max_x)
            max_x = p.x;
          if (p.y < min_y)
            min_y = p.y;
          if (p.y > max_y)
            max_y = p.y;
        }
        cx /= cluster.size();
        cy /= cluster.size();
        double width =
            std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_y - min_y, 2));

        // FILTER: Must be human-sized
        if (width > 0.9)
          continue;

        double d2 = std::pow(cx - human.original.x, 2) +
                    std::pow(cy - human.original.y, 2);

        // FILTER: Distance Tolerance.
        // If the cluster is too far from the original position (e.g. > 6.0m),
        // it is likely NOT this human, but someone else or clutter.
        if (d2 > 36.0)
          continue; // 6.0m squared

        if (d2 < best_d2) {
          best_d2 = d2;
          best_center = {cx, cy};
          found_candidate = true;
        }
      }

      if (found_candidate) {

        if (!human.has_new_pos) {
          // First detection: Initialize
          human.new_pos = best_center;
          human.has_new_pos = true;
        } else {
          // Refinement: Low-Pass Filter (Exponential Moving Average)
          // New = 0.9 * Old + 0.1 * Measurement
          // This smooths out noise over time.
          const double alpha = 0.2;
          human.new_pos.x =
              (1.0 - alpha) * human.new_pos.x + alpha * best_center.x;
          human.new_pos.y =
              (1.0 - alpha) * human.new_pos.y + alpha * best_center.y;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Refining Human %zu position... Current Est: (%.2f, %.2f).", h + 1,
            human.new_pos.x, human.new_pos.y);
      }
    }

    // --- Summary print (occasionally) ---

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 7000,
                         "=== Human status summary ===");
    for (size_t h = 0; h < humans_.size(); ++h) {
      const auto &human = humans_[h];
      if (!human.moved) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 7000,
                             "Human %zu: at original (%.2f, %.2f).", h + 1,
                             human.original.x, human.original.y);
      } else if (human.has_new_pos) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 7000,
                             "Human %zu: moved. New pos ~ (%.2f, %.2f).", h + 1,
                             human.new_pos.x, human.new_pos.y);
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 7000,
                             "Human %zu: moved away from (%.2f, %.2f), "
                             "searching for new position...",
                             h + 1, human.original.x, human.original.y);
      }
    }
  }

  // === Helper functions ===

  bool pointInMap(double x, double y) const {
    if (!have_map_)
      return false;
    const auto &info = map_.info;
    double mx = info.origin.position.x;
    double my = info.origin.position.y;
    double res = info.resolution;
    double max_x = mx + info.width * res;
    double max_y = my + info.height * res;
    return (x >= mx && x <= max_x && y >= my && y <= max_y);
  }

  bool worldToMap(double x, double y, int &mx, int &my) const {
    if (!have_map_)
      return false;
    const auto &info = map_.info;
    double origin_x = info.origin.position.x;
    double origin_y = info.origin.position.y;
    double res = info.resolution;

    if (!pointInMap(x, y))
      return false;

    mx = static_cast<int>((x - origin_x) / res);
    my = static_cast<int>((y - origin_y) / res);
    return true;
  }

  int8_t getMapValue(int mx, int my) const {
    if (!have_map_)
      return -1;
    const auto &info = map_.info;
    if (mx < 0 || my < 0 || static_cast<unsigned int>(mx) >= info.width ||
        static_cast<unsigned int>(my) >= info.height) {
      return -1;
    }
    size_t index = my * info.width + mx;
    return map_.data[index];
  }

  double distanceSquared(const Point2D &a, const Point2D &b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
  }

  // === Members ===

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      init_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::Pose current_pose_;
  bool have_map_;
  bool have_pose_;

  std::vector<Point2D> current_hits_;
  std::vector<HumanInfo> humans_;

  mutable std::mutex mutex_;

  // Navigation
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      nav_client_;
  std::thread mission_thread_;

  void missionLoop() {
    // Wait for action server
    RCLCPP_INFO(this->get_logger(),
                "Waiting for navigate_to_pose action server...");
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(20))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Navigation action server connected.");

    // Simple delay to let everything initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // --- Auto-Initialize Pose ---
    RCLCPP_INFO(this->get_logger(),
                "Setting initial pose to known start (2.12, -21.3, 1.57)...");
    auto p = geometry_msgs::msg::PoseWithCovarianceStamped();
    p.header.frame_id = "map";
    p.header.stamp = this->now();
    p.pose.pose.position.x = 2.12;
    p.pose.pose.position.y = -21.3;
    p.pose.pose.position.z = 0.0;

    tf2::Quaternion q_init;
    q_init.setRPY(0, 0, 1.57); // Yaw = 1.57 rad
    p.pose.pose.orientation = tf2::toMsg(q_init);

    // Set covariance (essential for AMCL to accept it)
    for (int i = 0; i < 36; i++)
      p.pose.covariance[i] = 0.0;
    p.pose.covariance[0] = 0.25;   // x
    p.pose.covariance[7] = 0.25;   // y
    p.pose.covariance[35] = 0.068; // yaw

    // Publish multiple times to be sure
    for (int i = 0; i < 5; i++) {
      init_pose_pub_->publish(p);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Wait for map and pose BEFORE moving
    RCLCPP_INFO(this->get_logger(),
                "Mission ready. Waiting for valid Map and Pose...");
    while (rclcpp::ok() && (!have_map_ || !have_pose_)) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(this->get_logger(),
                "Localization acquired. Starting Continuous Patrol.");

    // Define inspection points relative to humans
    // Assuming humans are static obstacles, we want to go NEAR them to see them
    // with lidar. Human 1: (2.0, -15.0) -> Try going to (3.0, -14.0) or (1.0,
    // -14.0)? Let's try 2.0 meters "north" of them (y + 2.0)

    // Patrol Loop
    while (rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "=== Starting Patrol Round ===");

      // 1. Go to Home/Start (2.12, -21.3)
      RCLCPP_INFO(this->get_logger(), "Moving to Patrol Start...");
      if (moveTo(2.12, -21.3, 1.57)) {
        RCLCPP_INFO(this->get_logger(), "Reached Start position.");
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));

      // 2. Inspect Human 1 at (1.0, -1.0)
      // Position robot slightly South-West or South of them
      // Target: (1.0, -3.0) facing North (approx 1.57 or 0)
      RCLCPP_INFO(this->get_logger(), "Inspecting Human 1 (1, -1)...");
      if (moveTo(1.0, -4.0, 1.57)) { // Back up a bit to (-4.0 y) to see better
                                     // without crashing
        RCLCPP_INFO(this->get_logger(), "reached Human 1 vantage point.");
      }
      std::this_thread::sleep_for(std::chrono::seconds(3));

      std::this_thread::sleep_for(std::chrono::seconds(2)); // Short pause

      // --- PATH ROUTING (User Specified) ---
      // Waypoints: (3,13) -> (12,13) -> (12,22) -> (-12,22)
      // Nav2 uses A* (or Dijkstra) by default between these points.

      RCLCPP_INFO(this->get_logger(), "Routing: Moving to WP1 (3.0, 13.0)...");
      moveTo(3.0, 13.0, 0.0);

      RCLCPP_INFO(this->get_logger(), "Routing: Moving to WP2 (12.0, 13.0)...");
      moveTo(12.0, 13.0, 1.57);

      RCLCPP_INFO(this->get_logger(), "Routing: Moving to WP3 (12.0, 22.0)...");
      moveTo(12.0, 22.0, 3.14);

      RCLCPP_INFO(this->get_logger(),
                  "Routing: Moving to WP4 (-12.0, 22.0)...");
      moveTo(-12.0, 22.0, 4.0); // Facing roughly South-West

      // 3. Inspect Human 2 at (-12.5, 15.0)
      // Now approach from the top/side
      RCLCPP_INFO(this->get_logger(), "Inspecting Human 2 (-12.5, 15)...");
      if (moveTo(-11.0, 14.0, 2.6)) {
        RCLCPP_INFO(this->get_logger(), "reached Human 2 vantage point.");
      }
      std::this_thread::sleep_for(std::chrono::seconds(3));

      // 4. Explore Left Aisle (Long loop)
      RCLCPP_INFO(this->get_logger(), "Exploring Left Center...");
      if (moveTo(-3.0, -5.0, 3.14)) {
        RCLCPP_INFO(this->get_logger(), "Reached Left Center.");
      }

      // 5. Explore Top Right
      RCLCPP_INFO(this->get_logger(), "Exploring Top Right...");
      if (moveTo(5.0, 5.0, 0.0)) {
        RCLCPP_INFO(this->get_logger(), "Reached Top Right.");
      }

      RCLCPP_INFO(this->get_logger(), "Round complete. Restarting...");
    }
  }

  bool moveTo(double x, double y, double yaw) {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "Sending goal: (%.2f, %.2f, %.2f)", x, y,
                yaw);

    auto send_goal_options = rclcpp_action::Client<
        nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&HumanTrackerNode::resultCallback, this, _1);

    auto future_goal_handle =
        nav_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for the goal to be accepted
    if (future_goal_handle.wait_for(std::chrono::seconds(5)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      return false;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return false;
    }

    // Wait for result
    // Note: this is blocking the mission thread, which is fine.
    auto result_future = nav_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) !=
        std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Navigation timed out.");
      return false; // or cancel?
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Navigation failed with result code: %d",
                  (int)wrapped_result.code);
      return false;
    }
  }

  void
  resultCallback(const rclcpp_action::ClientGoalHandle<
                 nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
  }

  // Destructor to join thread
public:
  ~HumanTrackerNode() {
    if (mission_thread_.joinable()) {
      mission_thread_.join();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
