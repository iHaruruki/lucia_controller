#ifndef LUCIA_CONTROLLER__COLLISION_MONITOR_HPP_
#define LUCIA_CONTROLLER__COLLISION_MONITOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <array>
#include <cmath>
#include <mutex>

// Direction enumeration for collision zones
enum class Direction {
  FRONT = 0,
  BACK = 1,
  LEFT = 2,
  RIGHT = 3
};

// Structure to hold collision data for each direction
struct CollisionZone {
  Direction direction;
  float min_distance = std::numeric_limits<float>::infinity();
  float max_distance = 0.0f;
  int scan_count = 0;
  bool is_collision = false;
  float collision_threshold = 0.5f;  // meters
};

class CollisionMonitor : public rclcpp::Node {
public:
  CollisionMonitor();
  ~CollisionMonitor() = default;

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Helper methods
  void analyze_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void check_collisions();
  void process_collision_zone(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
                             Direction direction, size_t start_idx, size_t end_idx);
  void publish_collision_status();
  void publish_visualization_markers();
  bool should_allow_motion(double linear_x, double linear_y, double angular_z);
  bool is_moving_towards_collision(Direction collision_dir, double linear_x, double linear_y);

  // Collision zones (Front, Back, Left, Right)
  std::array<CollisionZone, 4> collision_zones_;

  // Mutex for thread safety
  std::mutex data_mutex_;

  // Configuration parameters
  float collision_threshold_;
  float front_angle_range_;  // degrees
  float back_angle_range_;   // degrees
  float side_angle_range_;   // degrees
  bool enable_emergency_stop_;
  bool publish_min_distance_;
  bool enable_visualization_;
  float linear_velocity_threshold_;  // m/s threshold for linear motion detection

  // Current velocity command
  geometry_msgs::msg::Twist current_cmd_vel_;
  bool has_recent_cmd_vel_ = false;

  // Collision state
  bool is_collision_ = false;
  float global_min_distance_ = std::numeric_limits<float>::infinity();
  std::string frame_id_;

  // Logging
  int frame_count_ = 0;
  bool log_detailed_ = false;

  // Color helpers for visualization
  void set_marker_color(visualization_msgs::msg::Marker& marker, float r, float g, float b, float a);
};

#endif  // LUCIA_CONTROLLER__COLLISION_MONITOR_HPP_
