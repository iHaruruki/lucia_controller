#include "lucia_controller/collision_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

CollisionMonitor::CollisionMonitor()
    : Node("collision_monitor"),
      collision_threshold_(0.5f),
      front_angle_range_(60.0f),
      back_angle_range_(60.0f),
      side_angle_range_(60.0f),
      enable_emergency_stop_(true),
      publish_min_distance_(true),
      enable_visualization_(true),
      linear_velocity_threshold_(0.01f),
      frame_id_("base_link")
{
    // Initialize collision zones
    collision_zones_[0].direction = Direction::FRONT;
    collision_zones_[1].direction = Direction::BACK;
    collision_zones_[2].direction = Direction::LEFT;
    collision_zones_[3].direction = Direction::RIGHT;

    // Declare parameters
    this->declare_parameter<float>("collision_threshold", 0.5f);
    this->declare_parameter<float>("front_angle_range", 60.0f);
    this->declare_parameter<float>("back_angle_range", 60.0f);
    this->declare_parameter<float>("side_angle_range", 60.0f);
    this->declare_parameter<bool>("enable_emergency_stop", true);
    this->declare_parameter<bool>("publish_min_distance", true);
    this->declare_parameter<bool>("enable_visualization", true);
    this->declare_parameter<float>("linear_velocity_threshold", 0.01f);
    this->declare_parameter<bool>("log_detailed", false);
    this->declare_parameter<std::string>("frame_id", "base_link");

    // Get parameters
    collision_threshold_ = this->get_parameter("collision_threshold").as_double();
    front_angle_range_ = this->get_parameter("front_angle_range").as_double();
    back_angle_range_ = this->get_parameter("back_angle_range").as_double();
    side_angle_range_ = this->get_parameter("side_angle_range").as_double();
    enable_emergency_stop_ = this->get_parameter("enable_emergency_stop").as_bool();
    publish_min_distance_ = this->get_parameter("publish_min_distance").as_bool();
    enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
    linear_velocity_threshold_ = this->get_parameter("linear_velocity_threshold").as_double();
    log_detailed_ = this->get_parameter("log_detailed").as_bool();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Update collision thresholds
    for (auto& zone : collision_zones_) {
        zone.collision_threshold = collision_threshold_;
    }

    // Create subscribers
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&CollisionMonitor::scan_callback, this, std::placeholders::_1));

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/twist_mux/cmd_vel",
        10,
        std::bind(&CollisionMonitor::cmd_vel_callback, this, std::placeholders::_1));

    // Create publishers
    collision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/lucia_controller/collision_detected",
        10);

    emergency_stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/lucia_controller/emergency_stop",
        10);

    if (publish_min_distance_) {
        min_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/lucia_controller/min_distance",
            10);
    }

    if (enable_visualization_) {
        visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/lucia_controller/collision_zones",
            10);
    }

    RCLCPP_INFO(this->get_logger(), "CollisionMonitor node initialized");
    RCLCPP_INFO(this->get_logger(), "Collision threshold: %.2f m", collision_threshold_);
    RCLCPP_INFO(this->get_logger(), "Front angle range: %.1f°", front_angle_range_);
    RCLCPP_INFO(this->get_logger(), "Back angle range: %.1f°", back_angle_range_);
    RCLCPP_INFO(this->get_logger(), "Side angle range: %.1f°", side_angle_range_);
    RCLCPP_INFO(this->get_logger(), "Emergency stop enabled: %s", enable_emergency_stop_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "RViz2 visualization enabled: %s", enable_visualization_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Linear velocity threshold: %.3f m/s", linear_velocity_threshold_);
}

void CollisionMonitor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Reset zones
    for (auto& zone : collision_zones_) {
        zone.min_distance = std::numeric_limits<float>::infinity();
        zone.max_distance = 0.0f;
        zone.scan_count = 0;
        zone.is_collision = false;
    }

    global_min_distance_ = std::numeric_limits<float>::infinity();

    // Analyze the scan
    analyze_scan(msg);

    // Check for collisions
    check_collisions();

    // Publish collision status
    publish_collision_status();

    // Publish visualization markers for RViz2
    if (enable_visualization_) {
        publish_visualization_markers();
    }

    frame_count_++;
}

void CollisionMonitor::analyze_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    const auto& ranges = scan_msg->ranges;
    size_t num_ranges = ranges.size();
    float angle_increment = scan_msg->angle_increment;

    if (num_ranges == 0) {
        RCLCPP_WARN(this->get_logger(), "Empty laser scan received");
        return;
    }

    // Calculate angle indices for each direction
    // Assuming laser scan is centered at 0 degrees (front)
    float front_idx_start = static_cast<float>(num_ranges) * (90.0f - front_angle_range_ / 2.0f) / 180.0f;
    float front_idx_end = static_cast<float>(num_ranges) * (90.0f + front_angle_range_ / 2.0f) / 180.0f;

    float back_idx_start = static_cast<float>(num_ranges) * (270.0f - back_angle_range_ / 2.0f) / 180.0f;
    float back_idx_end = static_cast<float>(num_ranges) * (270.0f + back_angle_range_ / 2.0f) / 180.0f;

    float left_idx_start = static_cast<float>(num_ranges) * (180.0f - side_angle_range_ / 2.0f) / 180.0f;
    float left_idx_end = static_cast<float>(num_ranges) * (180.0f + side_angle_range_ / 2.0f) / 180.0f;

    float right_idx_start = static_cast<float>(num_ranges) * (0.0f - side_angle_range_ / 2.0f) / 180.0f;
    float right_idx_end = static_cast<float>(num_ranges) * (0.0f + side_angle_range_ / 2.0f) / 180.0f;

    // Process each direction
    process_collision_zone(scan_msg, Direction::FRONT, 
                         static_cast<size_t>(front_idx_start), 
                         static_cast<size_t>(front_idx_end));

    process_collision_zone(scan_msg, Direction::BACK, 
                         static_cast<size_t>(back_idx_start), 
                         static_cast<size_t>(back_idx_end));

    process_collision_zone(scan_msg, Direction::LEFT, 
                         static_cast<size_t>(left_idx_start), 
                         static_cast<size_t>(left_idx_end));

    process_collision_zone(scan_msg, Direction::RIGHT, 
                         static_cast<size_t>(right_idx_start), 
                         static_cast<size_t>(right_idx_end));
}

void CollisionMonitor::process_collision_zone(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
    Direction direction, 
    size_t start_idx, 
    size_t end_idx)
{
    const auto& ranges = scan_msg->ranges;
    size_t num_ranges = ranges.size();

    // Clamp indices
    start_idx = std::max(size_t(0), std::min(start_idx, num_ranges - 1));
    end_idx = std::max(size_t(0), std::min(end_idx, num_ranges - 1));

    CollisionZone& zone = collision_zones_[static_cast<size_t>(direction)];

    // Process ranges in this zone
    for (size_t i = start_idx; i <= end_idx && i < num_ranges; ++i) {
        float range = ranges[i];

        // Filter out invalid ranges
        if (std::isfinite(range) && range > scan_msg->range_min && range < scan_msg->range_max) {
            zone.min_distance = std::min(zone.min_distance, range);
            zone.max_distance = std::max(zone.max_distance, range);
            zone.scan_count++;
            global_min_distance_ = std::min(global_min_distance_, range);
        }
    }

    if (log_detailed_ && frame_count_ % 10 == 0) {
        const char* dir_str[] = {"FRONT", "BACK", "LEFT", "RIGHT"};
        RCLCPP_DEBUG(this->get_logger(), 
                    "%s - Min: %.3f m, Max: %.3f m, Count: %d",
                    dir_str[static_cast<size_t>(direction)],
                    zone.min_distance,
                    zone.max_distance,
                    zone.scan_count);
    }
}

void CollisionMonitor::check_collisions()
{
    bool any_collision = false;

    for (auto& zone : collision_zones_) {
        zone.is_collision = (zone.min_distance < zone.collision_threshold);
        any_collision = any_collision || zone.is_collision;
    }

    is_collision_ = any_collision;

    // Log collision events
    static bool last_collision_state = false;
    if (is_collision_ != last_collision_state) {
        if (is_collision_) {
            RCLCPP_WARN(this->get_logger(), "🚨 COLLISION DETECTED!");
        } else {
            RCLCPP_INFO(this->get_logger(), "✓ No collision detected");
        }
        last_collision_state = is_collision_;
    }
}

void CollisionMonitor::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_cmd_vel_ = *msg;
    has_recent_cmd_vel_ = true;
}

bool CollisionMonitor::is_moving_towards_collision(Direction collision_dir, double linear_x, double linear_y)
{
    // Check if the commanded linear velocity would move towards the collision zone
    switch (collision_dir) {
        case Direction::FRONT:
            return linear_x > linear_velocity_threshold_;
        case Direction::BACK:
            return linear_x < -linear_velocity_threshold_;
        case Direction::LEFT:
            return linear_y > linear_velocity_threshold_;
        case Direction::RIGHT:
            return linear_y < -linear_velocity_threshold_;
        default:
            return false;
    }
}

bool CollisionMonitor::should_allow_motion(double linear_x, double linear_y, double angular_z)
{
    // 回転動作（angular_z）のみの場合は、衝突検出中でも許可
    if (std::abs(linear_x) <= linear_velocity_threshold_ && 
        std::abs(linear_y) <= linear_velocity_threshold_ &&
        std::abs(angular_z) > linear_velocity_threshold_) {
        RCLCPP_DEBUG(this->get_logger(), "Rotation allowed during collision");
        return true;
    }

    // 衝突していない場合は、すべての動作を許可
    if (!is_collision_) {
        return true;
    }

    // 衝突している場合、衝突方向への移動をブロック
    for (const auto& zone : collision_zones_) {
        if (zone.is_collision && is_moving_towards_collision(zone.direction, linear_x, linear_y)) {
            RCLCPP_WARN(this->get_logger(), "Motion blocked towards collision zone");
            return false;
        }
    }

    return true;
}

void CollisionMonitor::set_marker_color(visualization_msgs::msg::Marker& marker, 
                                       float r, float g, float b, float a)
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

void CollisionMonitor::publish_visualization_markers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    auto now = this->get_clock()->now();

    const char* dir_str[] = {"FRONT", "BACK", "LEFT", "RIGHT"};
    const float robot_radius = 0.15f;  // ロボット半径（メートル）

    for (size_t i = 0; i < collision_zones_.size(); ++i) {
        const auto& zone = collision_zones_[i];
        if (zone.scan_count == 0) continue;

        // Create collision zone cylinder marker (円柱形に合わせた側面)
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = now;
        marker.ns = "collision_zones";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // tf2::Quaternion をスイッチ文の外で宣言
        tf2::Quaternion q;
        float distance_offset = zone.min_distance / 2.0f;

        // 各方向に円柱を配置
        switch (zone.direction) {
            case Direction::FRONT:
                // 前方：X軸正方向
                marker.pose.position.x = robot_radius + distance_offset;
                marker.pose.position.y = 0.0f;
                marker.pose.position.z = 0.0f;
                q.setRPY(0.0, M_PI / 2.0, 0.0);
                marker.pose.orientation = tf2::toMsg(q);
                break;

            case Direction::BACK:
                // 背後：X軸負方向
                marker.pose.position.x = -(robot_radius + distance_offset);
                marker.pose.position.y = 0.0f;
                marker.pose.position.z = 0.0f;
                q.setRPY(0.0, M_PI / 2.0, 0.0);
                marker.pose.orientation = tf2::toMsg(q);
                break;

            case Direction::LEFT:
                // 左側：Y軸正方向
                marker.pose.position.x = 0.0f;
                marker.pose.position.y = robot_radius + distance_offset;
                marker.pose.position.z = 0.0f;
                q.setRPY(M_PI / 2.0, 0.0, 0.0);
                marker.pose.orientation = tf2::toMsg(q);
                break;

            case Direction::RIGHT:
                // 右側：Y軸負方向
                marker.pose.position.x = 0.0f;
                marker.pose.position.y = -(robot_radius + distance_offset);
                marker.pose.position.z = 0.0f;
                q.setRPY(M_PI / 2.0, 0.0, 0.0);
                marker.pose.orientation = tf2::toMsg(q);
                break;
        }

        // スケール設定：円柱形状
        marker.scale.x = 0.3f;   // 直径
        marker.scale.y = 0.3f;   // 直径
        marker.scale.z = 0.5f;   // 高さ

        // 色設定：衝突状態に応じて変更
        if (zone.is_collision) {
            set_marker_color(marker, 1.0f, 0.0f, 0.0f, 0.8f);  // 赤：衝突
        } else {
            set_marker_color(marker, 0.0f, 1.0f, 0.0f, 0.5f);  // 緑：安全
        }

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);

        // Create text label marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id_;
        text_marker.header.stamp = now;
        text_marker.ns = "collision_labels";
        text_marker.id = i + 10;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // テキスト位置
        switch (zone.direction) {
            case Direction::FRONT:
                text_marker.pose.position.x = robot_radius + distance_offset;
                text_marker.pose.position.y = 0.0f;
                break;
            case Direction::BACK:
                text_marker.pose.position.x = -(robot_radius + distance_offset);
                text_marker.pose.position.y = 0.0f;
                break;
            case Direction::LEFT:
                text_marker.pose.position.x = 0.0f;
                text_marker.pose.position.y = robot_radius + distance_offset;
                break;
            case Direction::RIGHT:
                text_marker.pose.position.x = 0.0f;
                text_marker.pose.position.y = -(robot_radius + distance_offset);
                break;
        }
        text_marker.pose.position.z = 0.25f;
        text_marker.pose.orientation.w = 1.0f;

        text_marker.scale.z = 0.1f;  // テキスト高さ
        text_marker.text = std::string(dir_str[i]) + "\n" + 
                          std::to_string(static_cast<int>(zone.min_distance * 100)) + "cm";
        
        set_marker_color(text_marker, 1.0f, 1.0f, 1.0f, 1.0f);
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(text_marker);

        // Create collision threshold ring marker
        visualization_msgs::msg::Marker ring_marker;
        ring_marker.header.frame_id = frame_id_;
        ring_marker.header.stamp = now;
        ring_marker.ns = "threshold_rings";
        ring_marker.id = i + 20;
        ring_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        ring_marker.action = visualization_msgs::msg::Marker::ADD;

        // 閾値位置
        switch (zone.direction) {
            case Direction::FRONT:
                ring_marker.pose.position.x = robot_radius + collision_threshold_ / 2.0f;
                ring_marker.pose.position.y = 0.0f;
                q.setRPY(0.0, M_PI / 2.0, 0.0);
                break;
            case Direction::BACK:
                ring_marker.pose.position.x = -(robot_radius + collision_threshold_ / 2.0f);
                ring_marker.pose.position.y = 0.0f;
                q.setRPY(0.0, M_PI / 2.0, 0.0);
                break;
            case Direction::LEFT:
                ring_marker.pose.position.x = 0.0f;
                ring_marker.pose.position.y = robot_radius + collision_threshold_ / 2.0f;
                q.setRPY(M_PI / 2.0, 0.0, 0.0);
                break;
            case Direction::RIGHT:
                ring_marker.pose.position.x = 0.0f;
                ring_marker.pose.position.y = -(robot_radius + collision_threshold_ / 2.0f);
                q.setRPY(M_PI / 2.0, 0.0, 0.0);
                break;
        }
        ring_marker.pose.position.z = 0.0f;
        ring_marker.pose.orientation = tf2::toMsg(q);

        ring_marker.scale.x = 0.05f;
        ring_marker.scale.y = 0.05f;
        ring_marker.scale.z = 0.01f;

        set_marker_color(ring_marker, 1.0f, 0.0f, 0.0f, 0.3f);
        ring_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(ring_marker);
    }

    visualization_pub_->publish(marker_array);
}

void CollisionMonitor::publish_collision_status()
{
    // Publish collision detected status
    auto collision_msg = std_msgs::msg::Bool();
    collision_msg.data = is_collision_;
    collision_pub_->publish(collision_msg);

    // Publish minimum distance if enabled
    if (publish_min_distance_ && global_min_distance_ != std::numeric_limits<float>::infinity()) {
        auto min_dist_msg = std_msgs::msg::Float32();
        min_dist_msg.data = global_min_distance_;
        min_distance_pub_->publish(min_dist_msg);
    }

    // Publish emergency stop if collision detected and moving towards obstacle
    if (enable_emergency_stop_ && is_collision_) {
        double linear_x = current_cmd_vel_.linear.x;
        double linear_y = current_cmd_vel_.linear.y;
        double angular_z = current_cmd_vel_.angular.z;

        // Check if rotation is allowed
        if (!should_allow_motion(linear_x, linear_y, angular_z)) {
            for (const auto& zone : collision_zones_) {
                if (zone.is_collision && is_moving_towards_collision(zone.direction, linear_x, linear_y)) {
                    auto stop_msg = geometry_msgs::msg::Twist();
                    emergency_stop_pub_->publish(stop_msg);
                    RCLCPP_WARN(this->get_logger(), "🛑 Emergency stop triggered!");
                    break;
                }
            }
        }
    }

    // Log status periodically
    if (frame_count_ % 30 == 0) {
        const char* dir_str[] = {"FRONT", "BACK", "LEFT", "RIGHT"};
        RCLCPP_INFO(this->get_logger(), "=== Collision Monitor Status (Frame %d) ===", frame_count_);
        for (size_t i = 0; i < collision_zones_.size(); ++i) {
            if (collision_zones_[i].scan_count > 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "%s: %.3f m %s",
                           dir_str[i],
                           collision_zones_[i].min_distance,
                           collision_zones_[i].is_collision ? "⚠️  [COLLISION]" : "");
            }
        }
        RCLCPP_INFO(this->get_logger(), "Global min distance: %.3f m", global_min_distance_);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}