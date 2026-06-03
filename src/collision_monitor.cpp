#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <mutex>

class SafeVelocityControllerNode : public rclcpp::Node {
public:
    SafeVelocityControllerNode() : Node("safe_velocity_controller") {
        // /scan トピックを購読
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SafeVelocityControllerNode::scan_callback, this, std::placeholders::_1));
        
        // /cmd_vel トピックを購読
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SafeVelocityControllerNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // /cmd_vel_safe トピックを配信
        safe_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/collision_monitor/cmd_vel", 10);
        
        // 介入状態を配信
        intervention_publisher_ = this->create_publisher<std_msgs::msg::String>("/intervention_status", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "Safe Velocity Controller Node started.\n"
            "Robot Radius: %.2f m\n"
            "Collision Threshold: %.2f m\n"
            "Subscribing to: /scan, /cmd_vel\n"
            "Publishing to: /cmd_vel_safe, /intervention_status",
            ROBOT_RADIUS, COLLISION_THRESHOLD);
    }

private:
    struct DirectionCollisionStatus {
        bool front_collision = false;
        bool right_collision = false;
        bool back_collision = false;
        bool left_collision = false;
        
        float front_min_dist = std::numeric_limits<float>::max();
        float right_min_dist = std::numeric_limits<float>::max();
        float back_min_dist = std::numeric_limits<float>::max();
        float left_min_dist = std::numeric_limits<float>::max();
    };

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intervention_publisher_;
    
    DirectionCollisionStatus collision_status_;
    std::mutex collision_mutex_;
    
    // パラメータ
    static constexpr float ROBOT_RADIUS = 0.25f;
    static constexpr float COLLISION_THRESHOLD = 0.4f;
    static constexpr float WARNING_THRESHOLD = 0.5f;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(collision_mutex_);
        
        // 衝突状態をリセット
        collision_status_.front_collision = false;
        collision_status_.right_collision = false;
        collision_status_.back_collision = false;
        collision_status_.left_collision = false;
        
        collision_status_.front_min_dist = std::numeric_limits<float>::max();
        collision_status_.right_min_dist = std::numeric_limits<float>::max();
        collision_status_.back_min_dist = std::numeric_limits<float>::max();
        collision_status_.left_min_dist = std::numeric_limits<float>::max();

        // スキャンデータを分析
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            if (range < msg->range_min || range > msg->range_max) {
                continue;
            }

            // 各方向に分類
            if (is_in_angle_range(angle, -M_PI/4.0, M_PI/4.0)) {  // FRONT
                collision_status_.front_min_dist = std::min(collision_status_.front_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.front_collision = true;
                    RCLCPP_INFO(this->get_logger(), "front_collision:TRUE");
                }
            }
            else if (is_in_angle_range(angle, -3*M_PI/4.0, -M_PI/4.0)) {  // RIGHT
                collision_status_.right_min_dist = std::min(collision_status_.right_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.right_collision = true;
                    RCLCPP_INFO(this->get_logger(), "right_collision:TRUE");
                }
            }
            else if (is_in_angle_range(angle, 3*M_PI/4.0, -3*M_PI/4.0)) {  // BACK
                collision_status_.back_min_dist = std::min(collision_status_.back_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.back_collision = true;
                    RCLCPP_INFO(this->get_logger(), "back_collision:TRUE");
                }
            }
            else if (is_in_angle_range(angle, M_PI/4.0, 3*M_PI/4.0)) {  // LEFT
                collision_status_.left_min_dist = std::min(collision_status_.left_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.left_collision = true;
                    RCLCPP_INFO(this->get_logger(), "left_collision:TRUE");
                }
            }
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(collision_mutex_);
        
        auto safe_cmd = std::make_shared<geometry_msgs::msg::Twist>(*msg);
        
        std::stringstream intervention_log;
        intervention_log << std::fixed << std::setprecision(3);
        
        bool intervention_made = false;
        std::vector<std::string> blocked_directions;

        // 前進が衝突方向に該当するか確認
        if (collision_status_.front_collision && msg->linear.x > 0.0) {
            RCLCPP_WARN(this->get_logger(), 
                "[INTERVENTION] FRONT COLLISION - Blocking forward movement (%.3f m/s)",
                msg->linear.x);
            safe_cmd->linear.x = 0.0;
            intervention_made = true;
            blocked_directions.push_back("FRONT (forward)");
        }

        // 後進が衝突方向に該当するか確認
        if (collision_status_.back_collision && msg->linear.x < 0.0) {
            RCLCPP_WARN(this->get_logger(), 
                "[INTERVENTION] BACK COLLISION - Blocking backward movement (%.3f m/s)",
                msg->linear.x);
            safe_cmd->linear.x = 0.0;
            intervention_made = true;
            blocked_directions.push_back("BACK (backward)");
        }

        // 左方向への移動が衝突方向に該当するか確認
        if (collision_status_.left_collision && msg->linear.y > 0.0) {
            RCLCPP_WARN(this->get_logger(), 
                "[INTERVENTION] LEFT COLLISION - Blocking left movement (%.3f m/s)",
                msg->linear.y);
            safe_cmd->linear.y = 0.0;
            intervention_made = true;
            blocked_directions.push_back("LEFT (left)");
        }

        // 右方向への移動が衝突方向に該当するか確認
        if (collision_status_.right_collision && msg->linear.y < 0.0) {
            RCLCPP_WARN(this->get_logger(), 
                "[INTERVENTION] RIGHT COLLISION - Blocking right movement (%.3f m/s)",
                msg->linear.y);
            safe_cmd->linear.y = 0.0;
            intervention_made = true;
            blocked_directions.push_back("RIGHT (right)");
        }

        // 安全な速度コマンドを配信
        safe_velocity_publisher_->publish(*safe_cmd);

        // 介入ログを生成
        if (intervention_made) {
            intervention_log << "[INTERVENTION]\n";
            intervention_log << "Blocked Directions: ";
            for (size_t i = 0; i < blocked_directions.size(); i++) {
                intervention_log << blocked_directions[i];
                if (i < blocked_directions.size() - 1) intervention_log << ", ";
            }
            intervention_log << "\n";
            intervention_log << "Original cmd_vel: "
                << "linear.x=" << msg->linear.x
                << ", linear.y=" << msg->linear.y
                << ", angular.z=" << msg->angular.z << "\n";
            intervention_log << "Modified cmd_vel: "
                << "linear.x=" << safe_cmd->linear.x
                << ", linear.y=" << safe_cmd->linear.y
                << ", angular.z=" << safe_cmd->angular.z << "\n";
            intervention_log << "Collision Status:\n"
                << "  FRONT: " << (collision_status_.front_collision ? "COLLISION" : "SAFE")
                << " (" << collision_status_.front_min_dist << " m)\n"
                << "  RIGHT: " << (collision_status_.right_collision ? "COLLISION" : "SAFE")
                << " (" << collision_status_.right_min_dist << " m)\n"
                << "  BACK: " << (collision_status_.back_collision ? "COLLISION" : "SAFE")
                << " (" << collision_status_.back_min_dist << " m)\n"
                << "  LEFT: " << (collision_status_.left_collision ? "COLLISION" : "SAFE")
                << " (" << collision_status_.left_min_dist << " m)";
        } else {
            intervention_log << "[PASS] Command allowed - Safe movement in all enabled directions\n"
                << "cmd_vel: linear.x=" << msg->linear.x
                << ", linear.y=" << msg->linear.y
                << ", angular.z=" << msg->angular.z;
        }

        // 介入状態を配信
        auto status_msg = std_msgs::msg::String();
        status_msg.data = intervention_log.str();
        intervention_publisher_->publish(status_msg);
    }

    bool is_in_angle_range(double angle, double min_angle, double max_angle) {
        // BACK方向（±π をまたぐ）の特殊処理
        if (min_angle > max_angle) {
            return (angle >= min_angle || angle <= max_angle);
        }
        return (angle >= min_angle && angle <= max_angle);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeVelocityControllerNode>());
    rclcpp::shutdown();
    return 0;
}