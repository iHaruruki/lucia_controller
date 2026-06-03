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
            "Publishing to: /collision_monitor/cmd_vel, /intervention_status",
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
    
    bool cmd_vel_received_ = false;
    
    // パラメータ
    static constexpr float ROBOT_RADIUS = 0.25f;
    static constexpr float COLLISION_THRESHOLD = 0.7f;
    static constexpr float WARNING_THRESHOLD = 0.8f;

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

        RCLCPP_DEBUG(this->get_logger(),
            "Scan received: ranges=%zu, angle_min=%.4f (%.1f°), angle_max=%.4f (%.1f°), angle_increment=%.4f",
            msg->ranges.size(), msg->angle_min, msg->angle_min * 180 / M_PI, 
            msg->angle_max, msg->angle_max * 180 / M_PI, msg->angle_increment);

        // スキャンデータを分析
        int valid_count = 0;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            if (range < msg->range_min || range > msg->range_max) {
                continue;
            }
            valid_count++;

            // 角度を正規化 (-π ~ π)
            double normalized_angle = normalize_angle(angle);
            
            // FRONT: -π/4 ~ π/4 (−45° ~ 45°)
            if (normalized_angle >= -M_PI/4.0 && normalized_angle <= M_PI/4.0) {
                collision_status_.front_min_dist = std::min(collision_status_.front_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.front_collision = true;
                    RCLCPP_WARN(this->get_logger(), 
                        "[FRONT COLLISION] angle=%.3f rad (%.1f°), distance=%.3f m",
                        normalized_angle, normalized_angle * 180 / M_PI, range);
                }
            }
            // LEFT: π/4 ~ 3π/4 (45° ~ 135°)
            else if (normalized_angle > M_PI/4.0 && normalized_angle <= 3*M_PI/4.0) {
                collision_status_.left_min_dist = std::min(collision_status_.left_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.left_collision = true;
                    RCLCPP_WARN(this->get_logger(), 
                        "[LEFT COLLISION] angle=%.3f rad (%.1f°), distance=%.3f m",
                        normalized_angle, normalized_angle * 180 / M_PI, range);
                }
            }
            // BACK: 3π/4 ~ -3π/4 (135° ~ -135°, wrapping around ±π)
            else if (normalized_angle > 3*M_PI/4.0 || normalized_angle < -3*M_PI/4.0) {
                collision_status_.back_min_dist = std::min(collision_status_.back_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.back_collision = true;
                    RCLCPP_WARN(this->get_logger(), 
                        "[BACK COLLISION] angle=%.3f rad (%.1f°), distance=%.3f m",
                        normalized_angle, normalized_angle * 180 / M_PI, range);
                }
            }
            // RIGHT: -3π/4 ~ -π/4 (−135° ~ −45°)
            else if (normalized_angle < -M_PI/4.0 && normalized_angle >= -3*M_PI/4.0) {
                collision_status_.right_min_dist = std::min(collision_status_.right_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.right_collision = true;
                    RCLCPP_WARN(this->get_logger(), 
                        "[RIGHT COLLISION] angle=%.3f rad (%.1f°), distance=%.3f m",
                        normalized_angle, normalized_angle * 180 / M_PI, range);
                }
            }
        }
        
        // デバッグ: スキャンサマリーを出力
        RCLCPP_DEBUG(this->get_logger(),
            "Scan processed: valid_points=%d, FRONT=%.3f m, RIGHT=%.3f m, BACK=%.3f m, LEFT=%.3f m | Collision Status: FRONT=%d, RIGHT=%d, BACK=%d, LEFT=%d",
            valid_count,
            collision_status_.front_min_dist,
            collision_status_.right_min_dist,
            collision_status_.back_min_dist,
            collision_status_.left_min_dist,
            collision_status_.front_collision,
            collision_status_.right_collision,
            collision_status_.back_collision,
            collision_status_.left_collision);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(collision_mutex_);
        
        if (!cmd_vel_received_) {
            RCLCPP_INFO(this->get_logger(), "[CMD_VEL] First command received!");
            cmd_vel_received_ = true;
        }
        
        RCLCPP_DEBUG(this->get_logger(),
            "[CMD_VEL RECEIVED] linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | Collision: FRONT=%d, BACK=%d, LEFT=%d, RIGHT=%d",
            msg->linear.x, msg->linear.y, msg->angular.z,
            collision_status_.front_collision,
            collision_status_.back_collision,
            collision_status_.left_collision,
            collision_status_.right_collision);
        
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
        
        RCLCPP_DEBUG(this->get_logger(),
            "[PUBLISHED] linear.x=%.3f, linear.y=%.3f, angular.z=%.3f | Intervention=%d",
            safe_cmd->linear.x, safe_cmd->linear.y, safe_cmd->angular.z, intervention_made);

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

    // 角度を -π ~ π の範囲に正規化
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeVelocityControllerNode>());
    rclcpp::shutdown();
    return 0;
}
