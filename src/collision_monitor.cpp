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
            "Publishing to: /collision_monitor/cmd_vel, /intervention_status\n"
            "=== SAFETY MODE: Publishes stop command immediately on collision detection ===",
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
    geometry_msgs::msg::Twist last_cmd_vel_;
    std::mutex data_mutex_;
    
    // パラメータ
    static constexpr float ROBOT_RADIUS = 0.25f;
    static constexpr float COLLISION_THRESHOLD = 0.6f;
    static constexpr float WARNING_THRESHOLD = 0.8f;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 前回の衝突状態を保存（変化を検出するため）
        bool prev_front_collision = collision_status_.front_collision;
        bool prev_back_collision = collision_status_.back_collision;
        bool prev_left_collision = collision_status_.left_collision;
        bool prev_right_collision = collision_status_.right_collision;
        
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

            // 角度を正規化 (-π ~ π)
            double normalized_angle = normalize_angle(angle);
            
            // FRONT: -π/4 ~ π/4 (−45° ~ 45°)
            if (normalized_angle >= -M_PI/4.0 && normalized_angle <= M_PI/4.0) {
                collision_status_.front_min_dist = std::min(collision_status_.front_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.front_collision = true;
                }
            }
            // LEFT: π/4 ~ 3π/4 (45° ~ 135°)
            else if (normalized_angle > M_PI/4.0 && normalized_angle <= 3*M_PI/4.0) {
                collision_status_.left_min_dist = std::min(collision_status_.left_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.left_collision = true;
                }
            }
            // BACK: 3π/4 ~ -3π/4 (135° ~ -135°, wrapping around ±π)
            else if (normalized_angle > 3*M_PI/4.0 || normalized_angle < -3*M_PI/4.0) {
                collision_status_.back_min_dist = std::min(collision_status_.back_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.back_collision = true;
                }
            }
            // RIGHT: -3π/4 ~ -π/4 (−135° ~ −45°)
            else if (normalized_angle < -M_PI/4.0 && normalized_angle >= -3*M_PI/4.0) {
                collision_status_.right_min_dist = std::min(collision_status_.right_min_dist, range);
                if (range <= COLLISION_THRESHOLD) {
                    collision_status_.right_collision = true;
                }
            }
        }
        
        // === SAFETY: 衝突検出時にすぐに安全速度を発行 ===
        bool collision_detected = collision_status_.front_collision || 
                                  collision_status_.back_collision ||
                                  collision_status_.left_collision ||
                                  collision_status_.right_collision;
        
        if (collision_detected) {
            // 衝突状態を検出したら、すぐに安全コマンドを生成して発行
            auto safe_cmd = generate_safe_velocity(last_cmd_vel_);
            safe_velocity_publisher_->publish(safe_cmd);
            
            // ステータスを発行
            auto status_msg = std_msgs::msg::String();
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3);
            ss << "[COLLISION DETECTED]\n"
               << "FRONT: " << (collision_status_.front_collision ? "COLLISION" : "SAFE") << " (" << collision_status_.front_min_dist << " m)\n"
               << "RIGHT: " << (collision_status_.right_collision ? "COLLISION" : "SAFE") << " (" << collision_status_.right_min_dist << " m)\n"
               << "BACK: " << (collision_status_.back_collision ? "COLLISION" : "SAFE") << " (" << collision_status_.back_min_dist << " m)\n"
               << "LEFT: " << (collision_status_.left_collision ? "COLLISION" : "SAFE") << " (" << collision_status_.left_min_dist << " m)\n"
               << "Last cmd_vel: x=" << last_cmd_vel_.linear.x << ", y=" << last_cmd_vel_.linear.y << "\n"
               << "Safe cmd_vel: x=" << safe_cmd.linear.x << ", y=" << safe_cmd.linear.y;
            status_msg.data = ss.str();
            intervention_publisher_->publish(status_msg);
            
            // 衝突状態の変化をログ
            if (!prev_front_collision && collision_status_.front_collision) {
                RCLCPP_DEBUG(this->get_logger(), "[SAFETY] FRONT COLLISION DETECTED - Publishing stop command!");
            }
            if (!prev_back_collision && collision_status_.back_collision) {
                RCLCPP_DEBUG(this->get_logger(), "[SAFETY] BACK COLLISION DETECTED - Publishing stop command!");
            }
            if (!prev_left_collision && collision_status_.left_collision) {
                RCLCPP_DEBUG(this->get_logger(), "[SAFETY] LEFT COLLISION DETECTED - Publishing stop command!");
            }
            if (!prev_right_collision && collision_status_.right_collision) {
                RCLCPP_DEBUG(this->get_logger(), "[SAFETY] RIGHT COLLISION DETECTED - Publishing stop command!");
            }
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 最新のコマンドを保存
        last_cmd_vel_ = *msg;
        
        RCLCPP_DEBUG(this->get_logger(),
            "[CMD_VEL] Received: x=%.3f, y=%.3f, z=%.3f",
            msg->linear.x, msg->linear.y, msg->angular.z);
        
        // 現在の衝突状態で安全なコマンドを生成
        auto safe_cmd = generate_safe_velocity(*msg);
        
        // 安全なコマンドを発行
        safe_velocity_publisher_->publish(safe_cmd);
        
        RCLCPP_DEBUG(this->get_logger(),
            "[SAFE_VEL] Published: x=%.3f, y=%.3f, z=%.3f | Collision: F=%d, B=%d, L=%d, R=%d",
            safe_cmd.linear.x, safe_cmd.linear.y, safe_cmd.angular.z,
            collision_status_.front_collision,
            collision_status_.back_collision,
            collision_status_.left_collision,
            collision_status_.right_collision);
    }

    // 安全な速度コマンドを生成
    geometry_msgs::msg::Twist generate_safe_velocity(const geometry_msgs::msg::Twist& cmd) {
        auto safe_cmd = cmd;
        bool intervention_made = false;
        std::vector<std::string> blocked_directions;

        // 前進が衝突方向に該当するか確認
        if (collision_status_.front_collision && cmd.linear.x > 0.0) {
            safe_cmd.linear.x = 0.0;
            intervention_made = true;
            blocked_directions.push_back("FRONT");
        }

        // 後進が衝突方向に該当するか確認
        if (collision_status_.back_collision && cmd.linear.x < 0.0) {
            safe_cmd.linear.x = 0.0;
            intervention_made = true;
            blocked_directions.push_back("BACK");
        }

        // 左方向への移動が衝突方向に該当するか確認
        if (collision_status_.left_collision && cmd.linear.y > 0.0) {
            safe_cmd.linear.y = 0.0;
            intervention_made = true;
            blocked_directions.push_back("LEFT");
        }

        // 右方向への移動が衝突方向に該当するか確認
        if (collision_status_.right_collision && cmd.linear.y < 0.0) {
            safe_cmd.linear.y = 0.0;
            intervention_made = true;
            blocked_directions.push_back("RIGHT");
        }

        // 介入があればログ出力
        if (intervention_made) {
            std::stringstream ss;
            ss << "Blocked: ";
            for (size_t i = 0; i < blocked_directions.size(); i++) {
                ss << blocked_directions[i];
                if (i < blocked_directions.size() - 1) ss << ", ";
            }
            RCLCPP_WARN(this->get_logger(), "[INTERVENTION] %s | Original: x=%.3f, y=%.3f → Safe: x=%.3f, y=%.3f",
                ss.str().c_str(), cmd.linear.x, cmd.linear.y, safe_cmd.linear.x, safe_cmd.linear.y);
        }

        return safe_cmd;
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