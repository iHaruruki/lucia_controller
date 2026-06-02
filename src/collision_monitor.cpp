#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>

class CollisionDetectorNode : public rclcpp::Node {
public:
    CollisionDetectorNode() : Node("collision_detector") {
        // /scan トピックを購読
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&CollisionDetectorNode::scan_callback, this, std::placeholders::_1));
        
        // 衝突状態を配信
        publisher_ = this->create_publisher<std_msgs::msg::String>("/collision_status", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "Collision Detector Node started.\n"
            "Robot Radius: %.2f m\n"
            "Collision Threshold: %.2f m\n"
            "Subscribing to: /scan\n"
            "Publishing to: /collision_status",
            ROBOT_RADIUS, COLLISION_THRESHOLD);
    }

private:
    struct DirectionData {
        std::string name;
        double angle_min;
        double angle_max;
        float min_distance = std::numeric_limits<float>::max();
        size_t critical_count = 0;
        size_t total_count = 0;
        bool is_colliding = false;
        std::string status_icon;
    };

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    // パラメータ
    static constexpr float ROBOT_RADIUS = 0.25f;           // ロボット半径 (m)
    static constexpr float COLLISION_THRESHOLD = 0.3f;     // 衝突判定閾値 (m)
    static constexpr float WARNING_THRESHOLD = 0.5f;       // 警告閾値 (m)
    
    bool first_message = true;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 4方向のデータを定義
        DirectionData directions[4] = {
            {"FRONT", -M_PI/4.0, M_PI/4.0, std::numeric_limits<float>::max(), 0, 0, false, ""},
            {"RIGHT", -3*M_PI/4.0, -M_PI/4.0, std::numeric_limits<float>::max(), 0, 0, false, ""},
            {"BACK", 3*M_PI/4.0, -3*M_PI/4.0, std::numeric_limits<float>::max(), 0, 0, false, ""},
            {"LEFT", M_PI/4.0, 3*M_PI/4.0, std::numeric_limits<float>::max(), 0, 0, false, ""}
        };

        // スキャンデータを各方向に分類
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            // 有効な距離値かチェック
            if (range < msg->range_min || range > msg->range_max) {
                continue;
            }

            // 各方向に該当するかチェック
            for (int d = 0; d < 4; d++) {
                if (is_angle_in_direction(angle, directions[d].angle_min, directions[d].angle_max)) {
                    directions[d].total_count++;
                    directions[d].min_distance = std::min(directions[d].min_distance, range);
                    
                    // 衝突判定
                    if (range <= COLLISION_THRESHOLD) {
                        directions[d].critical_count++;
                        directions[d].is_colliding = true;
                    }
                }
            }
        }

        // 各方向のステータスアイコンを設定
        for (int d = 0; d < 4; d++) {
            if (directions[d].is_colliding) {
                directions[d].status_icon = "✗ COLLISION";
            } else if (directions[d].min_distance <= WARNING_THRESHOLD) {
                directions[d].status_icon = "⚠ WARNING";
            } else {
                directions[d].status_icon = "✓ SAFE";
            }
        }

        // 結果を出力・配信
        print_and_publish_results(directions);
    }

    bool is_angle_in_direction(double angle, double min_angle, double max_angle) {
        // BACK方向（±π をまたぐ）の特殊処理
        if (min_angle > max_angle) {  // 3π/4 > -3π/4 のケース
            return (angle >= min_angle || angle <= max_angle);
        }
        return (angle >= min_angle && angle <= max_angle);
    }

    void print_and_publish_results(DirectionData directions[]) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);

        // ヘッダー
        if (first_message) {
            ss << "\n" << std::string(70, '=') << "\n";
            ss << "COLLISION DETECTION SYSTEM STARTED\n";
            ss << "Robot Radius: " << ROBOT_RADIUS << " m\n";
            ss << "Collision Threshold: " << COLLISION_THRESHOLD << " m\n";
            ss << "Warning Threshold: " << WARNING_THRESHOLD << " m\n";
            ss << std::string(70, '=') << "\n\n";
            first_message = false;
        }

        // 現在の状態を表示
        ss << "\r";  // キャリッジリターン（上書き）
        ss << "TIME: " << std::setw(6) << std::setfill('0') << (int)(rclcpp::Clock().now().seconds()) % 100000 << " | ";
        
        // 各方向の状態
        for (int d = 0; d < 4; d++) {
            ss << directions[d].name << ": " << directions[d].status_icon;
            ss << " (" << directions[d].min_distance << "m)";
            if (d < 3) ss << " | ";
        }

        // 詳細情報
        ss << "\n";
        ss << "───────────────────────────────────────────────────────────────────\n";
        
        bool any_collision = false;
        for (int d = 0; d < 4; d++) {
            ss << "  " << std::setw(5) << directions[d].name << ": ";
            ss << std::setw(18) << directions[d].status_icon;
            ss << " | Min: " << std::setw(5) << directions[d].min_distance << " m";
            ss << " | Critical: " << std::setw(3) << directions[d].critical_count;
            ss << "/" << std::setw(4) << directions[d].total_count;
            
            if (directions[d].is_colliding) {
                any_collision = true;
                ss << " ⚠ ALERT!";
            }
            ss << "\n";
        }

        ss << "───────────────────────────────────────────────────────────────────\n";
        
        // 総合判定
        if (any_collision) {
            ss << "OVERALL STATUS: ✗ COLLISION DETECTED - EMERGENCY STOP REQUIRED!\n";
        } else {
            ss << "OVERALL STATUS: ✓ SAFE - No collision detected\n";
        }
        ss << std::string(70, '=') << "\n";

        // コンソール出力
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, ss.str().c_str());

        // /collision_status トピックに配信
        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        publisher_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}