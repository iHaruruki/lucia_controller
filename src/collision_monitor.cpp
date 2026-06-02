#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class ScanFrontFilterNode : public rclcpp::Node {
public:
    ScanFrontFilterNode() : Node("scan_front_filter") {
        // /scan トピックを購読
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ScanFrontFilterNode::scan_callback, this, std::placeholders::_1));
        
        // /scan/rviz トピックを配信
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan/rviz", 10);
        
        // RCLCPP_INFO(this->get_logger(), 
        //     "Scan Front Filter Node started.\n"
        //     "Subscribing to: /scan\n"
        //     "Publishing to: /scan/rviz\n"
        //     "Filter: Front direction only (-45° to +45°)");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    
    // 正面方向の角度範囲（ラジアン）
    static constexpr double FRONT_ANGLE_MIN = M_PI / 4.0;  // -45°
    static constexpr double FRONT_ANGLE_MAX = M_PI*3 / 4.0;   // +45°

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // フィルター済みのLaserScanメッセージを作成
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        
        // ヘッダー情報をコピー
        filtered_msg->header = msg->header;
        filtered_msg->header.frame_id = msg->header.frame_id;
        
        // スキャン設定をコピー（角度範囲は正面方向に調整）
        filtered_msg->angle_min = FRONT_ANGLE_MIN;
        filtered_msg->angle_max = FRONT_ANGLE_MAX;
        filtered_msg->angle_increment = msg->angle_increment;
        filtered_msg->time_increment = msg->time_increment;
        filtered_msg->scan_time = msg->scan_time;
        filtered_msg->range_min = msg->range_min;
        filtered_msg->range_max = msg->range_max;
        
        // 正面方向のデータのみを抽出
        std::vector<float> front_ranges;
        size_t front_count = 0;
        
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            // i番目の測定点の角度を計算
            double angle = msg->angle_min + i * msg->angle_increment;
            
            // 角度が正面方向範囲内にあるかチェック
            if (angle >= FRONT_ANGLE_MIN && angle <= FRONT_ANGLE_MAX) {
                front_ranges.push_back(msg->ranges[i]);
                front_count++;
            }
        }
        
        // フィルター済みのrangesをセット
        filtered_msg->ranges = front_ranges;
        
        // intensitiesがある場合は空のまま（またはコピー可能）
        filtered_msg->intensities.clear();
        
        // /scan/rviz トピックに配信
        publisher_->publish(*filtered_msg);
        
        // ログ出力（最初のメッセージのみ詳細表示）
        static bool first_message = true;
        if (first_message) {
            RCLCPP_INFO(this->get_logger(),
                "\n========== FRONT FILTER ANALYSIS ==========\n"
                "Original scan: %zu measurements (-180° to +180°)\n"
                "Filtered scan: %zu measurements (-45° to +45°)\n"
                "Filtering rate: %.1f%%\n"
                "Original angle_min: %.4f rad (%.1f°)\n"
                "Original angle_max: %.4f rad (%.1f°)\n"
                "Filtered angle_min: %.4f rad (%.1f°)\n"
                "Filtered angle_max: %.4f rad (%.1f°)\n"
                "Angle increment: %.6f rad (%.3f°)\n"
                "Frame ID: %s\n"
                "==========================================\n",
                msg->ranges.size(),
                front_count,
                (front_count * 100.0) / msg->ranges.size(),
                msg->angle_min, radians_to_degrees(msg->angle_min),
                msg->angle_max, radians_to_degrees(msg->angle_max),
                FRONT_ANGLE_MIN, radians_to_degrees(FRONT_ANGLE_MIN),
                FRONT_ANGLE_MAX, radians_to_degrees(FRONT_ANGLE_MAX),
                msg->angle_increment, radians_to_degrees(msg->angle_increment),
                msg->header.frame_id.c_str());
            first_message = false;
        }
    }
    
    double radians_to_degrees(double radians) {
        return radians * 180.0 / M_PI;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFrontFilterNode>());
    rclcpp::shutdown();
    return 0;
}