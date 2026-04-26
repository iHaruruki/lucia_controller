#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

class SpeedSmoothingNode : public rclcpp::Node
{
public:
    SpeedSmoothingNode()
    : Node("speed_smoothing_node"),
      last_update_time_initialized_(false),
      target_linear_vel_x_(0.0),
      target_linear_vel_y_(0.0),
      target_angular_vel_(0.0),
      current_linear_vel_x_(0.0),
      current_linear_vel_y_(0.0),
      current_angular_vel_(0.0),
      max_linear_vel_x_(0.4),
      max_linear_vel_y_(0.4),
      max_angular_vel_(0.8),
      tau_linear_x_(0.2),
      tau_linear_y_(0.2),
      tau_angular_(0.2),
      update_period_ms_(20),
      update_frequency_(50)
    {
        // Parameters
        this->declare_parameter<double>("max_linear_vel_x", 0.3);
        this->declare_parameter<double>("max_linear_vel_y", 0.3);
        this->declare_parameter<double>("max_angular_vel", 0.8);
        this->declare_parameter<double>("tau_linear_x", 0.7); // 0(quickly) - 1(slowly) 
        this->declare_parameter<double>("tau_linear_y", 0.7); // 0(quickly) - 1(slowly) 
        this->declare_parameter<double>("tau_angular", 0.2); // 0(quickly) - 1(slowly) 
        this->declare_parameter<int>("update_frequency", 30);

        this->get_parameter("max_linear_vel_x", max_linear_vel_x_);
        this->get_parameter("max_linear_vel_y", max_linear_vel_y_);
        this->get_parameter("max_angular_vel", max_angular_vel_);
        this->get_parameter("tau_linear_x", tau_linear_x_);
        this->get_parameter("tau_linear_y", tau_linear_y_);
        this->get_parameter("tau_angular", tau_angular_);
        this->get_parameter("update_frequency", update_frequency_);

        update_frequency_ = std::max(1, update_frequency_);
        update_period_ms_ = std::max(1, 1000 / update_frequency_);

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&SpeedSmoothingNode::cmdVelCallback, this, std::placeholders::_1));

        // Publisher
        smoothed_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "smoothed_cmd_vel",
            10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_period_ms_),
            std::bind(&SpeedSmoothingNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Speed smoothing node initialized");
        RCLCPP_INFO(this->get_logger(), "Update frequency: %d Hz (period: %d ms)",
                    update_frequency_, update_period_ms_);
        RCLCPP_INFO(this->get_logger(), "Max linear vel X: %.3f m/s", max_linear_vel_x_);
        RCLCPP_INFO(this->get_logger(), "Max linear vel Y: %.3f m/s", max_linear_vel_y_);
        RCLCPP_INFO(this->get_logger(), "Max angular vel : %.3f rad/s", max_angular_vel_);
        RCLCPP_INFO(this->get_logger(), "Tau linear X    : %.3f s", tau_linear_x_);
        RCLCPP_INFO(this->get_logger(), "Tau linear Y    : %.3f s", tau_linear_y_);
        RCLCPP_INFO(this->get_logger(), "Tau angular     : %.3f s", tau_angular_);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_update_time_;
    bool last_update_time_initialized_;

    double target_linear_vel_x_;
    double target_linear_vel_y_;
    double target_angular_vel_;
    double current_linear_vel_x_;
    double current_linear_vel_y_;
    double current_angular_vel_;

    double max_linear_vel_x_;
    double max_linear_vel_y_;
    double max_angular_vel_;
    double tau_linear_x_;
    double tau_linear_y_;
    double tau_angular_;

    int update_period_ms_;
    int update_frequency_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 受信した目標速度を最大速度で制限
        target_linear_vel_x_ = std::clamp(msg->linear.x, -max_linear_vel_x_, max_linear_vel_x_);
        target_linear_vel_y_ = std::clamp(msg->linear.y, -max_linear_vel_y_, max_linear_vel_y_);
        target_angular_vel_ = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

        RCLCPP_DEBUG(this->get_logger(),
                     "Received cmd_vel: target_linear_x=%.3f, target_linear_y=%.3f, target_angular=%.3f",
                     target_linear_vel_x_, target_linear_vel_y_, target_angular_vel_);
    }

    void timerCallback()
    {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = static_cast<double>(update_period_ms_) / 1000.0;

        if (!last_update_time_initialized_)
        {
            last_update_time_ = current_time;
            last_update_time_initialized_ = true;
            return;
        }

        dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // 異常な dt を防ぐ
        if (dt <= 0.0 || dt > 1.0)
        {
            dt = static_cast<double>(update_period_ms_) / 1000.0;
        }

        // tau が不正なときの保護
        const double safe_tau_linear_x = std::max(1e-6, tau_linear_x_);
        const double safe_tau_linear_y = std::max(1e-6, tau_linear_y_);
        const double safe_tau_angular = std::max(1e-6, tau_angular_);

        // 時間依存の一次遅れフィルタ
        const double alpha_linear_x = 1.0 - std::exp(-dt / safe_tau_linear_x);
        const double alpha_linear_y = 1.0 - std::exp(-dt / safe_tau_linear_y);
        const double alpha_angular = 1.0 - std::exp(-dt / safe_tau_angular);

        current_linear_vel_x_ = current_linear_vel_x_ + alpha_linear_x * (target_linear_vel_x_ - current_linear_vel_x_);
        current_linear_vel_y_ = current_linear_vel_y_ + alpha_linear_y * (target_linear_vel_y_ - current_linear_vel_y_);
        current_angular_vel_ = current_angular_vel_ + alpha_angular * (target_angular_vel_ - current_angular_vel_);

        // 最終出力も最大速度で制限
        current_linear_vel_x_ = std::clamp(current_linear_vel_x_, -max_linear_vel_x_, max_linear_vel_x_);
        current_linear_vel_y_ = std::clamp(current_linear_vel_y_, -max_linear_vel_y_, max_linear_vel_y_);
        current_angular_vel_ = std::clamp(current_angular_vel_, -max_angular_vel_, max_angular_vel_);

        geometry_msgs::msg::Twist out_msg;
        out_msg.linear.x = current_linear_vel_x_;
        out_msg.linear.y = current_linear_vel_y_;
        out_msg.linear.z = 0.0;
        out_msg.angular.x = 0.0;
        out_msg.angular.y = 0.0;
        out_msg.angular.z = current_angular_vel_;

        smoothed_cmd_vel_pub_->publish(out_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Published smoothed_cmd_vel: linear_x=%.3f, linear_y=%.3f, angular=%.3f, dt=%.4f, alpha_x=%.4f, alpha_y=%.4f, alpha_ang=%.4f",
                     current_linear_vel_x_, current_linear_vel_y_, current_angular_vel_, dt, alpha_linear_x, alpha_linear_y, alpha_angular);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedSmoothingNode>());
    rclcpp::shutdown();
    return 0;
}
