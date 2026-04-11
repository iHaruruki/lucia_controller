#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <chrono>

// モジュール化されたPIDコントローラクラス
class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0), prev_error_(0.0),
          integral_limit_(1.0)
    {
    }

    void updateGains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    double p_control(double error)
    {
        return kp_ * error;
    }

    double i_control(double error, double dt)
    {
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        return ki_ * integral_;
    }

    double d_control(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;

        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kd_ * derivative;
    }

    double update(double error, double dt)
    {
        double p_term = p_control(error);
        double i_term = i_control(error, dt);
        double d_term = d_control(error, dt);
        return p_term + i_term + d_term;
    }

    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double integral_;
    double prev_error_;
    double integral_limit_;
};

class SpeedSmoothingNode : public rclcpp::Node
{
public:
    SpeedSmoothingNode() : Node("speed_smoothing_node"),
                           linear_pid_(1.0, 0.1, 0.1),
                           angular_pid_(0.8, 0.05, 0.08),
                           update_period_ms_(20),
                           last_update_time_initialized_(false),
                           current_linear_vel_(0.0),
                           current_angular_vel_(0.0),
                           target_linear_vel_(0.0),
                           target_angular_vel_(0.0),
                           max_linear_vel_(0.4),
                           max_angular_vel_(0.8)
    {
        // Declare ROS parameters
        this->declare_parameter<double>("max_linear_vel", 0.4);
        this->declare_parameter<double>("max_angular_vel", 0.8);

        this->declare_parameter<double>("pid_kp_linear", 1.0);
        this->declare_parameter<double>("pid_ki_linear", 0.1);
        this->declare_parameter<double>("pid_kd_linear", 0.1);

        this->declare_parameter<double>("pid_kp_angular", 0.8);
        this->declare_parameter<double>("pid_ki_angular", 0.05);
        this->declare_parameter<double>("pid_kd_angular", 0.08);

        this->declare_parameter<int>("update_frequency", 50);
        this->declare_parameter<int>("message_queue_size", 10);
        this->declare_parameter<double>("sync_tolerance", 0.1);

        // Get parameters
        this->get_parameter("max_linear_vel", max_linear_vel_);
        this->get_parameter("max_angular_vel", max_angular_vel_);

        double kp_linear, ki_linear, kd_linear;
        double kp_angular, ki_angular, kd_angular;
        int update_frequency;
        int message_queue_size;

        this->get_parameter("pid_kp_linear", kp_linear);
        this->get_parameter("pid_ki_linear", ki_linear);
        this->get_parameter("pid_kd_linear", kd_linear);

        this->get_parameter("pid_kp_angular", kp_angular);
        this->get_parameter("pid_ki_angular", ki_angular);
        this->get_parameter("pid_kd_angular", kd_angular);

        this->get_parameter("update_frequency", update_frequency);
        this->get_parameter("message_queue_size", message_queue_size);

        // Initialize PID controllers with parameters
        linear_pid_.updateGains(kp_linear, ki_linear, kd_linear);
        angular_pid_.updateGains(kp_angular, ki_angular, kd_angular);

        // Calculate update period in milliseconds
        update_period_ms_ = std::max(1, 1000 / update_frequency);

        RCLCPP_INFO(this->get_logger(), "Update frequency: %d Hz (period: %d ms)",
                    update_frequency, update_period_ms_);

        // Create message filter subscribers
        cmd_vel_sub_.subscribe(this, "cmd_vel", rmw_qos_profile_sensor_data);
        odom_sub_.subscribe(this, "odom", rmw_qos_profile_sensor_data);

        // Create synchronizer with ApproximateTime policy
        typedef message_filters::sync_policies::ApproximateTime<
            geometry_msgs::msg::Twist, nav_msgs::msg::Odometry>
            SyncPolicy;

        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(message_queue_size), cmd_vel_sub_, odom_sub_);

        // registerCallback の正しい使用法
        sync_->registerCallback(std::bind(&SpeedSmoothingNode::syncCallback, this,
                                         std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "Message synchronizer initialized (queue size: %d)",
                    message_queue_size);

        // Create publisher for smoothed commands
        smoothed_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "smoothed_cmd_vel", 10);

        // Create timer for periodic updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_period_ms_),
            std::bind(&SpeedSmoothingNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Speed Smoothing Node initialized successfully");
        RCLCPP_INFO(this->get_logger(),
                    "Max velocities - Linear: %.2f m/s, Angular: %.2f rad/s",
                    max_linear_vel_, max_angular_vel_);
    }

private:
    // PID Controllers
    PIDController linear_pid_;
    PIDController angular_pid_;

    rclcpp::Time last_update_time_;
    int update_period_ms_;
    bool last_update_time_initialized_;

    // State variables
    double current_linear_vel_;
    double current_angular_vel_;
    double target_linear_vel_;
    double target_angular_vel_;

    // Parameters
    double max_linear_vel_;
    double max_angular_vel_;

    // Message filters
    message_filters::Subscriber<geometry_msgs::msg::Twist> cmd_vel_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::Twist, nav_msgs::msg::Odometry>
        SyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ROS interface
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 同期コールバック
    void syncCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &cmd_vel_msg,
                      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        // Update target velocities from command
        target_linear_vel_ = cmd_vel_msg->linear.x;
        target_angular_vel_ = cmd_vel_msg->angular.z;

        // Clamp to max velocities
        target_linear_vel_ = std::clamp(target_linear_vel_, -max_linear_vel_, max_linear_vel_);
        target_angular_vel_ = std::clamp(target_angular_vel_, -max_angular_vel_, max_angular_vel_);

        // Update current velocities from odometry
        current_linear_vel_ = odom_msg->twist.twist.linear.x;
        current_angular_vel_ = odom_msg->twist.twist.angular.z;

        RCLCPP_DEBUG(this->get_logger(),
                     "Synchronized: target_linear=%.3f, current_linear=%.3f, "
                     "target_angular=%.3f, current_angular=%.3f",
                     target_linear_vel_, current_linear_vel_,
                     target_angular_vel_, current_angular_vel_);
    }

    // タイマーコールバック
    void timerCallback()
    {
        // 前回の更新からの経過時間を計算
        rclcpp::Time current_time = this->get_clock()->now();
        // rclcpp::Duration elapsed = current_time - last_update_time_;
        // double dt = elapsed.seconds();
        
        double dt = static_cast<double>(update_period_ms_) / 1000.0;

        // 初回実行時は last_update_time_ を初期化
        if (!last_update_time_initialized_)
        {
            last_update_time_ = current_time;
            last_update_time_initialized_ = true;
            RCLCPP_DEBUG(this->get_logger(), "First timer callback, initializing last_update_time");
            return;
        }

        // 経過時間を計算
        rclcpp::Duration elapsed = current_time - last_update_time_;
        dt = elapsed.seconds();
        
        // dt が異常に大きい場合は設定値を使用
        if (dt > 1.0 || dt <= 0.0)
        {
            dt = static_cast<double>(update_period_ms_) / 1000.0;
        }

        last_update_time_ = current_time;

        // エラーを計算
        auto [error_linear, error_angular] = errorCalculation();

        // PIDコントロールを適用
        double smoothed_linear = linear_pid_.update(error_linear, dt);
        double smoothed_angular = angular_pid_.update(error_angular, dt);

        // 最終出力をクランプ
        smoothed_linear = std::clamp(smoothed_linear, -max_linear_vel_, max_linear_vel_);
        smoothed_angular = std::clamp(smoothed_angular, -max_angular_vel_, max_angular_vel_);

        // 平滑化されたコマンドを発行
        geometry_msgs::msg::Twist smoothed_msg;
        smoothed_msg.linear.x = smoothed_linear;
        smoothed_msg.angular.z = smoothed_angular;
        smoothed_cmd_vel_pub_->publish(smoothed_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Timer update (dt=%.4f s): smoothed_linear=%.3f, smoothed_angular=%.3f",
                     dt, smoothed_linear, smoothed_angular);
    }

    // エラー計算
    std::pair<double, double> errorCalculation()
    {
        double linear_error = target_linear_vel_ - current_linear_vel_;
        double angular_error = target_angular_vel_ - current_angular_vel_;
        return std::make_pair(linear_error, angular_error);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedSmoothingNode>());
    rclcpp::shutdown();
    return 0;
}