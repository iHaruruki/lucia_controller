#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <utility>

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
    SpeedSmoothingNode()
        : Node("speed_smoothing_node"),
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
        this->declare_parameter<double>("max_linear_vel", 0.4);
        this->declare_parameter<double>("max_angular_vel", 0.8);

        this->declare_parameter<double>("pid_kp_linear", 1.0);
        this->declare_parameter<double>("pid_ki_linear", 0.1);
        this->declare_parameter<double>("pid_kd_linear", 0.1);

        this->declare_parameter<double>("pid_kp_angular", 0.8);
        this->declare_parameter<double>("pid_ki_angular", 0.05);
        this->declare_parameter<double>("pid_kd_angular", 0.08);

        this->declare_parameter<int>("update_frequency", 50);

        this->get_parameter("max_linear_vel", max_linear_vel_);
        this->get_parameter("max_angular_vel", max_angular_vel_);

        double kp_linear, ki_linear, kd_linear;
        double kp_angular, ki_angular, kd_angular;
        int update_frequency;

        this->get_parameter("pid_kp_linear", kp_linear);
        this->get_parameter("pid_ki_linear", ki_linear);
        this->get_parameter("pid_kd_linear", kd_linear);

        this->get_parameter("pid_kp_angular", kp_angular);
        this->get_parameter("pid_ki_angular", ki_angular);
        this->get_parameter("pid_kd_angular", kd_angular);

        this->get_parameter("update_frequency", update_frequency);

        linear_pid_.updateGains(kp_linear, ki_linear, kd_linear);
        angular_pid_.updateGains(kp_angular, ki_angular, kd_angular);

        update_period_ms_ = std::max(1, 1000 / update_frequency);

        RCLCPP_INFO(this->get_logger(), "Update frequency: %d Hz (period: %d ms)",
                    update_frequency, update_period_ms_);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            rclcpp::SensorDataQoS(),
            std::bind(&SpeedSmoothingNode::cmdVelCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            rclcpp::SensorDataQoS(),
            std::bind(&SpeedSmoothingNode::odomCallback, this, std::placeholders::_1));

        smoothed_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "smoothed_cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_period_ms_),
            std::bind(&SpeedSmoothingNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Speed Smoothing Node initialized successfully");
        RCLCPP_INFO(this->get_logger(),
                    "Max velocities - Linear: %.2f m/s, Angular: %.2f rad/s",
                    max_linear_vel_, max_angular_vel_);
    }

private:
    PIDController linear_pid_;
    PIDController angular_pid_;

    int update_period_ms_;
    bool last_update_time_initialized_;
    std::chrono::steady_clock::time_point last_update_time_steady_;

    double current_linear_vel_;
    double current_angular_vel_;
    double target_linear_vel_;
    double target_angular_vel_;

    double max_linear_vel_;
    double max_angular_vel_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_vel_ = std::clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
        target_angular_vel_ = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

        RCLCPP_DEBUG(this->get_logger(),
                     "cmd_vel received: target_linear=%.3f, target_angular=%.3f",
                     target_linear_vel_, target_angular_vel_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_linear_vel_ = msg->twist.twist.linear.x;
        current_angular_vel_ = msg->twist.twist.angular.z;

        RCLCPP_DEBUG(this->get_logger(),
                     "odom received: current_linear=%.3f, current_angular=%.3f",
                     current_linear_vel_, current_angular_vel_);
    }

    void timerCallback()
    {
        auto current_time = std::chrono::steady_clock::now();
        double dt = static_cast<double>(update_period_ms_) / 1000.0;

        if (!last_update_time_initialized_)
        {
            last_update_time_steady_ = current_time;
            last_update_time_initialized_ = true;
            return;
        }

        dt = std::chrono::duration<double>(current_time - last_update_time_steady_).count();

        if (dt > 1.0 || dt <= 0.0)
        {
            dt = static_cast<double>(update_period_ms_) / 1000.0;
        }

        last_update_time_steady_ = current_time;

        auto [error_linear, error_angular] = errorCalculation();

        double smoothed_linear = linear_pid_.update(error_linear, dt);
        double smoothed_angular = angular_pid_.update(error_angular, dt);

        smoothed_linear = std::clamp(smoothed_linear, -max_linear_vel_, max_linear_vel_);
        smoothed_angular = std::clamp(smoothed_angular, -max_angular_vel_, max_angular_vel_);

        geometry_msgs::msg::Twist smoothed_msg;
        smoothed_msg.linear.x = smoothed_linear;
        smoothed_msg.angular.z = smoothed_angular;
        smoothed_cmd_vel_pub_->publish(smoothed_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Timer update (dt=%.4f s): smoothed_linear=%.3f, smoothed_angular=%.3f",
                     dt, smoothed_linear, smoothed_angular);
    }

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