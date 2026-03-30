#ifndef LUCIA_CONTROLLER__VELOCITY_SMOOTHER_NODE_HPP_
#define LUCIA_CONTROLLER__VELOCITY_SMOOTHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class VelocitySmootherNode : public rclcpp::Node
{
public:
    VelocitySmootherNode();
    ~VelocitySmootherNode();

private:
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_vel_pub_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

    // Smoothing functions
    double apply_smoothing(double current, double target, double smoothing_factor);
    double clamp(double value, double min, double max);
    double apply_acceleration_limit(
        double current_vel, double target_vel, double max_accel, double dt);

    // Member variables
    geometry_msgs::msg::Twist current_cmd_vel_;  // Current desired velocity command
    geometry_msgs::msg::Twist smoothed_vel_;     // Smoothed velocity to publish
    geometry_msgs::msg::Twist actual_vel_;       // Current velocity from odometry

    // Parameters
    double smoothing_factor_;
    double max_linear_vel_;
    double max_angular_vel_;
    double max_linear_accel_;
    double max_angular_accel_;
    double control_loop_rate_;  // Hz

    // Mutex for thread safety
    std::mutex vel_mutex_;
    std::mutex odom_mutex_;
};

#endif  // LUCIA_CONTROLLER__VELOCITY_SMOOTHER_NODE_HPP_