#include "lucia_controller/velocity_smoother.hpp"
#include <algorithm>
#include <cmath>

VelocitySmootherNode::VelocitySmootherNode()
    : rclcpp::Node("velocity_smoother_node")
{
    // Declare parameters
    this->declare_parameter<double>("smoothing_factor", 0.3);
    this->declare_parameter<double>("max_linear_vel", 0.4);
    this->declare_parameter<double>("max_angular_vel", 0.8);
    this->declare_parameter<double>("max_linear_accel", 0.5);
    this->declare_parameter<double>("max_angular_accel", 1.0);
    this->declare_parameter<double>("control_loop_rate", 50.0);

    // Get parameters
    this->get_parameter("smoothing_factor", smoothing_factor_);
    this->get_parameter("max_linear_vel", max_linear_vel_);
    this->get_parameter("max_angular_vel", max_angular_vel_);
    this->get_parameter("max_linear_accel", max_linear_accel_);
    this->get_parameter("max_angular_accel", max_angular_accel_);
    this->get_parameter("control_loop_rate", control_loop_rate_);

    // Clamp smoothing factor
    smoothing_factor_ = clamp(smoothing_factor_, 0.0, 1.0);

    RCLCPP_DEBUG(this->get_logger(), "Velocity Smoother Node initialized");
    RCLCPP_DEBUG(this->get_logger(), "  Smoothing factor: %.3f", smoothing_factor_);
    RCLCPP_DEBUG(this->get_logger(), "  Max linear vel: %.3f m/s", max_linear_vel_);
    RCLCPP_DEBUG(this->get_logger(), "  Max angular vel: %.3f rad/s", max_angular_vel_);
    RCLCPP_DEBUG(this->get_logger(), "  Max linear accel: %.3f m/s²", max_linear_accel_);
    RCLCPP_DEBUG(this->get_logger(), "  Max angular accel: %.3f rad/s²", max_angular_accel_);
    RCLCPP_DEBUG(this->get_logger(), "  Control loop rate: %.1f Hz", control_loop_rate_);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&VelocitySmootherNode::cmd_vel_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        std::bind(&VelocitySmootherNode::odom_callback, this, std::placeholders::_1));

    // Create publisher
    smoothed_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "smoothed_cmd_vel", 10);

    // Initialize velocity messages
    current_cmd_vel_.linear.x = 0.0;
    current_cmd_vel_.linear.y = 0.0;
    current_cmd_vel_.linear.z = 0.0;
    current_cmd_vel_.angular.x = 0.0;
    current_cmd_vel_.angular.y = 0.0;
    current_cmd_vel_.angular.z = 0.0;

    smoothed_vel_ = current_cmd_vel_;
    actual_vel_ = current_cmd_vel_;

    // Create timer for control loop
    auto timer_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / control_loop_rate_));
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&VelocitySmootherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Velocity Smoother Node ready");
}

VelocitySmootherNode::~VelocitySmootherNode()
{
    RCLCPP_INFO(this->get_logger(), "Velocity Smoother Node shutting down");
}

void VelocitySmootherNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(vel_mutex_);

    // Clamp input velocities to max limits
    current_cmd_vel_.linear.x = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
    current_cmd_vel_.linear.y = clamp(msg->linear.y, -max_linear_vel_, max_linear_vel_);
    current_cmd_vel_.linear.z = clamp(msg->linear.z, -max_linear_vel_, max_linear_vel_);

    current_cmd_vel_.angular.x = clamp(msg->angular.x, -max_angular_vel_, max_angular_vel_);
    current_cmd_vel_.angular.y = clamp(msg->angular.y, -max_angular_vel_, max_angular_vel_);
    current_cmd_vel_.angular.z = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);
}

void VelocitySmootherNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    actual_vel_ = msg->twist.twist;
}

void VelocitySmootherNode::timer_callback()
{
    std::lock_guard<std::mutex> vel_lock(vel_mutex_);
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);

    double dt = 1.0 / control_loop_rate_;

    // Apply smoothing to linear velocities
    smoothed_vel_.linear.x = apply_acceleration_limit(
        smoothed_vel_.linear.x,
        current_cmd_vel_.linear.x,
        max_linear_accel_,
        dt);

    smoothed_vel_.linear.y = apply_acceleration_limit(
        smoothed_vel_.linear.y,
        current_cmd_vel_.linear.y,
        max_linear_accel_,
        dt);

    smoothed_vel_.linear.z = apply_acceleration_limit(
        smoothed_vel_.linear.z,
        current_cmd_vel_.linear.z,
        max_linear_accel_,
        dt);

    // Apply smoothing to angular velocities
    smoothed_vel_.angular.x = apply_acceleration_limit(
        smoothed_vel_.angular.x,
        current_cmd_vel_.angular.x,
        max_angular_accel_,
        dt);

    smoothed_vel_.angular.y = apply_acceleration_limit(
        smoothed_vel_.angular.y,
        current_cmd_vel_.angular.y,
        max_angular_accel_,
        dt);

    smoothed_vel_.angular.z = apply_acceleration_limit(
        smoothed_vel_.angular.z,
        current_cmd_vel_.angular.z,
        max_angular_accel_,
        dt);

    // Apply exponential smoothing filter
    smoothed_vel_.linear.x = apply_smoothing(
        smoothed_vel_.linear.x,
        current_cmd_vel_.linear.x,
        smoothing_factor_);
    smoothed_vel_.linear.y = apply_smoothing(
        smoothed_vel_.linear.y,
        current_cmd_vel_.linear.y,
        smoothing_factor_);
    smoothed_vel_.linear.z = apply_smoothing(
        smoothed_vel_.linear.z,
        current_cmd_vel_.linear.z,
        smoothing_factor_);

    smoothed_vel_.angular.x = apply_smoothing(
        smoothed_vel_.angular.x,
        current_cmd_vel_.angular.x,
        smoothing_factor_);
    smoothed_vel_.angular.y = apply_smoothing(
        smoothed_vel_.angular.y,
        current_cmd_vel_.angular.y,
        smoothing_factor_);
    smoothed_vel_.angular.z = apply_smoothing(
        smoothed_vel_.angular.z,
        current_cmd_vel_.angular.z,
        smoothing_factor_);

    // Publish smoothed velocity
    smoothed_cmd_vel_pub_->publish(smoothed_vel_);
}

double VelocitySmootherNode::apply_smoothing(
    double current, double target, double smoothing_factor)
{
    // Exponential smoothing: y = α * target + (1 - α) * current
    // Higher α → faster response to target changes
    return smoothing_factor * target + (1.0 - smoothing_factor) * current;
}

double VelocitySmootherNode::clamp(
    double value, double min, double max)
{
    return std::max(min, std::min(value, max));
}

double VelocitySmootherNode::apply_acceleration_limit(
    double current_vel, double target_vel, double max_accel, double dt)
{
    // Calculate maximum velocity change allowed in this time step
    double max_change = max_accel * dt;

    // Calculate desired change
    double desired_change = target_vel - current_vel;

    // Clamp desired change to acceleration limit
    double limited_change = clamp(desired_change, -max_change, max_change);

    // Return new velocity
    return current_vel + limited_change;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocitySmootherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}