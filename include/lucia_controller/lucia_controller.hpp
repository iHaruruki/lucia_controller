#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yarp/os/all.h>
#include <mutex>
#include <memory>

namespace VehicleStateConstants {
    constexpr double DEFAULT_DT = 0.02;
    constexpr double MIN_DT = 0.03;
    constexpr double MAX_DT = 0.07;
    constexpr size_t CMD_DATA_SIZE = 4;
    constexpr size_t ENCODER_DATA_SIZE = 3;
}

class LuciaController : public rclcpp::Node {
public:
    LuciaController();
    ~LuciaController();

private:
    // Callbacks
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encoder_timer_callback();

    // Encoder processing
    void readEncoderAndUpdate(double dt, const rclcpp::Time& stamp);
    void integrate(double vx, double vy, double vth, double dt);
    void publishOdometry(const rclcpp::Time& stamp, double vx, double vy, double vth);

    // Command
    void send_velocity_command(const std::vector<double>& cmd);

    // State variables
    double x_, y_, yaw_;
    double dt_;
    rclcpp::Time last_callback_time_;

    int count;

    // YARP
    yarp::os::BufferedPort<yarp::os::Bottle> p_cmd;
    yarp::os::BufferedPort<yarp::os::Bottle> p_enc;
    std::mutex yarp_mutex_;

    // ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::TimerBase::SharedPtr encoder_timer_;

    // Diagnostics
    int encoder_failure_count_ = 0;
    int encoder_error_count_ = 0;
};