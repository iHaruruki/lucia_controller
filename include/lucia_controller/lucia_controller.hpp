#ifndef LUCIA_CONTROLLER__LUCIA_CONTROLLER_HPP_
#define LUCIA_CONTROLLER__LUCIA_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <yarp/os/all.h>
#include <mutex>
#include <chrono>

// Vehicle constants
namespace VehicleStateConstants {
    constexpr double DEFAULT_DT = 0.05;  // 50ms
    constexpr double MIN_DT = 0.01;      // 10ms
    constexpr double MAX_DT = 0.2;       // 200ms
    constexpr size_t CMD_DATA_SIZE = 4;
}

class LuciaController : public rclcpp::Node
{
public:
    LuciaController();
    ~LuciaController();

private:
    // YARP
    yarp::os::BufferedPort<yarp::os::Bottle> p_cmd;
    yarp::os::BufferedPort<yarp::os::Bottle> p_enc;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr encoder_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;  // ← 新規

    // Callbacks
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encoder_timer_callback();
    void watchdog_timer_callback();  // ← 新規

    // Helper methods
    void send_velocity_command(const std::vector<double>& cmd);
    void readEncoderAndUpdate(double dt, const rclcpp::Time& stamp);
    void integrate(double vx, double vy, double vth, double dt);
    void publishOdometry(const rclcpp::Time& stamp, double vx, double vy, double vth);

    // Mutex
    std::mutex yarp_mutex_;

    // Odometry state
    double x_, y_, yaw_;
    double dt_;

    // Time tracking
    rclcpp::Time last_callback_time_;
    std::chrono::steady_clock::time_point last_cmd_time_;  // ← 新規

    // Watchdog timeout
    static constexpr std::chrono::milliseconds CMD_TIMEOUT{500};  // ← 新規

    // Error tracking
    int encoder_failure_count_ = 0;
    int encoder_error_count_ = 0;
    int count = 0;
    bool cmd_timeout_warned_ = false;  // ← 新規
};

#endif  // LUCIA_CONTROLLER__LUCIA_CONTROLLER_HPP_