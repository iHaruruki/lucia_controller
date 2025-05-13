#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class LuciaController : public rclcpp::Node
{
public:
  LuciaController()
  : Node("lucia_controller"), x_(0.0), y_(0.0), theta_(0.0),
    vx_(0.0), vy_(0.0), omega_(0.0)
  {
    // cmd_vel subscriber (for control input; here for logging)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&LuciaController::cmdVelCallback, this, std::placeholders::_1));

    // Encoder data subscriber (reads vx, vy, and omega from encoder)
    encoder_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "encoder", 10,
      std::bind(&LuciaController::encoderCallback, this, std::placeholders::_1));

    // /odom publisher (publishes nav_msgs/msg/Odometry)
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Timer to update odometry every 100ms
    timer_ = this->create_wall_timer(
      100ms, std::bind(&LuciaController::updateOdometry, this));
  }

private:
  // Callback for cmd_vel data: logs received control commands.
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f angular=%.2f",
                msg->linear.x, msg->angular.z);
  }

  // Callback for encoder data: updates vx, vy, and omega.
  void encoderCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    omega_ = msg->angular.z;

    RCLCPP_INFO(this->get_logger(), "Encoder data: vx=%.2f, vy=%.2f, omega=%.2f",
                vx_, vy_, omega_);
  }

  // Timer callback to compute and publish odometry using encoder data.
  void updateOdometry()
  {
    double dt = 0.1; // 100ms

    double current_vx, current_vy, current_omega;
    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      current_vx = vx_;
      current_vy = vy_;
      current_omega = omega_;
    }

    // Convert local velocities (vx, vy) to global frame using the current orientation (theta_)
    double delta_x = (current_vx * std::cos(theta_) - current_vy * std::sin(theta_)) * dt;
    double delta_y = (current_vx * std::sin(theta_) + current_vy * std::cos(theta_)) * dt;
    double delta_theta = current_omega * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Create and publish odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Convert yaw (theta_) to quaternion representation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = current_vx;
    odom_msg.twist.twist.linear.y = current_vy;
    odom_msg.twist.twist.angular.z = current_omega;

    odom_pub_->publish(odom_msg);
  }

  // Member variables for subscriptions, publisher, and timer.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables to store robot pose and encoder velocity
  double x_, y_, theta_;
  double vx_, vy_, omega_;
  std::mutex vel_mutex_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LuciaController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
