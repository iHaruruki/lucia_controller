#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <yarp/os/all.h>

// 最大速度制限 (nav2_velocity_smoother と整合)
constexpr double MAX_LINEAR_X = 0.3;   // [m/s]
constexpr double MAX_LINEAR_Y = 0.3;   // [m/s]
constexpr double MAX_ANGULAR_Z = 0.4;  // [rad/s]

// コマンド未受信タイムアウト
constexpr double CMD_TIMEOUT = 0.5; // [s]

using namespace std::chrono_literals;

// YARP ポート
yarp::os::BufferedPort<yarp::os::Bottle> p_cmd;
yarp::os::BufferedPort<yarp::os::Bottle> p_enc;

class RobotDriver : public rclcpp::Node
{
public:
  RobotDriver()
  : Node("robot_driver"),
    x_(0.0), y_(0.0), th_(0.0),
    failure_count_(0), encoder_error_count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "RobotDriver started. Smoothing is external (nav2_velocity_smoother).");

    yarp::os::Network yarp;
    if(!yarp.checkNetwork(5.0)){
      RCLCPP_ERROR(this->get_logger(), "YARP network unavailable");
      throw std::runtime_error("YARP network unavailable");
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&RobotDriver::cmdVelCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&RobotDriver::timerCallback, this));
    last_cmd_time_ = this->get_clock()->now();

    // YARP ポート
    p_cmd.open("/robot_driver/command:o");
    p_enc.open("/robot_driver/encoder:i");

    bool cmd_connected = yarp::os::Network::connect("/robot_driver/command:o", "/vehicleDriver/remote:i");
    bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/robot_driver/encoder:i");
    if(!cmd_connected) RCLCPP_WARN(this->get_logger(), "Command port not connected yet.");
    if(!enc_connected) RCLCPP_WARN(this->get_logger(), "Encoder port not connected yet.");
  }

  ~RobotDriver()
  {
    p_cmd.close();
    p_enc.close();
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    // 既に平滑化済み想定。安全のためクリップのみ。
    latest_cmd_[0] = std::clamp(msg->linear.x, -MAX_LINEAR_X, MAX_LINEAR_X);
    latest_cmd_[1] = std::clamp(msg->linear.y, -MAX_LINEAR_Y, MAX_LINEAR_Y);
    latest_cmd_[2] = std::clamp(msg->angular.z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
    latest_cmd_[3] = 0.0;
    last_cmd_time_ = this->get_clock()->now();
  }

  void timerCallback()
  {
    rclcpp::Time now = this->get_clock()->now();
    static rclcpp::Time last_time = now;
    double dt = (now - last_time).seconds();
    last_time = now;

    // タイムアウトで停止
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if ((now - last_cmd_time_).seconds() > CMD_TIMEOUT)
      {
        latest_cmd_[0] = 0.0;
        latest_cmd_[1] = 0.0;
        latest_cmd_[2] = 0.0;
        latest_cmd_[3] = 0.0;
      }
    }

    // コマンドコピーして送信
    double cmd_copy[4];
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      for (int i=0;i<4;++i) cmd_copy[i] = latest_cmd_[i];
    }

    yarp::os::Bottle & cmd_bottle = p_cmd.prepare();
    cmd_bottle.clear();
    for (double v : cmd_copy) {
      cmd_bottle.addFloat64(v);
    }
    p_cmd.write(false);

    // エンコーダ読み取り
    yarp::os::Bottle* enc_bottle = p_enc.read(false);
    if (enc_bottle != nullptr)
    {
      if (enc_bottle->size() < 3) {
        encoder_error_count_++;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Encoder data too short size=%ld err=%d",
                              enc_bottle->size(), encoder_error_count_);
        return;
      }

      double vx = enc_bottle->get(0).asFloat64();
      double vy = enc_bottle->get(1).asFloat64();
      double vth = enc_bottle->get(2).asFloat64();
      if (std::isnan(vx) || std::isnan(vy) || std::isnan(vth) ||
          std::isinf(vx) || std::isinf(vy) || std::isinf(vth)) {
        encoder_error_count_++;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Invalid encoder value err=%d", encoder_error_count_);
        return;
      }

      // 速度からオドメトリ積分（全方向移動モデル）
      x_  += (vx * std::cos(th_) - vy * std::sin(th_)) * dt;
      y_  += (vx * std::sin(th_) + vy * std::cos(th_)) * dt;
      th_ += vth * dt;

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";

      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, th_);
      odom.pose.pose.orientation = tf2::toMsg(q);

      odom.twist.twist.linear.x  = vx;
      odom.twist.twist.linear.y  = vy;
      odom.twist.twist.angular.z = vth;

      odom_pub_->publish(odom);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = now;
      tf_msg.header.frame_id = "odom";
      tf_msg.child_frame_id = "base_footprint";
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
    else
    {
      failure_count_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Encoder read failed count=%d", failure_count_);
    }
  }

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 状態
  double x_, y_, th_;
  double latest_cmd_[4] = {0.0, 0.0, 0.0, 0.0};
  int failure_count_;
  int encoder_error_count_;
  std::mutex cmd_mutex_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}