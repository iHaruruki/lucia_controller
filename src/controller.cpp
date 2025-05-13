#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class LuciaController : public rclcpp::Node
{
public:
  LuciaController()
  : Node("lucia_controller"), x_(0.0), y_(0.0), theta_(0.0),
    vx_(0.0), vy_(0.0), omega_(0.0)
  {
    // cmd_velサブスクライバーの作成（制御入力用、例としてログ出力）
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&LuciaController::cmdVelCallback, this, std::placeholders::_1));

    // エンコーダからの速度情報を受け取るサブスクライバーの作成（geometry_msgs/msg/Twistを使用）
    encoder_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "encoder", 10,
      std::bind(&LuciaController::encoderCallback, this, std::placeholders::_1));

    // /odomパブリッシャーの作成（出力はnav_msgs/msg/Odometry）
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // オドメトリの更新用タイマー（100ms毎に更新）
    timer_ = this->create_wall_timer(
      100ms, std::bind(&LuciaController::updateOdometry, this));
  }

private:
  // cmd_velのコールバック：制御入力として受信、ここでは単にログ出力
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f angular=%.2f",
                msg->linear.x, msg->angular.z);
  }

  // エンコーダからのデータのコールバック：vx, vy, omegaを更新
  void encoderCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    omega_ = msg->angular.z;

    RCLCPP_INFO(this->get_logger(), "Encoder data: vx=%.2f, vy=%.2f, omega=%.2f",
                vx_, vy_, omega_);
  }

  // オドメトリの更新（エンコーダからの値を統合する）
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

    // エンコーダ計測値(vx, vy)はロボットの局所座標系における速度
    // これを地図座標系に変換して更新
    double delta_x = (current_vx * std::cos(theta_) - current_vy * std::sin(theta_)) * dt;
    double delta_y = (current_vx * std::sin(theta_) + current_vy * std::cos(theta_)) * dt;
    double delta_theta = current_omega * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // オドメトリメッセージの作成
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // 現在のyaw角からクォータニオンを生成
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // オドメトリの速度情報にエンコーダからの速度を設定
    odom_msg.twist.twist.linear.x = current_vx;
    odom_msg.twist.twist.linear.y = current_vy;
    odom_msg.twist.twist.angular.z = current_omega;

    odom_pub_->publish(odom_msg);
  }

  // メンバ変数
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ロボットのグローバル位置と姿勢
  double x_, y_, theta_;
  // エンコーダからの速度情報（局所座標系）
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
