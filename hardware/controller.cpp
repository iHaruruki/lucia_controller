// robot_driver.cpp
#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// YARPヘッダ
#include <yarp/os/all.h>

// 最大速度の定義
#define MAX_LINEAR 0.2   // [m/s]
#define MAX_ANGULAR 0.4  // [rad/s]

using namespace std::chrono_literals;

// YARPポートグローバル変数
yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; // モータ指令
yarp::os::BufferedPort<yarp::os::Bottle> p_enc; // エンコーダ読み取り

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("odom_publisher"),
    x_(0.0), y_(0.0), th_(0.0),
    current_linear_x_(0.0), current_linear_y_(0.0), current_angular_z_(0.0),
    target_linear_x_(0.0), target_linear_y_(0.0), target_angular_z_(0.0),
    a_max_linear_(MAX_LINEAR / 1.0),
    a_max_angular_(MAX_ANGULAR / 1.0),
    failure_count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "OdomPublisher node is starting...");

    // YARPネットワークの初期化（5秒以内に接続できるか確認）
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(5.0)){
      RCLCPP_ERROR(this->get_logger(), "YARP network is not available");
      throw std::runtime_error("YARP network is not available");
    }

    // ROS2のPublisher, Subscriber, Timerの設定
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&OdomPublisher::velocity_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(30ms, std::bind(&OdomPublisher::timer_callback, this));

    // YARPポートをオープンし、接続（YARP側のポート名は実システムに合わせて変更）
    p_cmd.open("/remoteController/command:o");      // モータ指令
    p_enc.open("/remoteController/encoder:i");        // エンコーダ読み取り

    bool cmd_connected = yarp::os::Network::connect("/remoteController/command:o", "/vehicleDriver/remote:i");
    bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/encoder:i");

    if(!cmd_connected){
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to /vehicleDriver/remote:i");
    }
    if(!enc_connected){
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to /vehicleDriver/encoder:o");
    }
  }

  ~OdomPublisher()
  {
    p_cmd.close();
    p_enc.close();
  }

private:
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 指令値の制限
    double cmd_linear_x = std::clamp(msg->linear.x, -MAX_LINEAR, MAX_LINEAR);
    double cmd_linear_y = std::clamp(msg->linear.y, -MAX_LINEAR, MAX_LINEAR);
    double cmd_angular_z = std::clamp(msg->angular.z, -MAX_ANGULAR, MAX_ANGULAR);

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      target_linear_x_ = cmd_linear_x;
      target_linear_y_ = cmd_linear_y;
      target_angular_z_ = cmd_angular_z;
    }
  }

  void timer_callback()
  {
    // 現在時刻の取得
    rclcpp::Time current_time = this->get_clock()->now();
    static rclcpp::Time last_time = current_time;
    double dt = (current_time - last_time).seconds();

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      // 現在の速度を目標に向けて加速・減速を反映
      current_linear_x_ = update_speed(current_linear_x_, target_linear_x_, a_max_linear_);
      current_linear_y_ = update_speed(current_linear_y_, target_linear_y_, a_max_linear_);
      current_angular_z_ = update_speed(current_angular_z_, target_angular_z_, a_max_angular_);

      latest_cmd_[0] = current_linear_x_;
      latest_cmd_[1] = current_linear_y_;
      latest_cmd_[2] = current_angular_z_;
      latest_cmd_[3] = 0.00;
    }

    // YARP経由でモータ指令を送信
    yarp::os::Bottle& bc = p_cmd.prepare();
    bc.clear();
    for(const auto& c : latest_cmd_){
      bc.addFloat64(c);
    }
    p_cmd.write();

    // YARP経由でエンコーダデータを読み取る（ノンブロッキング）
    yarp::os::Bottle* bt = p_enc.read(false);
    if(bt != nullptr)
    {
      std::vector<double> enc(4);
      for(size_t i = 0; i < enc.size(); i++){
        enc[i] = bt->get(i).asFloat64();
      }

      // 取得したエンコーダ値からオドメトリ計算（例：vx, vy, w）
      double vx = enc[0];
      double vy = enc[1];
      double w  = enc[2];
      // enc[3]は使用しない（例：ta）

      // 単純な運動モデル（実システムに合わせて補正が必要）
      x_ += (vx * cos(th_) - vy * sin(th_)) * dt;
      y_ += (vx * sin(th_) + vy * cos(th_)) * dt;
      th_ += w * dt;

      // オドメトリメッセージの作成と発行
      auto odom = nav_msgs::msg::Odometry();
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      tf2::Quaternion odom_q;
      odom_q.setRPY(0, 0, th_);
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf2::toMsg(odom_q);

      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = w;
      odom_publisher_->publish(odom);

      // TFブロードキャスト（odom→base_link）
      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x_;
      odom_trans.transform.translation.y = y_;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom.pose.pose.orientation;
      odom_broadcaster_->sendTransform(odom_trans);
    }
    else
    {
      failure_count_++;
      RCLCPP_WARN(this->get_logger(), "Failed to read encoder data count: %d", failure_count_);
    }
    last_time = current_time;
  }

  // 急発進・急停止を避けるため、比例制御
  double update_speed(double current, double target, double max_acc)
  {
    double step = max_acc * 0.03; // dt=30msとして
    if(current < target)
    {
      current += step;
      if(current > target)
        current = target;
    }
    else if(current > target)
    {
      current -= step;
      if(current < target)
        current = target;
    }
    return current;
  }

  // ROS2関連メンバ
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ロボット位置・姿勢
  double x_, y_, th_;
  // 最新のモータ指令値（配列のサイズはシステムに合わせる）
  double latest_cmd_[4] = {0.0, 0.0, 0.0, 0.0};
  int failure_count_;

  // 現在の速度と目標速度
  double current_linear_x_, current_linear_y_, current_angular_z_;
  double target_linear_x_, target_linear_y_, target_angular_z_;
  double a_max_linear_, a_max_angular_;
  std::mutex cmd_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
