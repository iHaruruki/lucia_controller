#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// YARP
#include <yarp/os/all.h>

// 最大速度制限
constexpr double MAX_LINEAR_X  = 0.3;
constexpr double MAX_LINEAR_Y  = 0.3;
constexpr double MAX_ANGULAR_Z = 0.4;

// タイマ周期 (秒)
constexpr double LOOP_PERIOD = 0.05; // 50ms

// YARPポート（シンプルさ優先でグローバル）
yarp::os::BufferedPort<yarp::os::Bottle> g_cmd_port;
yarp::os::BufferedPort<yarp::os::Bottle> g_enc_port;

class RobotDriver : public rclcpp::Node
{
public:
  RobotDriver()
  : Node("robot_driver_min"),
    x_(0.0), y_(0.0), yaw_(0.0),
    cur_vx_(0.0), cur_vy_(0.0), cur_vth_(0.0),
    tgt_vx_(0.0), tgt_vy_(0.0), tgt_vth_(0.0),
    failure_count_(0), encoder_error_count_(0)
  {
    RCLCPP_INFO(get_logger(), "RobotDriver minimal started.");

    // オプション: シンプルな速度ランプを使うか
    declare_parameter<bool>("use_ramp", false);
    declare_parameter<double>("ramp_time_linear", 1.0);   // 目標速度までにかける秒 (簡易)
    declare_parameter<double>("ramp_time_angular", 1.0);

    // YARPネットワーク確認
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(3.0)){
      RCLCPP_FATAL(get_logger(), "YARP network unavailable");
      throw std::runtime_error("YARP network unavailable");
    }

    // ROS通信
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&RobotDriver::onCmdVel, this, std::placeholders::_1));

    // タイマ
    timer_ = create_wall_timer(
      std::chrono::duration<double>(LOOP_PERIOD),
      std::bind(&RobotDriver::onLoop, this));

    // YARPポート
    g_cmd_port.open("/robot_driver/command:o");
    g_enc_port.open("/robot_driver/encoder:i");

    bool ok_cmd = yarp::os::Network::connect("/robot_driver/command:o", "/vehicleDriver/remote:i");
    bool ok_enc = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/robot_driver/encoder:i");
    if(!ok_cmd) RCLCPP_WARN(get_logger(), "Failed to connect command port");
    if(!ok_enc) RCLCPP_WARN(get_logger(), "Failed to connect encoder port");

    last_time_ = now();
  }

  ~RobotDriver() override
  {
    g_cmd_port.close();
    g_enc_port.close();
  }

private:
  /* ===== cmd_vel コールバック ===== */
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    tgt_vx_  = std::clamp(msg->linear.x,  -MAX_LINEAR_X,  MAX_LINEAR_X);
    tgt_vy_  = std::clamp(msg->linear.y,  -MAX_LINEAR_Y,  MAX_LINEAR_Y);
    tgt_vth_ = std::clamp(msg->angular.z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
  }

  /* ===== メインループ ===== */
  void onLoop()
  {
    rclcpp::Time now_t = now();
    double dt = (now_t - last_time_).seconds();
    if (dt <= 0.0) dt = LOOP_PERIOD;
    last_time_ = now_t;

    updateCommand(dt);
    sendMotorCommand();
    readEncoderAndUpdate(dt, now_t);
  }

  /* ===== コマンド更新（任意ランプ） ===== */
  void updateCommand(double dt)
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    bool use_ramp = get_parameter("use_ramp").as_bool();
    if (!use_ramp) {
      cur_vx_  = tgt_vx_;
      cur_vy_  = tgt_vy_;
      cur_vth_ = tgt_vth_;
    } else {
      double ramp_lin = get_parameter("ramp_time_linear").as_double();
      double ramp_ang = get_parameter("ramp_time_angular").as_double();
      ramp_lin = std::max(0.01, ramp_lin);
      ramp_ang = std::max(0.01, ramp_ang);
      // 1秒あたり (tgt - cur)/ramp_time 分だけ近づく簡易ランプ
      cur_vx_  = approach(cur_vx_,  tgt_vx_,  dt / ramp_lin);
      cur_vy_  = approach(cur_vy_,  tgt_vy_,  dt / ramp_lin);
      cur_vth_ = approach(cur_vth_, tgt_vth_, dt / ramp_ang);
    }

    latest_cmd_[0] = cur_vx_;
    latest_cmd_[1] = cur_vy_;
    latest_cmd_[2] = cur_vth_;
    latest_cmd_[3] = 0.0;
  }

  double approach(double current, double target, double ratio)
  {
    ratio = std::clamp(ratio, 0.0, 1.0);
    return current + (target - current) * ratio;
  }

  /* ===== YARP 送信 ===== */
  void sendMotorCommand()
  {
    yarp::os::Bottle& b = g_cmd_port.prepare();
    b.clear();
    for (double v : latest_cmd_) b.addFloat64(v);
    g_cmd_port.write();
  }

  /* ===== エンコーダ読み取り + オドメトリ更新 ===== */
  void readEncoderAndUpdate(double dt, const rclcpp::Time& stamp)
  {
    // 非ブロッキング読み取り
    yarp::os::Bottle* enc = g_enc_port.read(false);
    if(!enc) {
      failure_count_++;
      if (failure_count_ % 50 == 0) {
        RCLCPP_WARN(get_logger(), "Encoder read failed (%d)", failure_count_);
      }
      publishOdometry(stamp, 0.0, 0.0, 0.0); // 読めない周期の扱いは用途に応じて変更
      return;
    }

    if (enc->size() < 3) {
      encoder_error_count_++;
      RCLCPP_WARN(get_logger(), "Encoder size too short (%d)", enc->size());
      return;
    }

    double vx  = enc->get(0).asFloat64();
    double vy  = enc->get(1).asFloat64();
    double vth = enc->get(2).asFloat64();

    if (invalidValue(vx) || invalidValue(vy) || invalidValue(vth)) {
      encoder_error_count_++;
      RCLCPP_WARN(get_logger(), "Invalid encoder value (NaN/Inf)");
      return;
    }

    // オドメトリ積分（(vx,vy) は機体座標系）
    integrate(vx, vy, vth, dt);
    publishOdometry(stamp, vx, vy, vth);
  }

  bool invalidValue(double v) const
  {
    return std::isnan(v) || std::isinf(v);
  }

  void integrate(double vx, double vy, double vth, double dt)
  {
    double c = std::cos(yaw_);
    double s = std::sin(yaw_);
    x_   += (vx * c - vy * s) * dt;
    y_   += (vx * s + vy * c) * dt;
    yaw_ += vth * dt;
    // -pi~pi 正規化
    if (yaw_ >  M_PI) yaw_ -= 2*M_PI;
    if (yaw_ < -M_PI) yaw_ += 2*M_PI;
  }

  /* ===== Odometry 出力 ===== */
  void publishOdometry(const rclcpp::Time& stamp, double vx, double vy, double vth)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,0,yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x  = vx;
    odom.twist.twist.linear.y  = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

private:
  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 時刻
  rclcpp::Time last_time_;

  // 座標・姿勢
  double x_, y_, yaw_;

  // 速度（現在値 / 目標値）
  double cur_vx_, cur_vy_, cur_vth_;
  double tgt_vx_, tgt_vy_, tgt_vth_;

  // 送信用
  double latest_cmd_[4] = {0,0,0,0};

  // 統計
  int failure_count_;
  int encoder_error_count_;

  // 排他
  std::mutex cmd_mutex_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}