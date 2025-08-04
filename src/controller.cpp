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

// YARPヘッダ
#include <yarp/os/all.h>

// 最大速度の定義（TurtleBot3参考値）
constexpr double MAX_LINEAR_X = 0.3;   // [m/s]
constexpr double MAX_LINEAR_Y = 0.3;   // [m/s]
constexpr double MAX_ANGULAR_Z = 0.4;  // [rad/s]

using namespace std::chrono_literals;

// YARPポートグローバル変数
yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; // モータ指令
yarp::os::BufferedPort<yarp::os::Bottle> p_enc; // エンコーダ読み取り

class RobotDriver : public rclcpp::Node
{
public:
  RobotDriver()
  : Node("robot_driver"),
    x_(0.0), y_(0.0), th_(0.0),
    current_linear_x_(0.0), current_linear_y_(0.0), current_angular_z_(0.0),
    target_linear_x_(0.0), target_linear_y_(0.0), target_angular_z_(0.0),
    failure_count_(0), encoder_error_count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "RobotDriver node started.");

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

    p_cmd.open("/robot_driver/command:o");
    p_enc.open("/robot_driver/encoder:i");

    bool cmd_connected = yarp::os::Network::connect("/robot_driver/command:o", "/vehicleDriver/remote:i");
    bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/robot_driver/encoder:i");
    if(!cmd_connected) RCLCPP_ERROR(this->get_logger(), "Failed to connect command port");
    if(!enc_connected) RCLCPP_ERROR(this->get_logger(), "Failed to connect encoder port");
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
    target_linear_x_ = std::clamp(msg->linear.x, -MAX_LINEAR_X, MAX_LINEAR_X);
    target_linear_y_ = std::clamp(msg->linear.y, -MAX_LINEAR_Y, MAX_LINEAR_Y);
    target_angular_z_ = std::clamp(msg->angular.z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
  }

  void timerCallback()
  {
    rclcpp::Time now = this->get_clock()->now();
    static rclcpp::Time last_time = now;
    double dt = (now - last_time).seconds();

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      current_linear_x_ = updateSpeed(current_linear_x_, target_linear_x_, MAX_LINEAR_X / 2.0);
      current_linear_y_ = updateSpeed(current_linear_y_, target_linear_y_, MAX_LINEAR_Y / 2.0);
      current_angular_z_ = updateSpeed(current_angular_z_, target_angular_z_, MAX_ANGULAR_Z / 2.0);

      latest_cmd_[0] = current_linear_x_;
      latest_cmd_[1] = current_linear_y_;
      latest_cmd_[2] = current_angular_z_;
      latest_cmd_[3] = 0.0;
    }

    // YARPモータ指令送信
    yarp::os::Bottle& cmd_bottle = p_cmd.prepare();
    cmd_bottle.clear();
    for (const auto& val : latest_cmd_) {
      cmd_bottle.addFloat64(val);
    }
    p_cmd.write();

    // YARPエンコーダ受信
    yarp::os::Bottle* enc_bottle = p_enc.read(false);
    if(enc_bottle != nullptr)
    {
      // --- エラーチェック: データ長の確認 ---
      if(enc_bottle->size() < 3) {
        encoder_error_count_++;
        RCLCPP_ERROR(this->get_logger(), "Encoder data too short (size=%ld), error count: %d",
                     enc_bottle->size(), encoder_error_count_);
        last_time = now;
        return;
      }

      // --- エラーチェック: NaN/Inf含み ---
      std::vector<double> enc(4, 0.0);
      bool invalid_value = false;
      for(size_t i=0; i<enc.size(); ++i) {
        double val = enc_bottle->get(i).asFloat64();
        if(std::isnan(val) || std::isinf(val)) {
          invalid_value = true;
          encoder_error_count_++;
          RCLCPP_ERROR(this->get_logger(),
                       "Encoder value invalid: index=%ld, value=%f, error count: %d",
                       i, val, encoder_error_count_);
        }
        enc[i] = val;
      }
      if(invalid_value) {
        last_time = now;
        return;
      }

      double vx = enc[0];
      double vy = enc[1];
      double vth = enc[2];

      x_ += (vx * cos(th_) - vy * sin(th_)) * dt;
      y_ += (vx * sin(th_) + vy * cos(th_)) * dt;
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
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;
      odom_pub_->publish(odom);

      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = now;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_footprint";
      odom_tf.transform.translation.x = x_;
      odom_tf.transform.translation.y = y_;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(odom_tf);
    }
    else
    {
      failure_count_++;
      RCLCPP_WARN(this->get_logger(), "Encoder read failed: %d times", failure_count_);
    }
    last_time = now;
  }

  double updateSpeed(double current, double target, double max_acc)
  {
    double step = max_acc * 0.05; // 50ms timer
    if(current < target) {
      current += step;
      if(current > target) current = target;
    } else if(current > target) {
      current -= step;
      if(current < target) current = target;
    }
    return current;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, th_;
  double latest_cmd_[4] = {0.0, 0.0, 0.0, 0.0};
  int failure_count_;
  int encoder_error_count_;

  double current_linear_x_, current_linear_y_, current_angular_z_;
  double target_linear_x_, target_linear_y_, target_angular_z_;
  std::mutex cmd_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}