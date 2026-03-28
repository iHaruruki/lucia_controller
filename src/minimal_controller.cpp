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
#include <yarp/os/all.h>

// 最大速度制限
constexpr double MAX_LINEAR_X  = 0.4;
constexpr double MAX_LINEAR_Y  = 0.4;
constexpr double MAX_ANGULAR_Z = 0.8;

// ループ周期 (秒)
constexpr double LOOP_PERIOD = 0.05;

// YARP port
yarp::os::BufferedPort<yarp::os::Bottle> g_cmd_port;
yarp::os::BufferedPort<yarp::os::Bottle> g_enc_port;

class RobotDriver : public rclcpp::Node
{
public:
  RobotDriver()
  : Node("robot_driver_min"),
    x_(0.0), y_(0.0), yaw_(0.0),
    tgt_vx_(0.0), tgt_vy_(0.0), tgt_vth_(0.0)
  {
    RCLCPP_INFO(get_logger(), "RobotDriver minimal started.");

    // YARPネットワーク確認
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(3.0)){
      RCLCPP_FATAL(get_logger(), "YARP network unavailable");
      throw std::runtime_error("YARP network unavailable");
    }

    // Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscriber
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&RobotDriver::onCmdVel, this, std::placeholders::_1));

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
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    tgt_vx_  = std::clamp(msg->linear.x,  -MAX_LINEAR_X,  MAX_LINEAR_X);
    tgt_vy_  = std::clamp(msg->linear.y,  -MAX_LINEAR_Y,  MAX_LINEAR_Y);
    tgt_vth_ = std::clamp(msg->angular.z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
  }

  void onLoop()
  {
    rclcpp::Time now_t = now();
    double dt = (now_t - last_time_).seconds();
    if (dt <= 0.0) dt = LOOP_PERIOD;
    last_time_ = now_t;

    sendMotorCommand();
    readEncoderAndUpdate(dt, now_t);
  }

  void sendMotorCommand()
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    yarp::os::Bottle& b = g_cmd_port.prepare();
    b.clear();
    b.addFloat64(tgt_vx_);
    b.addFloat64(tgt_vy_);
    b.addFloat64(tgt_vth_);
    g_cmd_port.write();
  }

  void readEncoderAndUpdate(double dt, const rclcpp::Time& stamp)
  {
    yarp::os::Bottle* enc = g_enc_port.read(false);
    if(!enc || enc->size() < 3) {
      return;
    }

    double vx  = enc->get(0).asFloat64();
    double vy  = enc->get(1).asFloat64();
    double vth = enc->get(2).asFloat64();

    integrate(vx, vy, vth, dt);
    publishOdometry(stamp, vx, vy, vth);
  }

  void integrate(double vx, double vy, double vth, double dt)
  {
    double c = std::cos(yaw_);
    double s = std::sin(yaw_);
    x_   += (vx * c - vy * s) * dt;
    y_   += (vx * s + vy * c) * dt;
    yaw_ += vth * dt;
    if (yaw_ >  M_PI) yaw_ -= 2*M_PI;
    if (yaw_ < -M_PI) yaw_ += 2*M_PI;
  }

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
    q.setRPY(0, 0, yaw_);
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_time_;
  double x_, y_, yaw_;
  double tgt_vx_, tgt_vy_, tgt_vth_;
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