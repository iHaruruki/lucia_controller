#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <yarp/os/all.h>

using namespace std::chrono_literals;

static inline double normalizeYaw(double a) {
  double w = std::fmod(a + M_PI, 2.0 * M_PI);
  if (w <= 0) w += 2.0 * M_PI;
  return w - M_PI;
}

class RobotDriver : public rclcpp::Node
{
public:
  RobotDriver()
  : Node("robot_driver"),
    x_(0.0), y_(0.0), th_(0.0),
    failure_count_(0), encoder_error_count_(0),
    last_time_steady_(std::chrono::steady_clock::now()),
    v_filter_{0.0,0.0,0.0}, first_filter_(true)
  {
    declare_parameter<bool>("use_raw_cmd_vel", false);
    declare_parameter<double>("safety_scale_linear", 1.08);   // smoother の上限より僅かに大きい
    declare_parameter<double>("safety_scale_angular", 1.08);
    declare_parameter<double>("nominal_max_linear_x", 0.30);
    declare_parameter<double>("nominal_max_linear_y", 0.30);
    declare_parameter<double>("nominal_max_angular_z", 0.40);
    declare_parameter<double>("encoder_lowpass_fc", 5.0);     // Hz
    declare_parameter<double>("outlier_linear_gain", 1.6);    // nominal * gain
    declare_parameter<double>("outlier_angular_gain", 2.0);
    declare_parameter<double>("cmd_timeout", 0.5);

    get_parameter("use_raw_cmd_vel", use_raw_cmd_vel_);
    get_parameter("safety_scale_linear", safety_scale_lin_);
    get_parameter("safety_scale_angular", safety_scale_ang_);
    get_parameter("nominal_max_linear_x", nominal_max_lin_x_);
    get_parameter("nominal_max_linear_y", nominal_max_lin_y_);
    get_parameter("nominal_max_angular_z", nominal_max_ang_z_);
    get_parameter("encoder_lowpass_fc", encoder_fc_);
    get_parameter("outlier_linear_gain", outlier_lin_gain_);
    get_parameter("outlier_angular_gain", outlier_ang_gain_);
    get_parameter("cmd_timeout", cmd_timeout_);

    safety_max_lin_x_ = nominal_max_lin_x_ * safety_scale_lin_;
    safety_max_lin_y_ = nominal_max_lin_y_ * safety_scale_lin_;
    safety_max_ang_z_ = nominal_max_ang_z_ * safety_scale_ang_;

    RCLCPP_INFO(get_logger(),
      "RobotDriver starting. use_raw_cmd_vel=%d safety_max: (%.3f, %.3f, %.3f)",
      use_raw_cmd_vel_, safety_max_lin_x_, safety_max_lin_y_, safety_max_ang_z_);

    yarp::os::Network yarp;
    if (!yarp.checkNetwork(5.0)) {
      RCLCPP_FATAL(get_logger(), "YARP network unavailable");
      throw std::runtime_error("YARP network unavailable");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    std::string cmd_topic = use_raw_cmd_vel_ ? "/cmd_vel_raw" : "/cmd_vel";
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, rclcpp::QoS(10),
      std::bind(&RobotDriver::cmdCallback, this, std::placeholders::_1));

    // Timer (20 ms = 50 Hz)
    timer_ = create_wall_timer(20ms, std::bind(&RobotDriver::onTimer, this));
    last_cmd_time_ = now();

    // YARP ports
    port_cmd_.open("/robot_driver/command:o");
    port_enc_.open("/robot_driver/encoder:i");
    bool cmd_conn = yarp::os::Network::connect("/robot_driver/command:o", "/vehicleDriver/remote:i");
    bool enc_conn = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/robot_driver/encoder:i");
    if(!cmd_conn) RCLCPP_WARN(get_logger(), "Command port not yet connected.");
    if(!enc_conn) RCLCPP_WARN(get_logger(), "Encoder port not yet connected.");
  }

  ~RobotDriver() override {
    port_cmd_.close();
    port_enc_.close();
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    // Safety clamp (広め)。ここで加速度等はもう制限しない。
    latest_cmd_[0] = std::clamp(msg->linear.x, -safety_max_lin_x_, safety_max_lin_x_);
    latest_cmd_[1] = std::clamp(msg->linear.y, -safety_max_lin_y_, safety_max_lin_y_);
    latest_cmd_[2] = std::clamp(msg->angular.z, -safety_max_ang_z_, safety_max_ang_z_);
    latest_cmd_[3] = 0.0;
    last_cmd_time_ = now();
  }

  bool filterEncoder(double &vx, double &vy, double &vth, double dt)
  {
    // Outlier guard
    double out_lin_x = nominal_max_lin_x_ * outlier_lin_gain_;
    double out_lin_y = nominal_max_lin_y_ * outlier_lin_gain_;
    double out_ang   = nominal_max_ang_z_ * outlier_ang_gain_;
    if (std::fabs(vx) > out_lin_x || std::fabs(vy) > out_lin_y || std::fabs(vth) > out_ang) {
      encoder_error_count_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Encoder outlier rejected (%.3f, %.3f, %.3f)", vx, vy, vth);
      return false;
    }

    if (dt <= 0) return false;
    double alpha = 1.0 - std::exp(-2.0 * M_PI * encoder_fc_ * dt);
    alpha = std::clamp(alpha, 0.0, 1.0);

    if (first_filter_) {
      v_filter_[0] = vx; v_filter_[1] = vy; v_filter_[2] = vth;
      first_filter_ = false;
    } else {
      v_filter_[0] = alpha * vx  + (1 - alpha) * v_filter_[0];
      v_filter_[1] = alpha * vy  + (1 - alpha) * v_filter_[1];
      v_filter_[2] = alpha * vth + (1 - alpha) * v_filter_[2];
    }

    vx = v_filter_[0];
    vy = v_filter_[1];
    vth = v_filter_[2];
    return true;
  }

  void integrate(double vx, double vy, double vth, double dt)
  {
    double theta_mid = th_ + 0.5 * vth * dt;
    double c = std::cos(theta_mid);
    double s = std::sin(theta_mid);
    x_  += (vx * c - vy * s) * dt;
    y_  += (vx * s + vy * c) * dt;
    th_  = normalizeYaw(th_ + vth * dt);
  }

  void publishOdom(const rclcpp::Time &stamp, double vx, double vy, double vth, double dt)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,th_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    double a_lin = 0.002;
    double b_lin = 0.010;
    double a_ang = 0.001;
    double b_ang = 0.020;
    double speed_lin = std::hypot(vx, vy);
    double sx = a_lin + b_lin * speed_lin;
    double sy = sx;
    double sth = a_ang + b_ang * std::fabs(vth);

    for (double &c : odom.pose.covariance) c = 0.0;
    odom.pose.covariance[0]  = sx*sx;
    odom.pose.covariance[7]  = sy*sy;
    odom.pose.covariance[35] = sth*sth;

    for (double &c : odom.twist.covariance) c = 0.0;
    double inv_dt = (dt > 1e-6) ? (1.0/dt) : 0.0;
    odom.twist.covariance[0]  = (sx * inv_dt) * (sx * inv_dt);
    odom.twist.covariance[7]  = (sy * inv_dt) * (sy * inv_dt);
    odom.twist.covariance[35] = (sth * inv_dt) * (sth * inv_dt);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = odom.child_frame_id;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void onTimer()
  {
    auto now_steady = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now_steady - last_time_steady_).count();
    if (dt <= 0) dt = 1e-3;
    last_time_steady_ = now_steady;

    rclcpp::Time stamp = now();

    // Timeout: smoother の velocity_timeout より少し大きい (cmd_timeout_)
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      if ((stamp - last_cmd_time_).seconds() > cmd_timeout_) {
        for (double &c : latest_cmd_) c = 0.0;
      }
    }

    // Send command (as-is; no smoothing here)
    double cmd_copy[4];
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      for (int i=0;i<4;++i) cmd_copy[i] = latest_cmd_[i];
    }
    auto &bot = port_cmd_.prepare();
    bot.clear();
    for (double v : cmd_copy) bot.addFloat64(v);
    port_cmd_.write(false);

    // Read encoder
    yarp::os::Bottle *enc = port_enc_.read(false);
    if (!enc) {
      failure_count_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Encoder miss count=%d", failure_count_);
      return;
    }
    if (enc->size() < 3) {
      encoder_error_count_++;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Encoder short size=%ld err=%d",
                            enc->size(), encoder_error_count_);
      return;
    }

    double vx = enc->get(0).asFloat64();
    double vy = enc->get(1).asFloat64();
    double vth = enc->get(2).asFloat64();
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vth)) {
      encoder_error_count_++;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Encoder NaN/Inf err=%d", encoder_error_count_);
      return;
    }

    if (!filterEncoder(vx, vy, vth, dt)) {
      return;
    }

    integrate(vx, vy, vth, dt);
    publishOdom(stamp, vx, vy, vth, dt);

    if (++perf_counter_ % 50 == 0) {
      double ex = vx - cmd_copy[0];
      double ey = vy - cmd_copy[1];
      double eω = vth - cmd_copy[2];
      RCLCPP_DEBUG(get_logger(), "Perf: lin_err=%.3f ang_err=%.3f",
                   std::hypot(ex, ey), std::fabs(eω));
    }
  }

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  double x_, y_, th_;
  double latest_cmd_[4] {0,0,0,0};
  int failure_count_;
  int encoder_error_count_;
  int perf_counter_ = 0;

  std::mutex cmd_mtx_;
  rclcpp::Time last_cmd_time_;

  // Timing
  std::chrono::steady_clock::time_point last_time_steady_;

  // Encoder filter
  std::array<double,3> v_filter_;
  bool first_filter_;
  double encoder_fc_;

  // Params
  bool use_raw_cmd_vel_;
  double safety_scale_lin_, safety_scale_ang_;
  double nominal_max_lin_x_, nominal_max_lin_y_, nominal_max_ang_z_;
  double safety_max_lin_x_, safety_max_lin_y_, safety_max_ang_z_;
  double outlier_lin_gain_, outlier_ang_gain_;
  double cmd_timeout_;

  // YARP
  yarp::os::BufferedPort<yarp::os::Bottle> port_cmd_;
  yarp::os::BufferedPort<yarp::os::Bottle> port_enc_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}