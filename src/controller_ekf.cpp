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

constexpr double MAX_LINEAR_X = 0.4;
constexpr double MAX_LINEAR_Y = 0.4;
constexpr double MAX_ANGULAR_Z = 0.8;
constexpr double LOOP_PERIOD = 0.05;

// YARP ポート（簡略化のためグローバル）
yarp::os::BufferedPort<yarp::os::Bottle> g_cmd_port;
yarp::os::BufferedPort<yarp::os::Bottle> g_enc_port;

class RobotDriver : public rclcpp::Node {
public:
    RobotDriver()
    : Node("robot_driver_min"),
      x_(0.0), y_(0.0), yaw_(0.0),
      cur_vx_(0.0), cur_vy_(0.0), cur_vth_(0.0),
      tgt_vx_(0.0), tgt_vy_(0.0), tgt_vth_(0.0),
      filt_tgt_vx_(0.0), filt_tgt_vy_(0.0), filt_tgt_vth_(0.0),
      failure_count_(0), encoder_error_count_(0)
    {
        RCLCPP_INFO(get_logger(), "RobotDriver (raw wheel odometry mode) started.");

        // パラメータ
        declare_parameter<bool>("use_smoothing", true);
        declare_parameter<double>("smoothing_tau_linear", 0.3);
        declare_parameter<double>("smoothing_tau_angular", 0.2);
        declare_parameter<bool>("use_ramp", true);
        declare_parameter<double>("ramp_time_linear", 0.6);
        declare_parameter<double>("ramp_time_angular", 0.6);

        // raw wheel odom 用
        declare_parameter<std::string>("raw_odom_topic", "wheel_odom");
        declare_parameter<std::string>("raw_odom_frame", "wheel_odom");
        declare_parameter<std::string>("base_link_frame", "base_link");
        declare_parameter<bool>("publish_tf", false); // robot_localization に任せるので通常 false

        // YARP ネットワーク
        yarp::os::Network yarp;
        if(!yarp.checkNetwork(3.0)){
            RCLCPP_FATAL(get_logger(), "YARP network unavailable");
            throw std::runtime_error("YARP network unavailable");
        }

        // ROS 通信
        std::string raw_topic = get_parameter("raw_odom_topic").as_string();
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(raw_topic, 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobotDriver::onCmdVel, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(
            std::chrono::duration<double>(LOOP_PERIOD),
            std::bind(&RobotDriver::onLoop, this)
        );

        // YARP ポート
        g_cmd_port.open("/robot_driver/command:o");
        g_enc_port.open("/robot_driver/encoder:i");

        bool ok_cmd = yarp::os::Network::connect("/robot_driver/command:o", "/vehicleDriver/remote:i");
        bool ok_enc = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/robot_driver/encoder:i");
        if(!ok_cmd) RCLCPP_WARN(get_logger(), "Failed to connect command port");
        if(!ok_enc) RCLCPP_WARN(get_logger(), "Failed to connect encoder port");

        last_time_ = now();
    }

    ~RobotDriver() override {
        g_cmd_port.close();
        g_enc_port.close();
    }

private:
    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        tgt_vx_  = std::clamp(msg->linear.x,  -MAX_LINEAR_X, MAX_LINEAR_X);
        tgt_vy_  = std::clamp(msg->linear.y,  -MAX_LINEAR_Y, MAX_LINEAR_Y);
        tgt_vth_ = std::clamp(msg->angular.z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
    }

    void onLoop() {
        rclcpp::Time now_t = now();
        double dt = (now_t - last_time_).seconds();
        if (dt <= 0.0) dt = LOOP_PERIOD;
        last_time_ = now_t;

        updateCommand(dt);
        sendMotorCommand();
        readEncoderAndUpdate(dt, now_t);
    }

    void updateCommand(double dt) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        bool use_smoothing = get_parameter("use_smoothing").as_bool();
        bool use_ramp      = get_parameter("use_ramp").as_bool();

        double raw_vx  = tgt_vx_;
        double raw_vy  = tgt_vy_;
        double raw_vth = tgt_vth_;

        if (use_smoothing) {
            double tau_lin = std::max(1e-4, get_parameter("smoothing_tau_linear").as_double());
            double tau_ang = std::max(1e-4, get_parameter("smoothing_tau_angular").as_double());
            double alpha_lin = dt / (tau_lin + dt);
            double alpha_ang = dt / (tau_ang + dt);
            filt_tgt_vx_  += alpha_lin * (raw_vx  - filt_tgt_vx_);
            filt_tgt_vy_  += alpha_lin * (raw_vy  - filt_tgt_vy_);
            filt_tgt_vth_ += alpha_ang * (raw_vth - filt_tgt_vth_);
        } else {
            filt_tgt_vx_  = raw_vx;
            filt_tgt_vy_  = raw_vy;
            filt_tgt_vth_ = raw_vth;
        }

        if (!use_ramp) {
            cur_vx_  = filt_tgt_vx_;
            cur_vy_  = filt_tgt_vy_;
            cur_vth_ = filt_tgt_vth_;
        } else {
            double rtl = std::max(0.01, get_parameter("ramp_time_linear").as_double());
            double rta = std::max(0.01, get_parameter("ramp_time_angular").as_double());
            double ratio_lin = std::clamp(dt / rtl, 0.0, 1.0);
            double ratio_ang = std::clamp(dt / rta, 0.0, 1.0);
            cur_vx_  += (filt_tgt_vx_  - cur_vx_)  * ratio_lin;
            cur_vy_  += (filt_tgt_vy_  - cur_vy_)  * ratio_lin;
            cur_vth_ += (filt_tgt_vth_ - cur_vth_) * ratio_ang;
        }

        latest_cmd_[0] = cur_vx_;
        latest_cmd_[1] = cur_vy_;
        latest_cmd_[2] = cur_vth_;
        latest_cmd_[3] = 0.0;
    }

    void sendMotorCommand() {
        yarp::os::Bottle& b = g_cmd_port.prepare();
        b.clear();
        for (double v : latest_cmd_) b.addFloat64(v);
        g_cmd_port.write();
    }

    void readEncoderAndUpdate(double dt, const rclcpp::Time& stamp) {
        yarp::os::Bottle* enc = g_enc_port.read(false);
        if(!enc) {
            failure_count_++;
            if (failure_count_ % 50 == 0) {
                RCLCPP_WARN(get_logger(), "Encoder read failed (%d)", failure_count_);
            }
            // 読めなかった周期はゼロ速度とみなす（必要なら保持）
            publishOdometry(stamp, 0.0, 0.0, 0.0);
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

        if (std::isnan(vx) || std::isnan(vy) || std::isnan(vth) ||
            std::isinf(vx) || std::isinf(vy) || std::isinf(vth)) {
            encoder_error_count_++;
            RCLCPP_WARN(get_logger(), "Invalid encoder value (NaN/Inf)");
            return;
        }

        integrate(vx, vy, vth, dt);
        publishOdometry(stamp, vx, vy, vth);
    }

    void integrate(double vx, double vy, double vth, double dt) {
        double c = std::cos(yaw_);
        double s = std::sin(yaw_);
        x_   += (vx * c - vy * s) * dt;
        y_   += (vx * s + vy * c) * dt;
        yaw_ += vth * dt;
        if (yaw_ >  M_PI) yaw_ -= 2*M_PI;
        if (yaw_ < -M_PI) yaw_ += 2*M_PI;
    }

    void publishOdometry(const rclcpp::Time& stamp, double vx, double vy, double vth) {
        nav_msgs::msg::Odometry odom;
        std::string odom_frame = get_parameter("raw_odom_frame").as_string(); // 例: wheel_odom
        std::string base_frame = get_parameter("base_link_frame").as_string(); // 例: base_link
        bool publish_tf = get_parameter("publish_tf").as_bool();

        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame;
        odom.child_frame_id  = base_frame;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // ---- Covariance 設定 ----
        // pose: [x y z r p y]
        for (double &c : odom.pose.covariance) c = 0.0;
        double pos_var = 0.05 * 0.05; // 5cm
        double yaw_std = 2.0 * M_PI / 180.0;
        double yaw_var = yaw_std * yaw_std;

        odom.pose.covariance[0]  = pos_var; // x
        odom.pose.covariance[7]  = pos_var; // y
        odom.pose.covariance[35] = yaw_var; // yaw

        // 未使用軸を大きな分散
        odom.pose.covariance[14] = 9999.0; // z
        odom.pose.covariance[21] = 9999.0; // roll
        odom.pose.covariance[28] = 9999.0; // pitch

        // twist: [vx vy vz vroll vpitch vyaw]
        for (double &c : odom.twist.covariance) c = 0.0;
        double v_lin_var = 0.02 * 0.02; // 2cm/s
        double v_yaw_std = 2.0 * M_PI / 180.0;
        double v_yaw_var = v_yaw_std * v_yaw_std;

        odom.twist.covariance[0]  = v_lin_var; // vx
        odom.twist.covariance[7]  = v_lin_var; // vy
        odom.twist.covariance[35] = v_yaw_var; // wz

        odom.twist.covariance[14] = 9999.0; // vz
        odom.twist.covariance[21] = 9999.0; // vroll
        odom.twist.covariance[28] = 9999.0; // vpitch

        odom_pub_->publish(odom);

        // raw センサなので通常 TF は送らない
        if (publish_tf) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = odom_frame;
            tf.child_frame_id  = base_frame;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_time_;

    // 状態
    double x_, y_, yaw_;
    double cur_vx_, cur_vy_, cur_vth_;
    double tgt_vx_, tgt_vy_, tgt_vth_;
    double filt_tgt_vx_, filt_tgt_vy_, filt_tgt_vth_;
    double latest_cmd_[4] = {0,0,0,0};

    int failure_count_;
    int encoder_error_count_;

    std::mutex cmd_mutex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}