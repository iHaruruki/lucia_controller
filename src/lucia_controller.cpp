#include "lucia_controller/lucia_controller.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

LuciaController::LuciaController()
    : Node("lucia_controller"),
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      dt_(VehicleStateConstants::DEFAULT_DT)
{
    // Initialize YARP network
    yarp::os::Network::init();

    // Open YARP ports
    p_cmd.open("/ros2/command:o");
    p_enc.open("/ros2/encoder:i");

    // Connect ports
    bool cmd_connected = yarp::os::Network::connect("/ros2/command:o", "/vehicleDriver/remote:i");
    bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/ros2/encoder:i");

    if (!cmd_connected || !enc_connected) {
        RCLCPP_WARN(this->get_logger(), "WARN: Failed to connect YARP ports");
        throw std::runtime_error("Failed to connect YARP ports");
    }
    RCLCPP_INFO(this->get_logger(), "All YARP ports connected successfully");

    // Publisher
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Subscriber
    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/smoothed_cmd_vel",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&LuciaController::velocity_callback, this, std::placeholders::_1));

    // Initialize time
    last_callback_time_ = this->get_clock()->now();

    // Encoder timer (50ms = 20Hz)
    encoder_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LuciaController::encoder_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "LuciaController initialized");
}

LuciaController::~LuciaController()
{
    p_cmd.close();
    p_enc.close();
    yarp::os::Network::fini();
}

void LuciaController::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(yarp_mutex_);
    std::vector<double> cmd = {msg->linear.x, msg->linear.y, msg->angular.z, 0.0};
    send_velocity_command(cmd);
}

void LuciaController::send_velocity_command(const std::vector<double>& cmd)
{
    if (cmd.size() != VehicleStateConstants::CMD_DATA_SIZE) {
        RCLCPP_WARN(this->get_logger(), "Invalid command size: %zu", cmd.size());
        return;
    }

    yarp::os::Bottle& bc = p_cmd.prepare();
    bc.clear();
    for (const auto& c : cmd) {
        bc.addFloat64(c);
    }
    p_cmd.write();
}

void LuciaController::encoder_timer_callback()
{
    std::lock_guard<std::mutex> lock(yarp_mutex_);

    rclcpp::Time current_time = this->get_clock()->now();
    int64_t dt_ns = (current_time - last_callback_time_).nanoseconds();
    dt_ = dt_ns * 1e-9;
    last_callback_time_ = current_time;

    if (dt_ <= VehicleStateConstants::MIN_DT || dt_ > VehicleStateConstants::MAX_DT) {
        RCLCPP_DEBUG(this->get_logger(), "Time gap check failed: dt=%f", dt_);
        return;
    }

    readEncoderAndUpdate(dt_, current_time);
}

void LuciaController::readEncoderAndUpdate(double dt, const rclcpp::Time& stamp)
{
    yarp::os::Bottle* bt = p_enc.read(false);

    if (!bt) {
        encoder_failure_count_++;
        if (encoder_failure_count_ % 50 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Encoder read failed (%d times)", encoder_failure_count_);
        }
        publishOdometry(stamp, 0.0, 0.0, 0.0);
        return;
    }

    // Encoder size check
    if (bt->size() < 3) {
        encoder_error_count_++;
        RCLCPP_DEBUG(this->get_logger(), "Encoder size too short: %ld (expected >= 3)", bt->size());
        return;
    }

    // Get encoder values
    double vx = bt->get(0).asFloat64();
    double vy = bt->get(1).asFloat64();
    double w = bt->get(2).asFloat64();
    double ta = bt->get(2).asFloat64();

    // NaN/Inf validation
    if (std::isnan(vx) || std::isnan(vy) || std::isnan(w) ||
        std::isinf(vx) || std::isinf(vy) || std::isinf(w)) {
        encoder_error_count_++;
        RCLCPP_WARN(this->get_logger(), "Invalid encoder value (NaN/Inf): vx=%f, vy=%f, vth=%f", vx, vy, w);
        return;
    }

    // Debug log
    count ++;
    if(count % 10 == 0){
        RCLCPP_DEBUG(this->get_logger(), "Encoder: vx=%f, vy=%f, w=%f, ta,=%f, dt=%f", vx, vy, w, ta, dt);
    }

    // Integrate odometry
    integrate(vx, vy, w, dt);

    // Publish odometry and broadcast transform
    publishOdometry(stamp, vx, vy, w);
}

void LuciaController::integrate(double vx, double vy, double vth, double dt)
{
    x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
    y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;
    yaw_ += vth * dt;

    // Normalize angle to [-π, π]
    if (yaw_ > M_PI) {
        yaw_ -= 2 * M_PI;
    }
    if (yaw_ < -M_PI) {
        yaw_ += 2 * M_PI;
    }

    RCLCPP_DEBUG(this->get_logger(), "Odometry: x=%f, y=%f, yaw=%f (rad, %.1f deg)", x_, y_, yaw_, yaw_ * 180.0 / M_PI);
}

void LuciaController::publishOdometry(const rclcpp::Time& stamp, double vx, double vy, double vth)
{
    // Create odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // Position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Linear
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;

    // Angular
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;

    // Covariance
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    // Pose
    odom.pose.covariance[0] = 0.02;   // x
    odom.pose.covariance[7] = 0.02;   // y
    odom.pose.covariance[14] = 1e6;   // z
    odom.pose.covariance[21] = 1e6;   // roll
    odom.pose.covariance[28] = 1e6;   // pitch
    odom.pose.covariance[35] = 0.5;   // yaw

    // Twist
    odom.twist.covariance[0] = 0.01;  // vx
    odom.twist.covariance[7] = 0.01;  // vy
    odom.twist.covariance[14] = 1e6;  // vz
    odom.twist.covariance[21] = 1e6;  // vroll
    odom.twist.covariance[28] = 1e6;  // vpitch
    odom.twist.covariance[35] = 0.5;  // vth

    odom_publisher_->publish(odom);

    // Broadcast transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LuciaController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}