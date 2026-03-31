#include "lucia_controller/lucia_controller.hpp"

LuciaController::LuciaController()
    : Node("lucia_controller"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
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

    // Encoder timer (20ms = 50Hz)
    encoder_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
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
        return;
    }

    yarp::os::Bottle& bc = p_cmd.prepare();
    bc.clear();
    for (const auto& c : cmd) {
        bc.addFloat64(c);
    }
    p_cmd.write();
}

std::vector<double> LuciaController::read_encoder_data()
{
    std::vector<double> enc(VehicleStateConstants::ENCODER_DATA_SIZE, 0.0);
    yarp::os::Bottle* bt = p_enc.read(false);

    if (bt == nullptr || bt->size() != VehicleStateConstants::ENCODER_DATA_SIZE) {
        return enc;
    }

    for (size_t i = 0; i < VehicleStateConstants::ENCODER_DATA_SIZE; i++) {
        enc[i] = bt->get(i).asFloat64();
    }

    return enc;
}

void LuciaController::encoder_timer_callback()
{
    std::lock_guard<std::mutex> lock(yarp_mutex_);
    std::vector<double> enc = read_encoder_data();

    if (enc.empty()) {
        return;
    }

    rclcpp::Time current_time = this->get_clock()->now();
    int64_t dt_ns = (current_time - last_callback_time_).nanoseconds();
    dt_ = dt_ns * 1e-9;
    last_callback_time_ = current_time;

    if (dt_ <= VehicleStateConstants::MIN_DT || dt_ > VehicleStateConstants::MAX_DT) {
        RCLCPP_DEBUG(this->get_logger(), "Large time gap detected: %f s", dt_);
        return;
    }

    update_odometry(enc);
    broadcast_transform();
}

void LuciaController::update_odometry(const std::vector<double>& encoder_data)
{
    x_ += encoder_data[0] * dt_;
    y_ += encoder_data[1] * dt_;
    theta_ += encoder_data[2] * dt_;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); 

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = encoder_data[0];
    odom.twist.twist.linear.y = encoder_data[1];
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = encoder_data[2];

    odom_publisher_->publish(odom);
}

void LuciaController::broadcast_transform()
{
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

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