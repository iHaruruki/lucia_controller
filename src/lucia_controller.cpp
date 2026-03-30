#include "lucia_controller/lucia_controller.hpp"

LuciaController::LuciaController()
    : Node("lucia_controller"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      dt_(VehicleStateConstants::DEFAULT_DT),
      previous_state_({-1, -1, -1, -1})
{
    // YARP network check
    yarp::os::Network yarp;
    if (!yarp.checkNetwork(1.0)) {
        RCLCPP_FATAL(get_logger(), "YARP network unavailable");
        throw std::runtime_error("YARP network unavailable");
    }

    // Initialize YARP network
    yarp::os::Network::init();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Open YARP ports
    p_mode.open("/ros2/mode:o");      // mode change of the vehicle system
    p_cmd.open("/ros2/command:o");    // remote-control command input
    p_enc.open("/ros2/encoder:i");    // encoder output of the vehicle system
    p_state.open("/ros2/state:i");    // the state of vehicle system

    // Connect with error checking
    std::this_thread::sleep_for(std::chrono::seconds(1));
    bool mode_connected = yarp::os::Network::connect("/ros2/mode:o", "/vehicleDriver/mode:i");
    bool cmd_connected = yarp::os::Network::connect("/ros2/command:o", "/vehicleDriver/remote:i");
    bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/ros2/encoder:i");
    bool state_connected = yarp::os::Network::connect("/vehicleDriver/state:o", "/ros2/state:i");

    if (!mode_connected || !cmd_connected || !enc_connected || !state_connected) {
        RCLCPP_WARN(this->get_logger(), "WARN: Failed to connect YARP ports");
        throw std::runtime_error("Failed to connect YARP ports");
    }
    RCLCPP_INFO(this->get_logger(), "All YARP ports connected successfully");

    // Publisher
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::QoS(rclcpp::KeepLast(50)).best_effort());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Subscriber
    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/smoothed_cmd_vel",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&LuciaController::velocity_callback, this, std::placeholders::_1));

    // Initialize time
    last_callback_time_ = this->get_clock()->now();

    // Timer (20ms = 50Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LuciaController::timer_callback, this));
}

LuciaController::~LuciaController()
{
    p_mode.close();
    p_cmd.close();
    p_enc.close();
    p_state.close();
    yarp::os::Network::fini();
}

void LuciaController::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(yarp_mutex_);

    if (!is_vehicle_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Vehicle is not ready. Cannot send command.");
        return;
    }

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

bool LuciaController::is_vehicle_ready()
{
    VehicleState state = read_vehicle_state();
    
    // Check vehicle state conditions
    bool init_ok = (state.init == VehicleStateConstants::INIT_SUCCESS);
    bool servo_on = (state.servo == VehicleStateConstants::SERVO_ON);
    bool mode_ok = (state.mode == VehicleStateConstants::MODE_CLIENT_CONTROL);
    bool emergency_ok = (state.emergency == VehicleStateConstants::EMERGENCY_OFF);

    return init_ok && servo_on && mode_ok && emergency_ok;
}

VehicleState LuciaController::read_vehicle_state()
{
    VehicleState state = {0, 0, 0, 0};
    yarp::os::Bottle* state_data = p_state.read(false);

    if (state_data == nullptr || state_data->size() < VehicleStateConstants::STATE_DATA_SIZE) {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "State data not available");
        log_vehicle_state(state);
        return state;
    }

    state.init = state_data->get(0).asInt32();
    state.servo = state_data->get(1).asInt32();
    state.mode = state_data->get(2).asInt32();
    state.emergency = state_data->get(3).asInt32();

    RCLCPP_DEBUG(this->get_logger(), 
        "| init: %d | servo: %d | mode: %d | emergency: %d |",
        state.init, state.servo, state.mode, state.emergency);

    log_vehicle_state(state);
    return state;
}

std::vector<double> LuciaController::read_encoder_data()
{
    std::vector<double> enc(VehicleStateConstants::ENCODER_DATA_SIZE, 0.0);
    yarp::os::Bottle* bt = p_enc.read(false);

    if (bt == nullptr || bt->size() != VehicleStateConstants::ENCODER_DATA_SIZE) {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Encoder data not available");
        return enc;
    }

    for (size_t i = 0; i < VehicleStateConstants::ENCODER_DATA_SIZE; i++) {
        enc[i] = bt->get(i).asFloat64();
    }

    RCLCPP_DEBUG(this->get_logger(),
        "| vx: %lf[m/s] | vy: %lf[m/s] | w: %lf[rad/s] | ta: %lf[rad] |",
        enc[0], enc[1], enc[2], enc[3]);

    return enc;
}

void LuciaController::log_vehicle_state(const VehicleState& state)
{
    // Log only if state changed
    if (state == previous_state_) {
        return;
    }

    std::string init_str = get_init_status_string(state.init);
    std::string servo_str = get_servo_status_string(state.servo);
    std::string mode_str = get_mode_status_string(state.mode);
    std::string emergency_str = get_emergency_status_string(state.emergency);

    RCLCPP_INFO(this->get_logger(),
        "[Vehicle State] Init: %s | Servo: %s | Mode: %s | Emergency: %s",
        init_str.c_str(), servo_str.c_str(), mode_str.c_str(), emergency_str.c_str());

    previous_state_ = state;
}

std::string LuciaController::get_init_status_string(int init)
{
    if (init == VehicleStateConstants::INIT_SUCCESS) {
        return "SUCCESS";
    }
    return "FAILED (value: " + std::to_string(init) + ")";
}

std::string LuciaController::get_servo_status_string(int servo)
{
    switch (servo) {
        case VehicleStateConstants::SERVO_OFF:
            return "OFF";
        case VehicleStateConstants::SERVO_TRANSFERRING:
            return "TRANSFERRING";
        case VehicleStateConstants::SERVO_ON:
            return "ON";
        default:
            return "UNKNOWN (value: " + std::to_string(servo) + ")";
    }
}

std::string LuciaController::get_mode_status_string(int mode)
{
    switch (mode) {
        case VehicleStateConstants::MODE_STOP:
            return "STOP";
        case VehicleStateConstants::MODE_POWER_ASSIST:
            return "POWER_ASSIST";
        case VehicleStateConstants::MODE_CLIENT_CONTROL:
            return "CLIENT_CONTROL";
        default:
            return "UNKNOWN (value: " + std::to_string(mode) + ")";
    }
}

std::string LuciaController::get_emergency_status_string(int emergency)
{
    if (emergency == VehicleStateConstants::EMERGENCY_OFF) {
        return "OFF (Normal)";
    }
    return "ON (Emergency)";
}

void LuciaController::timer_callback()
{
    std::lock_guard<std::mutex> lock(yarp_mutex_);
    std::vector<double> enc = read_encoder_data();

    if (enc.empty()) {
        return;
    }

    // Get current time
    rclcpp::Time current_time = this->get_clock()->now();
    int64_t dt_ns = (current_time - last_callback_time_).nanoseconds();
    dt_ = dt_ns * 1e-9;
    last_callback_time_ = current_time;

    // dt check (15ms < dt < 25ms)
    if (dt_ <= VehicleStateConstants::MIN_DT || dt_ > VehicleStateConstants::MAX_DT) {
        RCLCPP_DEBUG(this->get_logger(), "Abnormal dt: %lf seconds", dt_);
        return;
    }

    update_odometry(enc);
    broadcast_transform(current_time);
}

void LuciaController::update_odometry(const std::vector<double>& encoder_data)
{
    // Update position and orientation
    double theta_mid = theta_ + (encoder_data[2] * dt_) / 2.0;
    x_ += (encoder_data[0] * std::cos(theta_mid) - encoder_data[1] * std::sin(theta_mid)) * dt_;
    y_ += (encoder_data[0] * std::sin(theta_mid) + encoder_data[1] * std::cos(theta_mid)) * dt_;
    theta_ += encoder_data[2] * dt_;

    normalize_angle();

    // Create odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // Position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Orientation (quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Twist (velocity)
    odom.twist.twist.linear.x = encoder_data[0];
    odom.twist.twist.linear.y = encoder_data[1];
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = encoder_data[2];

    odom_publisher_->publish(odom);
}

void LuciaController::broadcast_transform(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped transform;

    // Header
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    // Translation
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    // Rotation (quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // Broadcast
    tf_broadcaster_->sendTransform(transform);
}

void LuciaController::normalize_angle()
{
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<LuciaController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("lucia_controller"),
            "Node initialization failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}