#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yarp/os/all.h>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

class LuciaController : public rclcpp::Node
{
public:
    LuciaController() : Node("lucia_controller"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // YARP network check
        yarp::os::Network yarp;
        if(!yarp.checkNetwork(1.0)){
            RCLCPP_FATAL(get_logger(), "YARP network unavailable");
            throw std::runtime_error("YARP network unavailable");
        }

        // Initialize YARP network
        yarp::os::Network::init();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Open YARP ports
        p_mode.open("/ros2/mode:o"); //mode change of the vehicle system
        p_cmd.open("/ros2/command:o"); //remote-control command input
        p_enc.open("/ros2/encoder:i"); //encoder output of the vehicle system
        p_state.open("/ros2/state:i"); //the state of vehicle system
        
        // Connect with error checking
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bool mode_connected = yarp::os::Network::connect("/ros2/mode:o", "/vehicleDriver/mode:i");
        bool cmd_connected = yarp::os::Network::connect("/ros2/command:o", "/vehicleDriver/remote:i");
        bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/ros2/encoder:i");
        bool state_connected = yarp::os::Network::connect("/vehicleDriver/state:o", "/ros2/state:i");
        
        if (!mode_connected || !cmd_connected || !enc_connected || !state_connected) {
            RCLCPP_WARN(this->get_logger(), "WARN: Failed to connect YARP ports");
            throw std::runtime_error("Failed to connect state port");
        }
        RCLCPP_INFO(this->get_logger(), "All YARP ports connected successfully");
        
        // QoS
        auto qos_odom = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos_odom);
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_smoothed", 
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
            std::bind(&LuciaController::velocity_callback, this, std::placeholders::_1));
        
        // Timer (10ms = 100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LuciaController::timer_callback, this));
    }
    
    ~LuciaController()
    {
        p_mode.close();
        p_cmd.close();
        p_enc.close();
        p_state.close();
        yarp::os::Network::fini();
    }

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(yarp_mutex_);
        
        // Check if vehicle is ready
        if (!is_vehicle_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Vehicle is not ready. Cannot send command.");
            return;
        }
        
        std::vector<double> cmd = {msg->linear.x, msg->linear.y, msg->angular.z, 0.0};
        yarp::os::Bottle& bc = p_cmd.prepare();
        bc.clear();
        for (const auto& c : cmd) {
            bc.addFloat64(c);
        }
        p_cmd.write();
    }
    
    bool is_vehicle_ready()
    {
        // Read vehicle state
        yarp::os::Bottle* state_data = p_state.read(false);
        
        RCLCPP_DEBUG(this->get_logger(), "| init: %d | servo: %d | mode: %d | emergency: %d |", state_data->get(0).asInt32(), state_data->get(1).asInt32(), state_data->get(2).asInt32(), state_data->get(3).asInt32());
        if (state_data == nullptr || state_data->size() < 4) {
            RCLCPP_DEBUG_ONCE(this->get_logger(), "State data not available");
            return false;
        }
        
        int init = state_data->get(0).asInt32();      // Initialization status
        int servo = state_data->get(1).asInt32();     // Servo status
        int mode = state_data->get(2).asInt32();      // Control mode
        int emergency = state_data->get(3).asInt32(); // Emergency stop status
        
        // Log vehicle state
        log_vehicle_state(init, servo, mode, emergency);
        
        // Check vehicle state conditions
        bool init_ok = (init == 255);        // Initialization successful
        bool servo_on = (servo == 2);        // Servo ON
        bool mode_ok = (mode == 2);          // Client control mode
        bool emergency_ok = (emergency == 1); // Emergency stop OFF

        return init_ok && servo_on && mode_ok && emergency_ok;
    }
    
    void log_vehicle_state(int init, int servo, int mode, int emergency)
    {
        // Initialization status
        std::string init_str;
        if (init == 255) {
            init_str = "SUCCESS";
        } else {
            init_str = "FAILED (value: " + std::to_string(init) + ")";
        }
        
        // Servo status
        std::string servo_str;
        if (servo == 0) servo_str = "OFF";
        else if (servo == 1) servo_str = "TRANSFERRING";
        else if (servo == 2) servo_str = "ON";
        else servo_str = "UNKNOWN (value: " + std::to_string(servo) + ")";
        
        // Control mode
        std::string mode_str;
        if (mode == 0) mode_str = "STOP";
        else if (mode == 1) mode_str = "POWER_ASSIST";
        else if (mode == 2) mode_str = "CLIENT_CONTROL";
        else mode_str = "UNKNOWN (value: " + std::to_string(mode) + ")";
        
        // Emergency stop
        std::string emergency_str = (emergency == 1) ? "OFF (Normal)" : "ON (Emergency)";
        
        // Log only if state changed
        static int prev_init = -1, prev_servo = -1, prev_mode = -1, prev_emergency = -1;
        if (init != prev_init || servo != prev_servo || mode != prev_mode || emergency != prev_emergency) {
            RCLCPP_INFO(this->get_logger(), 
                "[Vehicle State] Init: %s | Servo: %s | Mode: %s | Emergency: %s",
                init_str.c_str(), servo_str.c_str(), mode_str.c_str(), emergency_str.c_str());
            
            prev_init = init;
            prev_servo = servo;
            prev_mode = mode;
            prev_emergency = emergency;
        }
    }
    
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(yarp_mutex_);
        yarp::os::Bottle* bt = p_enc.read(false);
        
        if (bt != nullptr && bt->size() == 4)
        {
            std::vector<double> enc(4);
            for (size_t i = 0; i < enc.size(); i++) {
                enc[i] = bt->get(i).asFloat64();
            }
            
            RCLCPP_DEBUG(this->get_logger(), "| vx: %lf[m/s] | vy: %lf[m/s] | w: %lf[rad/s] | ta: %lf[rad] |", enc[0], enc[1], enc[2], enc[3]);

            const double dt = 0.010;  // 10ms (100Hz)
            x_ += enc[0] * dt;
            y_ += enc[1] * dt;
            theta_ += enc[2] * dt;
            
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";
            
            // Pose (position and orientation)
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
            odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
            
            // Twist (velocity)
            odom.twist.twist.linear.x = enc[0];
            odom.twist.twist.linear.y = enc[1];
            odom.twist.twist.angular.z = enc[2];
            
            odom_publisher_->publish(odom);
        }
    }
    
    // ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // YARP
    yarp::os::BufferedPort<yarp::os::Bottle> p_mode;
    yarp::os::BufferedPort<yarp::os::Bottle> p_cmd;
    yarp::os::BufferedPort<yarp::os::Bottle> p_enc;
    yarp::os::BufferedPort<yarp::os::Bottle> p_state;
    std::mutex yarp_mutex_;
    
    double x_, y_, theta_;
};

int main(int argc, char * argv[])
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