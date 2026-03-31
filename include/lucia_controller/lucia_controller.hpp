#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <yarp/os/all.h>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>

// Vehicle state constants
namespace VehicleStateConstants {
    constexpr int INIT_SUCCESS = 255;
    constexpr int SERVO_OFF = 0;
    constexpr int SERVO_TRANSFERRING = 1;
    constexpr int SERVO_ON = 2;
    constexpr int MODE_STOP = 0;
    constexpr int MODE_POWER_ASSIST = 1;
    constexpr int MODE_CLIENT_CONTROL = 2;
    constexpr int EMERGENCY_OFF = 1;
    constexpr int EMERGENCY_ON = 0;
    
    constexpr double MIN_DT = 0.015;  // 15ms
    constexpr double MAX_DT = 0.025;  // 25ms
    constexpr double DEFAULT_DT = 0.020;  // 20ms
    
    constexpr size_t ENCODER_DATA_SIZE = 4;
    constexpr size_t STATE_DATA_SIZE = 4;
    constexpr size_t CMD_DATA_SIZE = 4;
}

// Vehicle state structure for logging
struct VehicleState {
    int init;
    int servo;
    int mode;
    int emergency;
    
    bool operator==(const VehicleState& other) const {
        return init == other.init && servo == other.servo && 
               mode == other.mode && emergency == other.emergency;
    }
    
    bool operator!=(const VehicleState& other) const {
        return !(*this == other);
    }
};

class LuciaController : public rclcpp::Node
{
public:
    LuciaController();
    ~LuciaController();

private:
    // ROS2 callbacks
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encoder_timer_callback();

    // YARP communication
    void send_velocity_command(const std::vector<double>& cmd);
    bool is_vehicle_ready();
    VehicleState read_vehicle_state();
    std::vector<double> read_encoder_data();

    // Vehicle state management
    void log_vehicle_state(const VehicleState& state);
    std::string get_init_status_string(int init);
    std::string get_servo_status_string(int servo);
    std::string get_mode_status_string(int mode);
    std::string get_emergency_status_string(int emergency);

    // Odometry and transform
    void update_odometry(const std::vector<double>& encoder_data);
    void broadcast_transform(const rclcpp::Time& stamp);

    // ROS2 publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr encoder_timer_;
    rclcpp::TimerBase::SharedPtr vehicle_state_timer_;  // timer for vehicle state
    //VehicleState cached_vehicle_state_;         // Cache the state
    std::mutex state_mutex_;                    // Protect state access

    // YARP ports
    yarp::os::BufferedPort<yarp::os::Bottle> p_mode;
    yarp::os::BufferedPort<yarp::os::Bottle> p_cmd;
    yarp::os::BufferedPort<yarp::os::Bottle> p_enc;
    yarp::os::BufferedPort<yarp::os::Bottle> p_state;
    std::mutex yarp_mutex_;

    // Odometry state
    double x_;
    double y_;
    double theta_;
    double dt_;
    rclcpp::Time last_callback_time_;

    // Vehicle state tracking
    VehicleState previous_state_;
};