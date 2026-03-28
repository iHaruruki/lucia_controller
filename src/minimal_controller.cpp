#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yarp/os/all.h>
#include <mutex>
#include <vector>

class LuciaController : public rclcpp::Node
{
public:
    LuciaController() : Node("lucia_controller")
    {
        yarp::os::Network yarp;
        
        p_cmd.open("/remoteController/command:o");
        p_enc.open("/remoteController/encoder:i");
        
        yarp::os::Network::connect("/remoteController/command:o", "/vehicleDriver/remote:i");
        yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/encoder:i");
        
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&LuciaController::velocity_callback, this, std::placeholders::_1));
        
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 20ms
            std::bind(&LuciaController::timer_callback, this));
    }
    
    ~LuciaController()
    {
        p_cmd.close();
        p_enc.close();
    }

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::vector<double> cmd = {msg->linear.x, 0.0, msg->angular.z, 0.0};
        {
            std::lock_guard<std::mutex> lock(yarp_mutex_);
            yarp::os::Bottle& bc = p_cmd.prepare();
            bc.clear();
            for (const auto& c : cmd) {
                bc.addFloat64(c);
            }
            p_cmd.write();
        }
    }
    
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(yarp_mutex_);
        yarp::os::Bottle* bt = p_enc.read(false);
        
        if (bt != nullptr)
        {
            std::vector<double> enc(4);
            for (size_t i = 0; i < enc.size(); i++) {
                enc[i] = bt->get(i).asFloat64();
            }
            
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odom";
            odom.twist.twist.linear.x = enc[0];
            odom.twist.twist.angular.z = enc[2];
            odom_publisher_->publish(odom);
        }
    }
    
    // ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // YARP
    yarp::os::BufferedPort<yarp::os::Bottle> p_cmd;
    yarp::os::BufferedPort<yarp::os::Bottle> p_enc;
    std::mutex yarp_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LuciaController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}