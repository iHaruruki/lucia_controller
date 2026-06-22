#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class Imu3dOdometry : public rclcpp::Node
{
public:
    Imu3dOdometry() : Node("imu_3d_odometry_node"), last_time_(0, 0, RCL_ROS_TIME)
    {
        // ROS 2 Publishers and Subscribers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&Imu3dOdometry::imuCallback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize state variables to zero
        x_ = y_ = z_ = 0.0;
        vx_ = vy_ = vz_ = 0.0;
        initialized_ = false;

        RCLCPP_INFO(this->get_logger(), "IMU 3D Odometry Node Started.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;

        if (!initialized_) {
            last_time_ = current_time;
            initialized_ = true;
            return;
        }

        // Calculate time step (dt)
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = current_time;

        // 1. Get current orientation quaternion from IMU
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        // 2. Transform linear acceleration from body frame to world frame
        tf2::Vector3 acc_body(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        tf2::Matrix3x3 rotation_matrix(q);
        tf2::Vector3 acc_world = rotation_matrix * acc_body;

        // 3. Subtract gravity (assuming IMU is level at start and gravity is Z-down)
        // Adjust 9.80665 based on your IMU's raw resting output calibration
        acc_world.setZ(acc_world.z() - 9.80665);

        // 4. Double integration step (Euler method) for 3D state estimation
        // Update positions
        x_ += vx_ * dt + 0.5 * acc_world.x() * dt * dt;
        y_ += vy_ * dt + 0.5 * acc_world.y() * dt * dt;
        z_ += vz_ * dt + 0.5 * acc_world.z() * dt * dt;

        // Update velocities
        vx_ += acc_world.x() * dt;
        vy_ += acc_world.y() * dt;
        vz_ += acc_world.z() * dt;

        // 5. Construct and publish the nav_msgs/msg/Odometry message
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = msg->header.frame_id; // Links to your IMU frame
        // odom.child_frame_id = "camera_link";

        // Set Position data
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = z_;
        odom.pose.pose.orientation = msg->orientation; // Pass directly to keep 3D orientation orientation

        // Set Velocity data (expressed in body frame for twist)
        odom.twist.twist.linear.x = msg->linear_acceleration.x * dt; // rough local velocity estimate
        odom.twist.twist.linear.y = msg->linear_acceleration.y * dt;
        odom.twist.twist.linear.z = msg->linear_acceleration.z * dt;
        odom.twist.twist.angular = msg->angular_velocity;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = msg->header.frame_id;

        transformStamped.transform.translation.x = x_;
        transformStamped.transform.translation.y = y_;
        transformStamped.transform.translation.z = z_;
        transformStamped.transform.rotation = msg->orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    // Node publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Odometry State Variables
    double x_, y_, z_;
    double vx_, vy_, vz_;
    rclcpp::Time last_time_;
    bool initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Imu3dOdometry>());
    rclcpp::shutdown();
    return 0;
}