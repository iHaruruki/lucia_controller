#ifndef JOINT_WHEEL_ODOMETRY_H
#define JOINT_WHEEL_ODOMETRY_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class JointWheelOdometry : public rclcpp::Node
{
public:
    JointWheelOdometry();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double wheel_base_;
    double wheel_radius_;
    double x_;
    double y_;
    double theta_;
    rclcpp::Time last_time_;
};

#endif // JOINT_WHEEL_ODOMETRY_H