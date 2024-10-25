#include<rclcpp/rclcpp.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include<geometry_msgs/msg/transform_stamped.hpp>
//#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/transform_broadcaster.h>

#ifndef _INCLUDED_WHEEL_ODOMETRY_HPP_
#define _INCLUDED_WHEEL_ODOMETRY_HPP_

class WheelOdometry : public rclcpp::Node
{
private:
    nav_msgs::msg::Odometry odom;
    tf2_ros::TransformBroadcaster tf_caster;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

    bool first_time;

    rclcpp::Time cur_stamp;
    rclcpp::Time last_stamp;
    double d_time;
    double cur_x;
    double cur_y;
    double cur_th;
    double cur_v_x;
    double cur_v_th;

    double Lr, Ll;
    double d_Lr, d_Ll;
    double Lr_o, Ll_o;

    double track_width;
    double wheel_radius;

    void init();
    void update();
    void publish_odom();
    void publish_tf();
    double normalize_angle(double angle);
    //void joint_states_callback(const sensor_msgs::msg::JointState::SharePtr* msg);
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
public:
    WheelOdometry(rclcpp::Node::SharedPtr node);
    void run();
};
#endif