#include "include/lucia_controller/wheel_odometry.hpp"
#include <cfloat>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

WheelOdometry::WheelOdometry(rclcpp::Node::SharedPtr node)
: Node_(node), tf_broadcaster_(node), first_time(true), cur_x(0.0), cur_y(0.0), cur_th(0.0), cur_v_x(0.0), cur_v_th(0.0),
  Lr_o(0.0), Ll_o(0.0), track_width(1.0), wheel_radius(0.1) {
    init();
}

void WheelOdometry::init() {
    cur_x = 0.0;
    cur_y = 0.0;
    cur_th = 0.0;
    cur_v_x = 0.0;
    cur_v_th = 0.0;
    Lr_o = 0.0;
    Ll_o = 0.0;
}

void WheelOdometry::run() {
    this->declare_parameter("track_width", 1.0);
    this->declare_parameter("wheel_radius", 0.1);

    this->get_parameter("track_width", track_width);
    RCLCPP_INFO(this->get_logger(), "Odometry Node: Set track width: %lf", track_width);

    this->get_parameter("wheel_radius", wheel_radius);
    RCLCPP_INFO(this->get_logger(), "Odometry Node: Set wheel radius: %lf", wheel_radius);

    sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&WheelOdometry::joint_states_callback, this, std::placeholders::_1));
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
}

void WheelOdometry::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    cur_stamp = msg->header.stamp;
    d_time = (cur_stamp - last_stamp).seconds();
    last_stamp = cur_stamp;

    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "wheel_right_joint") Lr = wheel_radius * msg->position[i];
        else if (msg->name[i] == "wheel_left_joint") Ll = wheel_radius * msg->position[i];
        else return;
    }

    d_Lr = Lr - Lr_o;
    d_Ll = Ll - Ll_o;
    Lr_o = Lr;
    Ll_o = Ll;

    update();
}

void WheelOdometry::update() {
    double d_L = (d_Lr + d_Ll) / 2.0;
    double d_theta = 0.0;

    if (std::fabs(std::fabs(d_Lr - d_Ll) < 0.000001)) {
        cur_x += d_L * std::cos(cur_th);
        cur_y += d_L * std::sin(cur_th);
    } else {
        d_theta = (d_Lr - d_Ll) / track_width;
        double rho = d_L / d_theta;
        double d_Lp = 2.0 * rho * std::sin(d_theta * 0.5);
        cur_x += d_Lp * std::cos(cur_th + d_theta * 0.5);
        cur_y += d_Lp * std::sin(cur_th + d_theta * 0.5);
        cur_th = normalize_angle(cur_th + d_theta);
    }

    if (d_time < 0.000001) {
        cur_v_x = 0.0;
        cur_v_th = 0.0;
    } else {
        cur_v_x = d_L / d_time;
        cur_v_th = d_theta / d_time;
    }

    publish_odom();
    publish_tf();
}

void WheelOdometry::publish_odom() {
    odom.header.stamp = cur_stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint"; // 自分のurdfに合わせて修正

    odom.pose.pose.position.x = cur_x;
    odom.pose.pose.position.y = cur_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, cur_th));

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = FLT_MAX;
    odom.pose.covariance[21] = FLT_MAX;
    odom.pose.covariance[28] = FLT_MAX;
    odom.pose.covariance[35] = 0.01;

    odom.twist.twist.linear.x = cur_v_x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = cur_v_th;

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[14] = FLT_MAX;
    odom.twist.covariance[21] = FLT_MAX;
    odom.twist.covariance[28] = FLT_MAX;
    odom.twist.covariance[35] = 0.01;

    pub_odom->publish(odom);
}

void WheelOdometry::publish_tf() {
    geometry_msgs::msg::TransformStamped tf_trans;

    tf_trans.header.stamp = odom.header.stamp;
    tf_trans.header.frame_id = odom.header.frame_id;
    tf_trans.child_frame_id = odom.child_frame_id;
    tf_trans.transform.translation.x = odom.pose.pose.position.x;
    tf_trans.transform.translation.y = odom.pose.pose.position.y;
    tf_trans.transform.translation.z = odom.pose.pose.position.z;
    tf_trans.transform.rotation = odom.pose.pose.orientation;

    tf_caster.sendTransform(tf_trans);
}

double WheelOdometry::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdometry>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}