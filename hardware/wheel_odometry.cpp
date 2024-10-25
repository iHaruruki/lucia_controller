#include "/home/peach/ros2_ws/src/lucia_controller/hardware/include/lucia_controller/wheel_odometry.hpp"

JointWheelOdometry::JointWheelOdometry()
: Node("joint_wheel_odometry"),
  wheel_base_(0.5),  // 車輪間の距離
  wheel_radius_(0.1),  // 車輪の半径
  x_(0.0),
  y_(0.0),
  theta_(0.0)
{
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&JointWheelOdometry::jointStateCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    last_time_ = this->now();
}

void JointWheelOdometry::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // 車輪の速度を取得
    double left_wheel_velocity = msg->velocity[0];
    double right_wheel_velocity = msg->velocity[1];

    // 時間の差を計算
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // オドメトリの計算
    double v = (right_wheel_velocity + left_wheel_velocity) * wheel_radius_ / 2.0;
    double omega = (right_wheel_velocity - left_wheel_velocity) * wheel_radius_ / wheel_base_;

    x_ += v * cos(theta_) * dt;
    y_ += v * sin(theta_) * dt;
    theta_ += omega * dt;

    // オドメトリメッセージの作成
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = cos(theta_ / 2.0);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    // オドメトリメッセージのパブリッシュ
    odom_pub_->publish(odom);

    // TFメッセージの作成
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation.z = sin(theta_ / 2.0);
    transform.transform.rotation.w = cos(theta_ / 2.0);

    // TFメッセージのブロードキャスト
    tf_broadcaster_->sendTransform(transform);

    // 時間の更新
    last_time_ = current_time;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointWheelOdometry>());
    rclcpp::shutdown();
    return 0;
}