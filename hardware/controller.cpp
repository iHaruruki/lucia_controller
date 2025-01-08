#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <vector>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// 最大速度の定義
#define MAX_LINEAR 0.2 //[m/s]
#define MAX_ANGULAR 0.4 //[rad/s]

using namespace std::chrono_literals;

// YARP port
yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; //motor command
yarp::os::BufferedPort<yarp::os::Bottle> p_enc; //encoder reading

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
    : Node("odom_publisher"), x(0.0), y(0.0), th(0.0), odom_sequence_(0),
      current_linear_x_(0.0), current_linear_y_(0), current_angular_z_(0),
      target_linear_x_(0.0), target_linear_y_(0.0), target_angular_z_(0.0),
      a_max_linear_((MAX_LINEAR) / 1.0),    // 最大加速度(m/s²)
      a_max_angular_(MAX_ANGULAR / 1.0)     //最大角加速度(rad/s²)
    {
        RCLCPP_INFO(this->get_logger(), "OdomPublisher node is starting...");

        //YARP networkの接続確認（5秒経過しても接続できない場合はエラーを出力）
        yarp::os::Network yarp;
        if(!yarp.checkNetwork(5.0)){
            RCLCPP_ERROR(this->get_logger(), "YARP network is not available");
            throw std::runtime_error("YARP network is not available");
        }

        //ROS2 PublisherとSubscriberの作成
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",10);
        odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OdomPublisher::velocity_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(30ms, std::bind(&OdomPublisher::timer_callback, this));

        //YARPポートの設定
        p_cmd.open("/remoteController/command:o");  //motor command
        p_enc.open("/remoteController/encoder:i");  //encoder reading

        //ポートの接続-Lucia側のYARPポートと接続する
        bool cmd_connected = yarp::os::Network::connect("/remoteController/command:o","/vehicleDriver/remote:i");    //motor command
        bool enc_connected = yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/encoder:i");  //encoder reading

        //ポートの接続確認
        if(!cmd_connected){
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to /vehicleDriver/remote:i");
        }
        if(!enc_connected){
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to /vehicleDriver/encoder:o");
        }
    }

    ~OdomPublisher()
    {
        p_cmd.close();
        p_enc.close();
    }

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        //速度指令の制限
        double cmd_linear_x = std::clamp(msg->linear.x, -MAX_LINEAR,MAX_LINEAR);
        double cmd_linear_y = std::clamp(msg->linear.y, -MAX_LINEAR,MAX_LINEAR);
        double cmd_angular_z = std::clamp(msg->angular.z, -MAX_ANGULAR, MAX_ANGULAR);

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            target_linear_x_ = cmd_linear_x;
            target_linear_y_ = cmd_linear_y;
            target_angular_z_ = cmd_angular_z;
        }
    }

    void timer_callback()
    {
        // 現在時刻の取得
        rclcpp::Time current_time = this->get_clock()->now();

        // 静的変数として前回の時刻を保持
        static rclcpp::Time last_time = current_time;

        // 時間の経過を計算
        double dt = (current_time - last_time).seconds();

        // 現在の速度を目標速度に向けて更新
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            current_linear_x_ = update_speed(current_linear_x_, target_linear_x_, a_max_linear_);
            current_linear_y_ = update_speed(current_linear_y_, target_linear_y_, a_max_linear_);
            current_angular_z_ = update_speed(current_angular_z_, target_angular_z_, a_max_angular_);
            latest_cmd_[0] = current_linear_x_;
            latest_cmd_[1] = current_linear_y_;
            latest_cmd_[2] = current_angular_z_;
            latest_cmd_[3] = 0.00;
        }

        //速度指令の送信
        yarp::os::Bottle& bc = p_cmd.prepare();
        bc.clear();
        for(const auto& c : latest_cmd_){
            bc.addFloat64(c);
        }
        p_cmd.write();

        //エンコーダの読み取り
        yarp::os::Bottle* bt = p_enc.read(false);
        if(bt != nullptr)
        {
            //エンコーダデータの取得
            std::vector<double> enc(4);
            for(std::vector<double>::size_type i = 0; i < enc.size(); i++){
                enc[i] = bt->get(i).asFloat64();
                //RCLCPP_INFO(this->get_logger(), "Encoder data received");
            }

            // 速度を取得
            double vx = enc[0];
            double vy = enc[1];
            double w  = enc[2]; //w[rad/s]
            double ta = enc[3]; //ta[rad]
            //RCLCPP_INFO(this->get_logger(), "encoder data : vx=%f, vy=%f\n, w=%f[rad/s], ta=%f[rad]", vx, vy, w, ta);

            // オドメトリの計算
            x += (vx * cos(th) - vy * sin(th)) * dt;
            y += (vx * sin(th) + vy * cos(th)) * dt;
            th += w * dt;
            //RCLCPP_INFO(this->get_logger(), "x_=%f, y_=%f, th_=%f", x, y, th);

            // オドメトリメッセージの作成
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            //odom.header.seq = odom_sequence_++;

            // 位置情報
            tf2::Quaternion odom_q;
            odom_q.setRPY(0, 0, th);
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.x = odom_q.x();
            odom.pose.pose.orientation.y = odom_q.y();
            odom.pose.pose.orientation.z = odom_q.z();
            odom.pose.pose.orientation.w = odom_q.w();

            // 速度情報
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = w;
            //odom.header.stamp = this->now();
            odom_publisher_->publish(odom);

            // オドメトリのブロードキャスト
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom.pose.pose.orientation;
            //odom_trans.transform.rotation.x = odom_q.x();
            //odom_trans.transform.rotation.y = odom_q.y();
            //odom_trans.transform.rotation.z = odom_q.z();
            //odom_trans.transform.rotation.w = odom_q.w();
            odom_broadcaster_->sendTransform(odom_trans);

            //RCLCPP_INFO(this->get_logger(), "Publishing odometry data");
        }
        else
        {
            failure_count = failure_count + 1;
            RCLCPP_WARN(this->get_logger(), "Failed to read encoder data count:%d", failure_count);
        }
        // 前回の時刻を更新
        last_time = current_time;
    }

    double update_speed(double current, double target, double max_acc)
    {
        if(current < target)
        {
            current += max_acc * 0.03;  //dt = 0.05[s]
            if(current > target)
            current = target;
        }
        else if(current > target)
        {
            current -= max_acc * 0.03;
            if(current < target)
            {
                current = target;
            }
        }
        return current;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x, y, th;
    double latest_cmd_[4] = {0.0, 0.0, 0.0, 0.0};  //motor_comand
    uint32_t odom_sequence_;
    int failure_count;

    // 現在の速度
    double current_linear_x_;
    double current_linear_y_;
    double current_angular_z_;

    // 目標速度
    double target_linear_x_;
    double target_linear_y_;
    double target_angular_z_;
    double a_max_linear_;   // 最大加速度 (m/s²)
    double a_max_angular_;  // 最大角加速度 (rad/s²)

    std::mutex cmd_mutex_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   //Node initialize

    //auto node = rclcpp::Node::make_shared("OdomPublisher"); //Node create
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown(); //Node shutdown

	return 0;
}
