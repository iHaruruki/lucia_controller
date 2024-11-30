#include <chrono>
#include <memory>
#include <cmath>

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

using namespace std::chrono_literals;

// YARP port
yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; //motor command
yarp::os::BufferedPort<yarp::os::Bottle> p_enc; //encoder reading

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
    : Node("odom_publisher"), x_(0.0), y_(0.0), th_(0.0)
    {
        //YARP networkの接続確認（8秒経過しても接続できない場合はエラーを出力）
        yarp::os::Network yarp;
        if(!yarp.checkNetwork(8.0)){
            RCLCPP_ERROR(this->get_logger(), "YARP network is not available");
            throw std::runtime_error("YARP network is not available");
        }

        //ROS2 PublisherとSubscriberの作成
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",10);
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OdomPublisher::velocity_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&OdomPublisher::timer_callback, this));

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
        //RCLCPP_INFO(this->get_logger(), "Receive velocity command: linear.x=%f, angular.z=%f", msg->linear.x, msg->angular.z);

        //速度指令の制限
        if(msg->linear.x > 0.15) msg->linear.x = 0.15;
        if(msg->linear.x < -0.15) msg->linear.x = -0.15;
        if(msg->angular.z > 0.3) msg->angular.z = 0.3;
        if(msg->angular.z < -0.3) msg->angular.z = -0.3;

        cmd[0] = msg->linear.x;
        cmd[1] = 0.00;
        cmd[2] = msg->angular.z;
        cmd[3] = 0.00;
    }

    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Timer callback triggered");

        //速度指令の送信
        yarp::os::Bottle& bc = p_cmd.prepare();
        bc.clear();
        for(const auto& c : cmd){
            bc.addFloat64(c);
        }
        p_cmd.write();
        
        RCLCPP_INFO(this->get_logger(), "Send velocity command to YARP: linear.x=%f, angular.z=%f", cmd[0], cmd[2]);

        //エンコーダの読み取り
        yarp::os::Bottle* bt = p_enc.read(false);
        if(bt != nullptr)
        {
            //エンコーダデータの取得
            std::vector<double> enc(4);
            for(int i = 0; i < enc.size(); i++){
                enc[i] = bt->get(i).asFloat64();
                //RCLCPP_INFO(this->get_logger(), "Encoder data received");
            }
            RCLCPP_INFO(this->get_logger(), "Recived encoder data: left_vel_speed=%f, right_vel_speed=%f",enc[0],enc[1]);

            //左右の車輪の速度を取得
            double left_vel_speed = enc[0];
            double right_vel_speed = enc[1];
            double wheel_base = 0.5; //車輪間距離

            //オドメトリの計算
            double dt = 0.05;   //タイマー周期と一致させること　timer_ 
            double vx = (right_vel_speed + left_vel_speed) / 2.0;
            double vy = 0.0;
            double vth = (right_vel_speed - left_vel_speed) / wheel_base;

            double delta_x = vx * std::cos(th_) * dt;
            double delta_y = vx * std::sin(th_) * dt;
            double delta_th = vth * dt;

            x_ += delta_x;
            y_ += delta_y;
            th_ += delta_th;

            //オドメトリメッセージの作成
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odom";

            //位置
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, th_);
            odom.pose.pose.orientation = tf2::toMsg(q);

            //速度
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            //Publish
            odom_publisher_->publish(odom);

            RCLCPP_INFO(this->get_logger(), "Odom data Published");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read encoder data");
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_, y_, th_;
    double cmd[4];  //motor_comand
    double t_total, v_max, a_max, j_max;    //全体の移動時間[s], 最大速度, 最大加速度, 最大躍度
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