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

// YARP port
yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; //motor command
yarp::os::BufferedPort<yarp::os::Bottle> p_enc; //encoder reading


void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::vector<double> cmd(4);
    cmd[0]=msg->linear.x;
    cmd[1]=0.00;
    cmd[2]=msg->angular.z;
    cmd[3]=0.00;

    yarp::os::Bottle& bc = p_cmd.prepare();
    bc.clear();
    for(int i = 0; i < cmd.size(); i++){
       bc.addFloat64(cmd[i]);
    }
    p_cmd.write();

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received velocity command: linear.x=%f, angular.z=%f", msg->linear.x, msg->angular.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Send velocity command: linear.x=%f, angular.z=%f", cmd[0], cmd[2]);
}

int main(int argc, char * argv[])
{
    /*----------YARP Initialize----------*/
	yarp::os::Network yarp;

	// Open YARP port
	p_cmd.open("/remoteController/command:o");  //motor command
	p_enc.open("/remoteController/encoder:i");  //encoder reading

    // Connect to the sender port
    yarp::os::Network::connect("/remoteController/command:o","/vehicleDriver/remote:i");    //motor command
    yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/encoder:i");  //encoder reading

    /*----------ROS2 Initialize----------*/
    rclcpp::init(argc, argv);   //Node initialize
    auto node = rclcpp::Node::make_shared("lucia_controller"); //Node create
    //create publisher
    
    
    //create subscriber
    auto velocity_subscriber = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, velocity_callback);


    rclcpp::WallRate loop_rate(100); //loop rate 100Hz
   	while(rclcpp::ok())
    {
        // Motor command
        rclcpp::spin_some(node);
        //velocity_callback;
        // Read encoder
        /*yarp::os::Bottle* bt = p_enc.read(false);
        std::vector<double> enc(4);
        if (bt != nullptr)
        {
            for(int i = 0; i < enc.size(); i++){
                enc[i] = bt->get(i).asFloat64();
            }
        }*/
        
        /*std::cout << "enc " 
            << enc[0] << " " 
            << enc[1] << " " 
            << enc[2] << " " 
            << enc[3] << " " 
            << "cmd " 
            << cmd[0] << " " 
            << cmd[1] << " " 
            << cmd[2] << " " 
            << cmd[3] << std::endl;*/

        yarp::os::Time::delay(0.05);
        loop_rate.sleep();  //coodinate with loop_rate
    }	

    p_cmd.close();
    p_enc.close();
	return 0;
}