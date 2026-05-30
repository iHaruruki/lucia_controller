#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

class JoyCmdVelNode : public rclcpp::Node {
public:
  JoyCmdVelNode() : Node("joy_to_cmd_vel") {
    // Declare parameters
    this->declare_parameter("linear_x_base", 0.1);
    this->declare_parameter("linear_y_base", 0.1);
    this->declare_parameter("linear_z_base", 0.1);
    this->declare_parameter("angular_x_base", 0.3);
    this->declare_parameter("angular_y_base", 0.3);
    this->declare_parameter("angular_z_base", 0.3);
    
    // Get parameters
    linear_x_base_ = this->get_parameter("linear_x_base").as_double();
    linear_y_base_ = this->get_parameter("linear_y_base").as_double();
    linear_z_base_ = this->get_parameter("linear_z_base").as_double();
    angular_x_base_ = this->get_parameter("angular_x_base").as_double();
    angular_y_base_ = this->get_parameter("angular_y_base").as_double();
    angular_z_base_ = this->get_parameter("angular_z_base").as_double();
    
    // Create subscriber and publisher
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyCmdVelNode::joyCallback, this, std::placeholders::_1));
    
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    reject_nav_vel_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/reject_nav_vel", 10);
    
    RCLCPP_INFO(this->get_logger(), "Joy to Cmd_vel node started");
    RCLCPP_INFO(this->get_logger(), "linear_x_base: %.2f m/s, angular_x_base: %.2f rad/s", linear_x_base_, angular_z_base_);
    RCLCPP_DEBUG(this->get_logger(), "linear_x_base: %.2f m/s, linear_y_base: %.2f m/s, linear_z_base: %.2f m/s", linear_x_base_, linear_y_base_, linear_z_base_);
    RCLCPP_DEBUG(this->get_logger(), "angular_x_base: %.2f rad/s, angular_x_base: %.2f rad/s, angular_x_base: %.2f rad/s", angular_x_base_, angular_y_base_, angular_z_base_);
  }

private:
  // AXES indices
  static constexpr int AXES_LEFT_STICK_X = 0; // Left analog-stick X horizontal
  static constexpr int AXES_LEFT_STICK_Y = 1; // Left analog-stick Y vertical
  static constexpr int AXES_L2 = 2;
  static constexpr int AXES_RIGHT_STICK_X = 3;  // Right analog-stick X horizontal
  static constexpr int AXES_RIGHT_STICK_Y = 4;  // Right analog-stick Y vertical
  static constexpr int AXES_R2 = 5;
  static constexpr int AXES_CROSS_KEY_X = 6;  // cross-key X horizontal
  static constexpr int AXES_CROSS_KEY_Y = 7;  // cross-key Y vertical

  // Buttons
  static constexpr int BUTTON_CROSS = 0;
  static constexpr int BUTTON_CIRCLE = 1;
  static constexpr int BUTTON_TRIANGLE = 2;
  static constexpr int BUTTON_SQUARE = 3;
  static constexpr int BUTTON_L1 = 4;
  static constexpr int BUTTON_R1 = 5;
  static constexpr int BUTTON_L2 = 6;
  static constexpr int BUTTON_R2 = 7;
  static constexpr int BUTTON_SHARE = 8;
  static constexpr int BUTTON_OPTIONS = 9;
  static constexpr int BUTTON_PS = 10;
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reject_nav_vel_publisher_;
  
  double linear_x_base_;
  double linear_y_base_;
  double linear_z_base_;
  double angular_x_base_;
  double angular_y_base_;
  double angular_z_base_;

  double speed_multiplier_ = 1.0;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check array sizes
    if (msg->buttons.size() < 11 || msg->axes.size() < 8) {
      RCLCPP_WARN(this->get_logger(), "Incomplete joy message received");
      return;
    }
    
    // Emergency stop (PlayStation button) - publish reject_nav_vel = true
    if (msg->buttons[BUTTON_PS] == 1) {
      publishTwist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      publishRejectNavVel(true);
      RCLCPP_INFO(this->get_logger(), "Emergency stop activated!");
      return;
    }
    
    // SQUARE button - publish reject_nav_vel = false
    if (msg->buttons[BUTTON_SQUARE] == 1) {
      publishRejectNavVel(false);
      RCLCPP_DEBUG(this->get_logger(), "Resume navigation enabled");
    }
    
    // Update speed multiplier based on button presses
    if (msg->buttons[BUTTON_TRIANGLE] == 1) {
      speed_multiplier_ = 1.0;  // 1x speed
    } else if (msg->buttons[BUTTON_CIRCLE] == 1) {
      speed_multiplier_ = 2.0;  // 2x speed
    } else if (msg->buttons[BUTTON_CROSS] == 1) {
      speed_multiplier_ = 3.0;  // 3x speed
    }
    
    double linear_x = 0.0;
    double linear_y = 0.0;
    double linear_z = 0.0;
    double angular_x = 0.0;
    double angular_y = 0.0;
    double angular_z = 0.0;
    
    // AXES_CROSS_KEY: go foward/go backward
    if (msg->axes[AXES_CROSS_KEY_Y] == 1) {
      linear_x = linear_x_base_ * speed_multiplier_;
    } else if (msg->axes[AXES_CROSS_KEY_Y] == -1) {
      linear_x = -linear_x_base_ * speed_multiplier_;
    }
    
    // AXES_CROSS_KEY: go left/go right
    if (msg->axes[AXES_CROSS_KEY_X] == 1) {
      linear_y = linear_y_base_ * speed_multiplier_;
    } else if (msg->axes[AXES_CROSS_KEY_X] == -1) {
      linear_y = -linear_y_base_ * speed_multiplier_;
    }
    
    // L1/R1 buttons: turn left/turn right
    if (msg->buttons[BUTTON_L1] == 1) {
      angular_z = angular_z_base_ * speed_multiplier_;
    }
    if (msg->buttons[BUTTON_R1] == 1) {
      angular_z = -angular_z_base_ * speed_multiplier_;
    }
    
    // Left analog stick: Mirror rotation control
    // Up on stick: mirror rotate minus (negative angular)
    // Down on stick: mirror rotate plus (positive angular)
    // if (msg->axes[AXES_LEFT_STICK_Y] > 0.3) {
    //   angular_z = -angular_z_base_ * speed_multiplier_ * msg->axes[AXES_LEFT_STICK_Y];
    // } else if (msg->axes[AXES_LEFT_STICK_Y] < -0.3) {
    //   angular_z = angular_z_base_ * speed_multiplier_ * (-msg->axes[AXES_LEFT_STICK_Y]);
    // }
    
    publishTwist(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);
  }

  void publishTwist(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z) {
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = linear_x;
    twist_msg->linear.y = linear_y;
    twist_msg->linear.z = linear_z;
    twist_msg->angular.x = angular_x;
    twist_msg->angular.y = angular_y;
    twist_msg->angular.z = angular_z;
    
    cmd_vel_publisher_->publish(std::move(twist_msg));
  }

  void publishRejectNavVel(bool value) {
    auto bool_msg = std::make_unique<std_msgs::msg::Bool>();
    bool_msg->data = value;
    reject_nav_vel_publisher_->publish(std::move(bool_msg));
    RCLCPP_DEBUG(this->get_logger(), "Published /reject_nav_vel: %s", value ? "true" : "false");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}