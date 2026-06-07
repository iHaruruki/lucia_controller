#include "mode_display_node.h"

ModeDisplayNode::ModeDisplayNode()
: Node("mode_display_gui_node"),
  interactive_mode_(false),
  message_count_(0),
  has_received_message_(false),
  last_message_time_(0, 0, this->get_clock()->get_clock_type()),
  topic_name_("/reject_nav_vel")
{
  reject_nav_vel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
    std::bind(&ModeDisplayNode::reject_nav_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Mode display GUI node started. Initial mode: Autonomous Mode");
}

void ModeDisplayNode::reject_nav_vel_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  interactive_mode_ = msg->data;
  ++message_count_;
  has_received_message_ = true;
  last_message_time_ = this->now();
}

bool ModeDisplayNode::is_interactive_mode() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return interactive_mode_;
}

std::size_t ModeDisplayNode::message_count() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return message_count_;
}

bool ModeDisplayNode::has_received_message() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_received_message_;
}

rclcpp::Time ModeDisplayNode::last_message_time() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_message_time_;
}

std::string ModeDisplayNode::topic_name() const
{
  return topic_name_;
}
