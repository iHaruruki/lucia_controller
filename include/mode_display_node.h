#ifndef MODE_DISPLAY_NODE_H_
#define MODE_DISPLAY_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cstddef>
#include <mutex>
#include <string>

class ModeDisplayNode : public rclcpp::Node
{
public:
  ModeDisplayNode();

  bool is_interactive_mode() const;
  std::size_t message_count() const;
  bool has_received_message() const;
  rclcpp::Time last_message_time() const;
  std::string topic_name() const;

private:
  void reject_nav_vel_callback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reject_nav_vel_sub_;
  mutable std::mutex state_mutex_;
  bool interactive_mode_;
  std::size_t message_count_;
  bool has_received_message_;
  rclcpp::Time last_message_time_;
  const std::string topic_name_;
};

#endif  // MODE_DISPLAY_NODE_H_
