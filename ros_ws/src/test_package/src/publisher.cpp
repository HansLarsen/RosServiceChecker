#include <cstdio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : LifecycleNode("my_subscriber") {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_WARN(get_logger(), "Configuring…");

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);

    // Add your configuration code here
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_WARN(get_logger(), "Activating…");
    // Create a function for when messages are to be sent.
    auto publish_message =
        [this]() -> void
    {
      msg_ = std::make_unique<std_msgs::msg::String>();
      msg_->data = "Hello World: " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

      // Put the message into a queue to be processed by the middleware.
      // This call is non-blocking.
      pub_->publish(std::move(msg_));
    };

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1ms, publish_message);
    // Add your activation code here
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  // Do the same for on_deactivate, on_cleanup, on_shutdown

private:
  std::string topic_name = "test_topic";
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}