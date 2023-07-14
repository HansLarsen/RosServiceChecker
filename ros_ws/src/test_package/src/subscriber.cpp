#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using std::placeholders::_1;

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : LifecycleNode("my_publisher") {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring…");
    // Add your configuration code here
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating…");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10, std::bind(&MyLifecycleNode::topic_callback, this, _1));
    // Add your activation code here
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::string topic_name = "test_topic";

  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // Do the same for on_deactivate, on_cleanup, on_shutdown
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
