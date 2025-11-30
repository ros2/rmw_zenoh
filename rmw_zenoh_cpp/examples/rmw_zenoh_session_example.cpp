#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <zenoh.hxx>

#include <rmw_zenoh_cpp/rmw_zenoh.h>

class ZenohSessionExampleNode : public rclcpp::Node
{
public:
  ZenohSessionExampleNode()
  : Node("zenoh_session_example")
  {
    // Get the RMW context from rclcpp
    auto context = rclcpp::contexts::get_global_default_context()->get_rmw_context();

    // Get the Zenoh session
    session_ = rmw_zenoh_get_session(context);
    if (!session_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get Zenoh session from RMW context");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully obtained Zenoh session from rclcpp application");

    // Example usage: Put a key-value pair using the session (if supported)
    // Note: The exact API may vary; this demonstrates session access
    try {
      // This is a placeholder - actual Zenoh session operations would depend on the API
      RCLCPP_INFO(this->get_logger(), "Zenoh session is available for advanced operations");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error with Zenoh session: %s", e.what());
    }

    // Create publisher and subscriber for demonstration
    publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "example_topic", 10,
      std::bind(&ZenohSessionExampleNode::topic_callback, this, std::placeholders::_1));

    // Timer to publish messages
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ZenohSessionExampleNode::publish_message, this));

    RCLCPP_INFO(this->get_logger(), "Zenoh session example node initialized successfully with pub/sub");
  }

private:
  std::shared_ptr<zenoh::Session> session_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ZenohSessionExampleNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}