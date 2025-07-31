#include "cobot/CobotNode.hpp"

namespace cobot {

CobotNode::CobotNode(const std::shared_ptr<BT::Tree> &tree):
  Node("cobot"),
  m_tree(tree) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Subscribing to estop");
    m_estopSubscription = create_subscription<std_msgs::msg::Bool>(
      "estop", 10, std::bind(&CobotNode::estopCallback, this,
      std::placeholders::_1));

      RCLCPP_DEBUG_STREAM(get_logger(), "Subscribing to range");
      m_rangeSubscription = create_subscription<std_msgs::msg::Int16>(
      "range", 10, std::bind(&CobotNode::rangeCallback, this,
        std::placeholders::_1));

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing to speed");
        m_speedPublisher = create_publisher<std_msgs::msg::String>("speed", 10);

        RCLCPP_DEBUG_STREAM(get_logger(), "Publishing to estop");
        m_estopPublisher = create_publisher<std_msgs::msg::String>(
          "estopped", 10);
}

void CobotNode::tick() {
  RCLCPP_DEBUG_STREAM(get_logger(), "Executing behavior tree");
  m_tree->tickExactlyOnce();

  auto estopMessage = std_msgs::msg::String();
  if (m_estop) {
    RCLCPP_WARN_STREAM(get_logger(), "Publishing estop on");
    estopMessage.data = "ESTOP";
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Publishing estop off");
    estopMessage.data = "RUN";
  }
  m_estopPublisher->publish(estopMessage);

  std::string speed;
  bool hasSpeed = m_tree->rootBlackboard()->get("speed", speed);
  auto speedMessage = std_msgs::msg::String();
  if (hasSpeed) {
    RCLCPP_DEBUG_STREAM(get_logger(), "behavior tree produced cobot speed "
    << speed);
    speedMessage.data = speed;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(),
    "No speed in behavior tree blackboard after executing behavior tree"
    << " publishing stop by default");
    speedMessage.data = "STOP";
  }

  m_speedPublisher->publish(speedMessage);
}

void CobotNode::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  m_estop = msg->data;
  m_tree->rootBlackboard()->set("estop", msg->data);
}

void CobotNode::rangeCallback(const std_msgs::msg::Int16::SharedPtr msg) {
  m_tree->rootBlackboard()->set("range", msg->data);
}

}  // namespace cobot
