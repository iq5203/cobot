#include "cobot/CobotNode.hpp"

CobotNode::CobotNode(std::shared_ptr<BT::Tree> tree): 
Node("cobot"),
m_tree(tree) {
    m_estopSubscription = create_subscription<std_msgs::msg::Bool>(
      "estop", 10, std::bind(&CobotNode::estopCallback, this, std::placeholders::_1));

    m_rangeSubscription = create_subscription<std_msgs::msg::Int16>(
      "range", 10, std::bind(&CobotNode::rangeCallback, this, std::placeholders::_1));

    m_speedPublisher = create_publisher<std_msgs::msg::String>("speed", 10);
}

void CobotNode::tick() {
    m_tree->tickExactlyOnce();
    std::string speed;
    m_tree->rootBlackboard()->get("speed", speed);
    auto message = std_msgs::msg::String();
    message.data = speed;
    m_speedPublisher->publish(message);
}

void CobotNode::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    m_tree->rootBlackboard()->set("estop", msg->data);
}

void CobotNode::rangeCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    m_tree->rootBlackboard()->set("range", msg->data);
}
