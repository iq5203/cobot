#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace cobot {

/** 
 * ROS2 node that manages the tree provided by forwarding ROS2 topics to the
 *  behavior tree blackboard and forwarding behavior tree blackboard changes to 
 * ROS2 topics.  Assumes tree structure as found in behavior_trees/Cobot_BT.xml
*/

class CobotNode : public rclcpp::Node {
public:
  /** 
   * Initializes node.
   * Subscribes to ROS2 topic "estop"
   * Subscribes to ROS2 topic "range"
   * Sets up Publisher to ROS2 topic "speed"
   */
  explicit CobotNode(const std::shared_ptr<BT::Tree> &tree);

  /**
   * Ticks behavior tree and once complete, forwards behavior tree blackboard 
   * variable "speed" to ROS2 topic "speed"
   */
  void tick();

private:
  std::shared_ptr<BT::Tree> m_tree;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_estopSubscription;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr m_rangeSubscription;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_speedPublisher;

  /** 
   * Called when ROS2 "estop" topic is updated.  Forwards new value to 
   * blackboard variable "estop"
   */
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /** 
   * Called when ROS2 "range" topic is updated.  Forwards new value to 
   * blackboard variable "range"
   */
  void rangeCallback(const std_msgs::msg::Int16::SharedPtr msg);
};

} /* namespace cobot */
