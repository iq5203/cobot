#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp/action_node.h"


class EstopAction : public BT::SyncActionNode
{
public:
  EstopAction(const std::string &name, const BT::NodeConfig &config);

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
  
private:
    bool m_estop;
};