#pragma once

#include "behaviortree_cpp/bt_factory.h"

class SlowAction : public BT::SyncActionNode
{
public:
  SlowAction(const std::string &name, const BT::NodeConfig &config);

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  int m_range;
};