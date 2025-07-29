#pragma once

#include "behaviortree_cpp/bt_factory.h"

namespace cobot {
class StopAction : public BT::SyncActionNode
{
public:
  StopAction(const std::string& name, const BT::NodeConfig &config);
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  int m_range;
};
}