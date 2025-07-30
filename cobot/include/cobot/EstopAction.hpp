#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace cobot {

/**
 * When ticked, this action checks to see if an estop has been set to the 
 * blackboard input "estop". If an estop signal has been set, output a speed of 
 * STOP and return success to indicate speed has been set.  Otherwise return 
 * failure.  If no input exists, assume estop is on.
 */
class EstopAction : public BT::SyncActionNode {
public:
  EstopAction(const std::string &name, const BT::NodeConfig &config);

  BT::NodeStatus tick() override;

  /**
   * Assumes a boolean "estop" input port configured in behavior tree.
   * Assumes a string "speed" output port configured in behavior tree.
   */
  static BT::PortsList providedPorts();

private:
    bool m_estop{true};
};

}  // namespace cobot
