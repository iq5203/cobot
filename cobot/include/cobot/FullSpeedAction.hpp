#pragma once

#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace cobot {

/**
 * When ticked, this action checks the blackboard input "range". If the range 
 * is greater than 800, output a speed of FULL_SPEED to blackboard port "speed" 
 * and return success to indicate speed has been set.  Otherwise return 
 * failure.  If no input, output a speed of STOP to blackboard port "speed" 
 */

class FullSpeedAction : public BT::SyncActionNode {
public:
  FullSpeedAction(const std::string &name, const BT::NodeConfig &config);

  BT::NodeStatus tick() override;

/**
 * Assumes an integer "range" input port configured in behavior tree.
 * Assumes a string "speed" output port configured in behavior tree.
 */
static BT::PortsList providedPorts();

private:
  int m_range{0};
};

}  // namespace cobot
