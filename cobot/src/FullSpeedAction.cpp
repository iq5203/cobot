#include "cobot/FullSpeedAction.hpp"

namespace cobot {
FullSpeedAction::FullSpeedAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config), 
    m_range(0) {}

BT::NodeStatus FullSpeedAction::tick() {
  auto rangeInput = getInput<int>("range");

  if (rangeInput) {
    m_range = rangeInput.value();
  } else {
    m_range = 0;
  } 
  
  if (m_range < 800) {
    setOutput("speed", "STOP");
    return BT::NodeStatus::FAILURE;
  } else {
    setOutput("speed", "FULL_SPEED");
    return BT::NodeStatus::SUCCESS;
  }
}

BT::PortsList FullSpeedAction::providedPorts() {
  return { BT::InputPort<short>("range"),
            BT::OutputPort<std::string>("speed")
          };
}
}