#include "cobot/StopAction.hpp"

namespace cobot {
StopAction::StopAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config) {}

// You must override the virtual function tick()
BT::NodeStatus StopAction::tick() {
  auto rangeInput = getInput<int16_t>("range");

  /** checks if input value exists */
  if (rangeInput) {
    m_range = rangeInput.value();
  } else { /** if not, fail safe to stop */
    m_range = 0;
    setOutput("speed", "STOP");
    return BT::NodeStatus::FAILURE;
  }

  if (m_range < 400) {
    setOutput("speed", "STOP");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList StopAction::providedPorts() {
  return { BT::InputPort<int16_t>("range"),
            BT::OutputPort<std::string>("speed")
          };
}

}  // namespace cobot
