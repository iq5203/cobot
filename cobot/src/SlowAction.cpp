#include "cobot/SlowAction.hpp"

namespace cobot {
SlowAction::SlowAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config) {}

BT::NodeStatus SlowAction::tick() {
  auto rangeInput = getInput<int>("range");

  /** checks if input value exists */
  if (rangeInput) {
    m_range = rangeInput.value();
  } else { /** if not, fail safe to stop */
    m_range = 0;
    setOutput("speed", "STOP");
    return BT::NodeStatus::FAILURE;
  }

  if (m_range < 800) {
    setOutput("speed", "SLOW");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList SlowAction::providedPorts() {
  return { BT::InputPort<int32_t>("range"),
            BT::OutputPort<std::string>("speed")
          };
}

}  // namespace cobot
