#include "cobot/EstopAction.hpp"

#include <time.h>

namespace cobot {

EstopAction::EstopAction(const std::string &name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config) {}

BT::NodeStatus EstopAction::tick() {
  auto estopInput = getInput<bool>("estop");

  /** checks if input value exists */
  if (estopInput) {
    m_estop = estopInput.value();
  }  else { /** if not, fail safe to estop */
    m_estop = true;
    setOutput("speed", "STOP");
    return BT::NodeStatus::FAILURE;
  }

  if (m_estop) {
    setOutput("speed", "STOP");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList EstopAction::providedPorts() {
  return { BT::InputPort<bool>("estop"),
            BT::OutputPort<std::string>("speed")
          };
}

}  // namespace cobot
