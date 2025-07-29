#include "cobot/Estop.hpp"

#include <time.h>

namespace cobot {

EstopAction::EstopAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config), 
    m_estop(false) {}

BT::NodeStatus EstopAction::tick() {
  auto estopInput = getInput<bool>("estop");

  if (estopInput) {
    m_estop = estopInput.value();
  } else {
    m_estop = true;
  } 
  
  if (m_estop) {
    setOutput("speed", "STOP");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList EstopAction::providedPorts()
{
  return { BT::InputPort<bool>("estop"),
            BT::OutputPort<std::string>("speed")
          };
}
}