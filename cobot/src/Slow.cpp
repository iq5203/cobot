#include "cobot/Slow.hpp"
#include <time.h>

namespace cobot {
SlowAction::SlowAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config), 
    m_range(0) {}

BT::NodeStatus SlowAction::tick() {
  auto rangeInput = getInput<int>("range");

  if (rangeInput) {
    m_range = rangeInput.value();
  } else {
    m_range = 0;
  } 
  
  if (m_range < 400) {
    setOutput("speed", "STOP");
    return BT::NodeStatus::FAILURE;
  } else if (m_range < 800) {
    setOutput("speed", "SLOW");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList SlowAction::providedPorts() {
  return { BT::InputPort<short>("range"),
            BT::OutputPort<std::string>("speed")
          };
}
}