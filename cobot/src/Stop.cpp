#include "cobot/Stop.hpp"

#include <time.h>

namespace cobot {
StopAction::StopAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config), 
    m_range(0) {}

// You must override the virtual function tick()
BT::NodeStatus StopAction::tick() {
  auto rangeInput = getInput<int16_t>("range");

  if (rangeInput) {
    m_range = rangeInput.value();
  } else {
    m_range = 0;
  } 
  
  if (m_range < 400) {
    setOutput("speed", "STOP");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }  
}

BT::PortsList StopAction::providedPorts()
{
  return { BT::InputPort<int16_t>("range"),
            BT::OutputPort<std::string>("speed")
          };
}
}