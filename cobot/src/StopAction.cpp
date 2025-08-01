#include "cobot/StopAction.hpp"

namespace cobot {
StopAction::StopAction(const std::string &name, const BT::NodeConfig &config) :
    BT::SyncActionNode(name, config),
    m_logger(std::make_shared<rclcpp::Node>("logger")->get_logger()) {
    }

// You must override the virtual function tick()
BT::NodeStatus StopAction::tick() {
  RCLCPP_DEBUG_STREAM(m_logger, "Ticking stop action");
  auto rangeInput = getInput<int16_t>("range");

  /** checks if input value exists */
  if (rangeInput) {
    m_range = rangeInput.value();
  } else { /** if not, fail safe to stop */
    RCLCPP_ERROR_STREAM(m_logger, "No range value found, triggering STOP");
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
