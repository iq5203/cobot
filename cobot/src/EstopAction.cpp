#include "cobot/EstopAction.hpp"

#include <time.h>

namespace cobot {

EstopAction::EstopAction(const std::string &name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config),
m_logger(std::make_shared<rclcpp::Node>("logger")->get_logger()) {
}

BT::NodeStatus EstopAction::tick() {
  RCLCPP_DEBUG_STREAM(m_logger, "Ticking estop action");
  auto estopInput = getInput<bool>("estop");

  /** checks if input value exists */
  if (estopInput) {
    m_estop = estopInput.value();
  }  else { /** if not, fail safe to estop */
    m_estop = true;
    RCLCPP_ERROR_STREAM(m_logger, "No estop value found, triggering estop");
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
