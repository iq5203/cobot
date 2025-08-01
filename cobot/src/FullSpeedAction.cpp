#include "cobot/FullSpeedAction.hpp"

namespace cobot {

FullSpeedAction::FullSpeedAction(const std::string &name,
  const BT::NodeConfig &config):
    BT::SyncActionNode(name, config),
    m_logger(std::make_shared<rclcpp::Node>("logger")->get_logger()) {
    }

BT::NodeStatus FullSpeedAction::tick() {
  RCLCPP_DEBUG_STREAM(m_logger, "Ticking full speed action");
  auto rangeInput = getInput<int>("range");

  /** checks if input value exists */
  if (rangeInput) {
    m_range = rangeInput.value();
  }  else { /** if not, fail safe to stop */
    RCLCPP_ERROR_STREAM(m_logger, "No range value found assuming STOP");
    m_range = 0;
    setOutput("speed", "STOP");
    return BT::NodeStatus::FAILURE;
  }

  if (m_range >= 800) {
    setOutput("speed", "FULL_SPEED");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList FullSpeedAction::providedPorts() {
  return { BT::InputPort<int16_t>("range"),
            BT::OutputPort<std::string>("speed")
          };
}

}  // namespace cobot
