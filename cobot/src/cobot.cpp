#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "cobot/EstopAction.hpp"
#include "cobot/StopAction.hpp"
#include "cobot/SlowAction.hpp"
#include "cobot/FullSpeedAction.hpp"
#include "cobot/CobotNode.hpp"

int main(int argc, char ** argv) {
  std::string share_dir = ament_index_cpp::get_package_share_directory("cobot");
  BT::BehaviorTreeFactory factory;

  rclcpp::init(argc, argv);

  factory.registerNodeType<cobot::EstopAction>("EstopAction");
  factory.registerNodeType<cobot::StopAction>("StopAction");
  factory.registerNodeType<cobot::SlowAction>("SlowAction");
  factory.registerNodeType<cobot::FullSpeedAction>("FullSpeedAction");

  auto tree = std::make_shared<BT::Tree>(factory.createTreeFromFile(share_dir +
    "/behavior_trees/Cobot_BT.xml"));

  auto cbNode = std::make_shared<cobot::CobotNode>(tree);

  tree->rootBlackboard()->set("range", -1);
  tree->rootBlackboard()->set("estop", true);

  rclcpp::Duration cycleTime{std::chrono::milliseconds(200)};

  while (true) {
    rclcpp::Time start = cbNode->get_clock()->now();
    rclcpp::spin_some(cbNode);
    cbNode->tick();
    rclcpp::Time end = cbNode->get_clock()->now();
    rclcpp::Duration duration = end - start;
    if (duration < cycleTime) {
      std::chrono::milliseconds sleep = (cycleTime - duration).
      to_chrono<std::chrono::milliseconds>();
      rclcpp::sleep_for(sleep);
    }
  }

  rclcpp::shutdown();
  return 0;
}
