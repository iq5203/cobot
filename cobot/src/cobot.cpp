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

  /** Provide node classes for use by behavior tree */
  factory.registerNodeType<cobot::EstopAction>("EstopAction");
  factory.registerNodeType<cobot::StopAction>("StopAction");
  factory.registerNodeType<cobot::SlowAction>("SlowAction");
  factory.registerNodeType<cobot::FullSpeedAction>("FullSpeedAction");

  /** Instantiate behavior tree */
  auto tree = std::make_shared<BT::Tree>(factory.createTreeFromFile(share_dir +
    "/behavior_trees/Cobot_BT.xml"));

  /** initialize ROS2 node */
  rclcpp::init(argc, argv);
  auto logNode = std::make_shared<rclcpp::Node>("logger");
  auto logger = logNode->get_logger();

  RCLCPP_DEBUG_STREAM(logger, "Started ROS2");
  auto cbNode = std::make_shared<cobot::CobotNode>(tree);

  /** Set initial blackboard values to estop by default */

  RCLCPP_DEBUG_STREAM(logger, "Initializing range to -1 and estop to true");

  tree->rootBlackboard()->set("range", -1);
  tree->rootBlackboard()->set("estop", true);

  /** Cycle time for ROS2 and Behavior Tree execution */
  rclcpp::Duration cycleTime{std::chrono::milliseconds(200)};


  while (true) {
    rclcpp::Time start = cbNode->get_clock()->now();
    /** Execute ROS2 cycle */
    RCLCPP_DEBUG_STREAM(logger, "Executing ROS2");
    rclcpp::spin_some(cbNode);

    RCLCPP_DEBUG_STREAM(logger, "Executing Behavior Tree");
    /** Execute Behavior Tree cycle */
    cbNode->tick();
    RCLCPP_DEBUG_STREAM(logger, "Completed executing cycle");
    /** Sleep for the rest of the cycle time */
    rclcpp::Time end = cbNode->get_clock()->now();
    rclcpp::Duration duration = end - start;
    if (duration < cycleTime) {
      std::chrono::milliseconds sleep = (cycleTime - duration).
      to_chrono<std::chrono::milliseconds>();

      RCLCPP_DEBUG_STREAM(logger, "Sleeping for " << sleep.count() <<
      "ms");

      rclcpp::sleep_for(sleep);
    } else {
      RCLCPP_ERROR_STREAM(logger,
      "Processing cycle took longer than "
      << cycleTime.to_chrono<std::chrono::milliseconds>().count()
      << "ms.  Cycle took "
      << duration.to_chrono<std::chrono::milliseconds>().count() << "ms");
    }
  }

  RCLCPP_DEBUG_STREAM(logger, "Shutting down system");
  /** Clean up ROS2 */
  rclcpp::shutdown();
  return 0;
}
