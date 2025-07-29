#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp/bt_factory.h"

class CobotNode : public rclcpp::Node {
public:
    CobotNode(std::shared_ptr<BT::Tree> tree);
    void tick();
private:
    std::shared_ptr<BT::Tree> m_tree;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_estopSubscription;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr m_rangeSubscription;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_speedPublisher;
    
    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rangeCallback(const std_msgs::msg::Int16::SharedPtr msg);
};

