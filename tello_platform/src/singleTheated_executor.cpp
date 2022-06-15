#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "as2_core/node.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/aerial_platform.hpp"

int main (int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node1 = std::make_shared<rclcpp::Node>("node1");
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node1);
    executor.spin();
    rclcpp::shutdown();
}