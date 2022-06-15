#include "tello_platform.hpp"

int main(int argc, char * argv[])
{  
  rclcpp::init(argc, argv);

  auto telloNode = std::make_shared<TelloPlatform>();
  telloNode->preset_loop_frequency(300);

  as2::spinLoop(telloNode);

  /*
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node1);
  executor.spin();*/

  //as2::spinLoop(node);
 

  rclcpp::shutdown();
  return 0;
}
