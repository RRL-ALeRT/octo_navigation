#include <rclcpp/rclcpp.hpp>
#include "graph_exploration/graph_explorer.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor so subscription callbacks can fire while the
  // exploration thread is blocked waiting for a MoveBase action result.
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<graph_exploration::GraphExplorer>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
