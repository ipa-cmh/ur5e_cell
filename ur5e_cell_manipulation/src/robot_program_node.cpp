#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include "ur5e_cell_manipulation/manipulation_behaviour_manager.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_program_node", "", options);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
  });
  rclcpp::sleep_for(std::chrono::seconds(1));
  ManipulationBehaviorManager manager(node, "arm");
  manager.start_behaviour();
  rclcpp::sleep_for(std::chrono::seconds(5));
  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}
