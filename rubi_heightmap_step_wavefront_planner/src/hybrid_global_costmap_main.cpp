#include <cstdlib>
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // The upstream Humble executable constructs Costmap2DROS("costmap"), which
  // fixes both its name and namespace to /costmap/costmap. Use the public Nav2
  // constructor so the lifecycle and topic FQN matches the Hybrid contract.
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    rclcpp::NodeOptions());
  rclcpp::spin(costmap->get_node_base_interface());

  // Costmap2DROS owns plugin objects whose Humble destructor crashes after the
  // global SIGINT handler has already invalidated the ROS context. Lifecycle
  // cleanup is handled by the manager; bypass only process-exit destruction.
  std::_Exit(EXIT_SUCCESS);
}
