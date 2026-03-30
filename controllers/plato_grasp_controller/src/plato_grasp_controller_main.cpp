#include <memory>

#include "plato_grasp_controller/plato_grasp_controller_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plato_grasp_controller::PlatoGraspControllerNode>());
  rclcpp::shutdown();
  return 0;
}
