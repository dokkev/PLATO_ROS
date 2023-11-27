#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
static const std::string PLANNING_GROUP = "optimo_arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");


class PlanArm : public rclcpp::Node{
  public:
    PlanArm();
    moveit::planning_interface::MoveGroupInterface move_group;
    std::vector<double> joint_group_positions;
    geometry_msgs::msg::PoseStamped current_pose;
};


PlanArm::PlanArm(): Node("moveit_control"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP){
  RCLCPP_INFO(LOGGER, "INITIALIZING!");

  // Option 1: Send a joint value target
  joint_group_positions = {0.0, 3.14, 0.0, 0.0, 0.0, 0.0 ,0.0};
  this->move_group.setJointValueTarget(joint_group_positions);
  this->move_group.move();

  // Option 2: Send a pose target
  geometry_msgs::msg::Pose target_pose2;
  target_pose2.orientation.w = 1.0;  // Example orientation
  target_pose2.position.x = 0.28;    // Example position
  target_pose2.position.y = -0.2;
  target_pose2.position.z = 0.5;
  this->move_group.setPoseTarget(target_pose2);
  this->move_group.move();
  
  // Option 3: Send a pose target relative to Current Pose
  current_pose = this->move_group.getCurrentPose("plato_base_link");
  geometry_msgs::msg::Pose target_pose3 = current_pose.pose;
  target_pose3.position.z = current_pose.pose.position.z + 0.1; // add 10 cm to current z
  this->move_group.setPoseTarget(target_pose3);
  this->move_group.move();
}



int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto move_group_node = std::make_shared<PlanArm>();
  RCLCPP_INFO(LOGGER, "In main");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.spin();


  rclcpp::shutdown();
  return 0;
}