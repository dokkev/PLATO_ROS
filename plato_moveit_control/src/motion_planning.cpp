#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[]){
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);

    // Declare a node
    std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("motion_planning",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
    true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("motion_planning");

    // SingleThreadedExecutor to spin the node in a single thread
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });


    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "optimo_arm");

    // Get current pose of plato_base_link (EE of Optimo)
    geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose("plato_base_link");

    // Print the current pose of the end effector
    RCLCPP_INFO(logger, " Current Pose: Point [ %f %f %f ]  Quaternion [ %f %f %f %f ]", 
    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
    current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,current_pose.pose.orientation.w );

    
    // Option 1: Set a target pose for the end effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    // Set the target pose for the end effector
    move_group_interface.setPoseTarget(target_pose, "plato_base_link");
    move_group_interface.move();

    // Option 2: Set a target pose from current pose
    target_pose = current_pose.pose;
    target_pose.position.z += 0.1; // add 10 cm to current z

    // Set the target pose for the end effector
    move_group_interface.setPoseTarget(target_pose, "plato_base_link");
    move_group_interface.move();

    // Shutdown ROS
    spinner.join();
    rclcpp::shutdown();

    return 0;
}