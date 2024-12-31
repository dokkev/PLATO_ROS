#include <pluginlib/class_loader.hpp>
// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <iostream>
#include <sstream>
#include <string>

#include "std_msgs/msg/string.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <std_msgs/msg/float64_multi_array.hpp> // Make sure to include the correct header for the message type
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#define NUM_SINGLE_FINGER_JOINT 3
#define NUM_ALL_FINGER_JOINT 9

using moveit::planning_interface::MoveGroupInterface;

using namespace moveit::core;

std::array<double, 12> receivedData;         // Data from Maestro joint space
std::vector<double> current_joint_positions; // Data from PLATO hand
bool shouldExit = false;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> torque_publisher = nullptr;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> joint_position_publisher = nullptr;

/* Function to initialize torque commands publisher */
void initTorquePublisher(rclcpp::Node::SharedPtr node)
{
    // Initialize publisher if it hasn't been already
    if (!torque_publisher)
    {
        torque_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/plato/plato_effort_controller/commands", 10);
        RCLCPP_INFO(node->get_logger(), "Torque commands publisher initialized.");
    }
}

/*
    Torque controller for finger joint, each torque controller control only one finger
 */
Eigen::VectorXd sendTorqueCommands(const Eigen::VectorXd &torques, rclcpp::Node::SharedPtr node, int finger_num)
{
    // Ensure the torque publisher is initialized
    initTorquePublisher(node);

    // Ensure the vector size matches your joints count
    if (torques.size() != NUM_ALL_FINGER_JOINT)
    {
        RCLCPP_ERROR(node->get_logger(), "Torque vector size does not match the expected size of 9");
        return torques;
    }

    // Populate your torque command message here
    std_msgs::msg::Float64MultiArray torque_command;
    torque_command.data.resize(9); // Ensure this matches the number of joints
    switch (finger_num)
    {
    case 1:
    {
        torque_command.data[0] = torques[0];
        torque_command.data[1] = torques[1];
        torque_command.data[2] = torques[2];
        break;
    }
    case 2:
    {
        torque_command.data[3] = torques[0];
        torque_command.data[4] = torques[1];
        torque_command.data[5] = torques[2];
        break;
    }
    case 3:
    {
        torque_command.data[6] = torques[0];
        torque_command.data[7] = torques[1];
        torque_command.data[8] = torques[2];
        break;
    }
    }

    // Publish the torque commands
    torque_publisher->publish(torque_command);
    RCLCPP_INFO(node->get_logger(), "Published torque commands to /plato/plato_effort_controller/commands");

    return torques;
}
/* Because the resting position between human's hand plato hand are diffferent, which
need a coversion in joint space. These are mainly offsets in joint spaces

 */
std::vector<double> handToPlatoConvert(){
    // Conpensation
    std::vector<double> converted_joint_positions(9);
    converted_joint_positions[0] = receivedData[1] - 50.0 *M_PI/180;
    converted_joint_positions[1] = receivedData[2];
    converted_joint_positions[2] = receivedData[3];
    converted_joint_positions[3] = receivedData[5];
    converted_joint_positions[4] = - receivedData[4];
    converted_joint_positions[5] = - receivedData[6];
    converted_joint_positions[6] = receivedData[9];
    converted_joint_positions[7] = - receivedData[8];
    converted_joint_positions[8] = - receivedData[10];

    return converted_joint_positions;

}

/* New joint control function to publish directly to the joint trajectory topic */
void publishJointTrajectory(std::vector<double> joint_positions, rclcpp::Node::SharedPtr node, const rclcpp::Logger &logger_)
{
    // Convert the plato's position
    joint_positions = handToPlatoConvert();
    // Create a new JointTrajectory message
    std_msgs::msg::Float64MultiArray joint_position_msg;
    joint_position_msg.data = {0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0,
                               0.0};
    

    // Ensure the size of joint_positions matches your joints count
    if (joint_positions.size() != joint_position_msg.data.size())
    {
        RCLCPP_ERROR(logger_, "Joint positions size does not match the expected size of 9");
        return;
    }

    joint_position_msg.data = joint_positions;

    // Publish the joint trajectory
    joint_position_publisher->publish(joint_position_msg);
    rclcpp::sleep_for(std::chrono::milliseconds (100));
    RCLCPP_INFO(logger_, "Published joint trajectory to /plato/plato_joint_controller/joint_trajectory");
}

// Callback func to getting data from Maestro
void messageCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() == 12)
    {
        for (size_t i = 0; i < 12; ++i)
        {
            receivedData[i] = msg->data[i];
        }
        // You can now use the receivedData array as needed
        RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Received data");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("subscriber_node"), "Received message has incorrect size");
    }
    rclcpp::sleep_for(std::chrono::microseconds(1));
}

// Callback func getting PLATO joints from JointState topic
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Assuming the order of joints in the message matches what you expect
    current_joint_positions = msg->position;
}

void robot_arm_init(MoveGroupInterface &move_group_interface_arm)
{
    // Set a target pose for the end effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.8;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    // Set the target pose for the end effector
    move_group_interface_arm.setPoseTarget(target_pose, "plato_base_link");
    move_group_interface_arm.move();
}

/*  Function to compute the position of a specified end-effector frame
    Input parameters: move_group is a kinematic chain of one finger.
                      Joint values are for one finger.
                      Name of end effector, could be "plato_f1_ee_frame" etc.
                      */
geometry_msgs::msg::PoseStamped computeEndEffectorPosition(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &joint_values,
    const std::string &end_effector_name)
{
    // Get the current robot state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Ensure the number of joint values matches the expected size
    const std::vector<std::string> &joint_names = move_group.getJointNames();
    if (joint_values.size() != joint_names.size())
    {
        throw std::runtime_error("The size of joint values does not match the expected number of joints in the move group");
    }

    // Set the new joint values for the computation
    current_state->setJointGroupPositions(move_group.getName(), joint_values);

    // Update the kinematic model with the new joint values
    current_state->update();

    // Get the transform for the specified end-effector frame
    const auto &tf = current_state->getGlobalLinkTransform(end_effector_name);

    // Convert the Eigen transform to a ROS Pose message
    geometry_msgs::msg::Pose pose_msg;
    tf2::convert(tf, pose_msg);

    // Now, create a PoseStamped message and fill in its fields
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = end_effector_name;                              // Set this to the appropriate reference frame
    pose_stamped.header.stamp = rclcpp::Node::make_shared("")->get_clock()->now(); // Set to the current time or appropriate timestamp
    pose_stamped.pose = pose_msg;                                                  // Set the pose

    // Return the PoseStamped message
    return pose_stamped;
}

// impedance control for each finger
/*
Input values:
    movegroup handle
    finger ID
    logger handle
    stiffness of gripper

Output value:
    torque for one finger motion.
 */
Eigen::VectorXd hand_impedance_control(
    moveit::planning_interface::MoveGroupInterface &move_group_interface_finger,
    int fingerID,
    const rclcpp::Logger &logger_,
    double stiffness_K)
{
    // Deterine which finger to control
    std::string ee_name;
    // std::string base_names;
    switch (fingerID)
    {
    case 1:
    {
        ee_name = "plato_f1_ee_frame";
        // base_names = "plato_f1_proxbracket";
        break;
    }
    case 2:
    {
        ee_name = "plato_f2_ee_frame";
        // base_names = "plato_f2_proxbracket";
        break;
    }
    case 3:
    {
        ee_name = "plato_f3_ee_frame";
        // base_names = "plato_f3_proxbracket";
        break;
    }
    }

    //  Go from receivedData to inputAngle
    /*  receivedData is from maestro
        inputAngle is to PLATO hand
         */
    std::vector<double> inputAngle;

    inputAngle[0] = receivedData[1];
    inputAngle[1] = receivedData[2];
    inputAngle[2] = receivedData[3];
    inputAngle[3] = receivedData[5];
    inputAngle[4] = receivedData[4];
    inputAngle[5] = receivedData[6];
    inputAngle[6] = receivedData[9];
    inputAngle[7] = receivedData[8];
    inputAngle[8] = receivedData[10];

    // Container for desired and real positions
    geometry_msgs::msg::PoseStamped desired_position, real_position;

    // Get current joint values and update move group for real positions
    move_group_interface_finger.setJointValueTarget(current_joint_positions);
    real_position = computeEndEffectorPosition(move_group_interface_finger,
                                               current_joint_positions,
                                               ee_name);

    // Compute desired end-effector positions based on input angles
    move_group_interface_finger.setJointValueTarget(inputAngle);
    desired_position = computeEndEffectorPosition(move_group_interface_finger,
                                                  inputAngle,
                                                  ee_name);

    // Calculate desired force based on stiffness and position error
    Eigen::Vector3d desired_force;
    // Define the real ee position
    Eigen::Vector3d real_pos(real_position.pose.position.x,
                             real_position.pose.position.y,
                             real_position.pose.position.z);
    // Define the desire ee position
    Eigen::Vector3d desired_pos(desired_position.pose.position.x,
                                desired_position.pose.position.y,
                                desired_position.pose.position.z);

    desired_force = stiffness_K * (desired_pos - real_pos);

    // Convert forces to joint torques using the Jacobian
    // auto kinematic_model = move_group_interface_finger.getRobotModel();
    auto kinematic_state = move_group_interface_finger.getCurrentState();
    std::vector<double> joint_torques(move_group_interface_finger.getCurrentJointValues().size(), 0.0);

    auto *joint_model_group = kinematic_state->getJointModelGroup(move_group_interface_finger.getName());
    const auto *ee_link_model = kinematic_state->getRobotModel()->getLinkModel(ee_name);

    Eigen::MatrixXd jacobian;

    // Get the Jacobian for the end-effector
    bool success = kinematic_state->getJacobian(
        joint_model_group,
        ee_link_model,
        Eigen::Vector3d::Zero(),
        jacobian);

    if (!success)
    {
        RCLCPP_ERROR(logger_, "Failed to compute Jacobian for %s", ee_name.c_str());
    }

    // Calculate torques for this end-effector: tau = J^T * F
    Eigen::VectorXd torques_for_ee = jacobian.transpose() * desired_force;

    return torques_for_ee;
}

void setupJointStateSubscriber(rclcpp::Node::SharedPtr node)
{
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, jointStateCallback);
}

void hand_joints_control(MoveGroupInterface &move_group_interface_hand, const rclcpp::Logger logger_)
{
/*         
        std::vector<double> initial_joints = {38.0 * (M_PI / 180), -60.0 * (M_PI / 180), -15.0 * (M_PI / 180),
                                              -15.0 * (M_PI / 180), 60.0 * (M_PI / 180), -15.0 * (M_PI / 180),
                                              -15.0 * (M_PI / 180), 60.0 * (M_PI / 180), -15.0 * (M_PI / 180)};
 */
    std::vector<double> initial_joints = {0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0};

    std::vector<double> final_joints = {35.0 * (M_PI / 180), 10.0 * (M_PI / 180), 55.0 * (M_PI / 180),
                                        -15.0 * (M_PI / 180), 20.0 * (M_PI / 180), -111.0 * (M_PI / 180),
                                        -15.0 * (M_PI / 180), 20.0 * (M_PI / 180), -111.0 * (M_PI / 180)};

    std::vector<double> input_max = {0.2, 0.23 * 2.5, 1.8,
                                     -0.4, -0.17 * 2.5, 1.89,
                                     -0.4, -0.17 * 2.5, 1.89}; // max values of your input range
    std::vector<double> current_joint_positions = move_group_interface_hand.getCurrentJointValues();

    // Modify the desired position
    std::vector<double> desired_rotation_angle;
    std::vector<double> inputAngle;

    inputAngle = current_joint_positions;
    desired_rotation_angle = current_joint_positions;

    inputAngle[0] = receivedData[1];
    inputAngle[1] = receivedData[2];
    inputAngle[2] = receivedData[3];
    inputAngle[3] = receivedData[5];
    inputAngle[4] = receivedData[4];
    inputAngle[5] = receivedData[6];
    inputAngle[6] = receivedData[9];
    inputAngle[7] = receivedData[8];
    inputAngle[8] = receivedData[10];

    for (size_t i = 0; i < final_joints.size(); ++i)
    {
        // Assuming your input_min is 0 for all joints
        double input_min = 0;
        double input = inputAngle[i]; // Your input value for each joint

        // Apply the linear transformation formula
        desired_rotation_angle[i] = initial_joints[i] +
                                    (final_joints[i] - initial_joints[i]) *
                                        (input - input_min) / (input_max[i] - input_min);
    }
    desired_rotation_angle[0] *= 20.0;
    desired_rotation_angle[1] *= 2.0;
    desired_rotation_angle[2] *= 3.0;

    desired_rotation_angle[3] *= 2.0;
    desired_rotation_angle[4] *= 4.0;
    desired_rotation_angle[5] *= 2.0;

    desired_rotation_angle[6] *= 2.0;
    desired_rotation_angle[7] *= 4.0;
    desired_rotation_angle[8] *= 2.0;

    // Set the target joint positions
    move_group_interface_hand.setJointValueTarget(desired_rotation_angle);

    // Set the planning time (e.g., 2.0 seconds)
    move_group_interface_hand.setPlanningTime(2.0);

    // Adjust velocity scaling to speed up the motion (e.g., set to 2.0 for double speed)
    move_group_interface_hand.setMaxVelocityScalingFactor(2.0);

    // Adjust acceleration scaling if needed (e.g., set to 2.0 for double acceleration)
    move_group_interface_hand.setMaxAccelerationScalingFactor(2.0);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_interface_hand.plan(plan))
    {
        move_group_interface_hand.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger_, "Failed to plan to the desired joint position");
    }

    return;
}

std::vector<double> parseVectorString(const std::string &input)
{
    std::vector<double> values;

    // Check if the input string starts with '{' and ends with '}'
    if (input.empty() || input.front() != '{' || input.back() != '}')
    {
        // Invalid input format
        return values;
    }

    // Remove the opening and closing braces from the input
    std::string trimmedInput = input.substr(1, input.size() - 2);

    // Create a stringstream to parse the values
    std::stringstream ss(trimmedInput);
    std::string token;

    // Split the input by commas and store the values in a vector
    while (std::getline(ss, token, ','))
    {
        try
        {
            // Convert each token to a double and add it to the vector
            double value = std::stod(token);
            values.push_back(value);
        }
        catch (const std::invalid_argument &e)
        {
            // Ignore invalid tokens
        }
    }

    return values;
}

int main(int argc, char *argv[])
{
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
    std::thread spinner = std::thread([&executor]()
                                      { executor.spin(); });

    // Create the MoveIt MoveGroup Interfaces
    // auto move_group_interface_arm = MoveGroupInterface(node, "optimo_arm");
    // auto move_group_interface_hand = MoveGroupInterface(node, "plato_hand");
    // auto move_group_interface_f1 = MoveGroupInterface(node, "plato_f1");
    // auto move_group_interface_f2 = MoveGroupInterface(node, "plato_f2");
    // auto move_group_interface_f3 = MoveGroupInterface(node, "plato_f3");
    // Get current pose of plato_base_link (EE of Optimo)
    // geometry_msgs::msg::PoseStamped current_pose = move_group_interface_arm.getCurrentPose("plato_base_link");

    // subscribe the hand posture from the human's hand estimation
    auto subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>("hand_joint_space",
                                                                                  10,
                                                                                  messageCallback);
    joint_position_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/plato/plato_position_controller/commands", 1);
    // initial postions
    // robot_arm_init(move_group_interface_arm);
    // setupJointStateSubscriber(node); // get joint values from encoders
    while (rclcpp::ok() && !shouldExit)
    {

/* 
        receivedData =
        {35.0 * (M_PI / 180), -10.0 * (M_PI / 180), 55.0 * (M_PI / 180),
                                        -15.0 * (M_PI / 180), 40.0 * (M_PI / 180), -111.0 * (M_PI / 180),
                                        -15.0 * (M_PI / 180), 34.0 * (M_PI / 180), -111.0 * (M_PI / 180)};
                                         */

        // hand_joints_control(move_group_interface_hand, logger);
        std::vector<double> joints_dersired =
            {-0.0, -0.1, -0.1,
             0.1, 0.5, 0.5,
             -0.1, 0.5, 0.5};
        publishJointTrajectory(joints_dersired, node, logger);
            
        // rclcpp::sleep_for(std::chrono::seconds(2));
        // Eigen::VectorXd torque_f1 = hand_impedance_control(move_group_interface_f1, 1,
        //                                                    logger, 20);
        // sendTorqueCommands(torque_f1, node, 1);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    // Shutdown ROS
    spinner.join();
    rclcpp::shutdown();

    return 0;
}