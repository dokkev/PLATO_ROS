/**
 * @file parallel_grasp_node.cpp
 * @brief ROS2 node for parallel grasp control
 *
 * Subscribes to /plato2/parallel_grasp_controller/command (Float64)
 * Subscribes to /joint_states to get current positions (for backdrivable joints)
 * Maps grasp width command (u) to joint positions (q3, q4, q5, q6)
 * Publishes to /plato2/joint_impedance_controller/commands
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include "joint_position_controller/parallel_grasp_controller.hpp"

using std::placeholders::_1;

class ParallelGraspNode : public rclcpp::Node
{
public:
  ParallelGraspNode()
  : Node("parallel_grasp_node"),
    current_joint_positions_(NUM_JOINTS, 0.0)
  {
    initialize_subscribers();
    initialize_publisher();
    initialize_default_gains();

    RCLCPP_INFO(this->get_logger(), "Parallel grasp node initialized successfully");
  }

private:
  // Constants
  static constexpr size_t NUM_JOINTS = 8;
  static constexpr size_t QUEUE_SIZE = 10;
  static constexpr double DEFAULT_STIFFNESS = 1.0;
  static constexpr double DEFAULT_DAMPING = 0.1;

  /**
   * @brief Initialize subscribers for grasp commands and joint states
   */
  void initialize_subscribers()
  {
    command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/plato2/parallel_grasp_controller/command",
      QUEUE_SIZE,
      std::bind(&ParallelGraspNode::grasp_command_callback, this, _1)
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/plato2/joint_states",
      QUEUE_SIZE,
      std::bind(&ParallelGraspNode::joint_state_callback, this, _1)
    );
  }

  /**
   * @brief Initialize publisher for impedance commands
   */
  void initialize_publisher()
  {
    rclcpp::QoS qos_profile{rclcpp::KeepLast(QUEUE_SIZE)};
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    impedance_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
      "/plato2/joint_impedance_controller/commands",
      qos_profile
    );
  }

  /**
   * @brief Initialize default impedance gains
   */
  void initialize_default_gains()
  {
    default_stiffness_ = std::vector<double>(NUM_JOINTS, DEFAULT_STIFFNESS);
    default_damping_ = std::vector<double>(NUM_JOINTS, DEFAULT_DAMPING);
  }

  /**
   * @brief Callback for joint state messages
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < NUM_JOINTS || msg->name.size() < NUM_JOINTS) {
      RCLCPP_WARN(this->get_logger(), "Received incomplete joint state");
      return;
    }

    // Reorder positions to match joint1-joint8 order
    std::vector<double> ordered_positions(NUM_JOINTS, 0.0);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto& name = msg->name[i];
      if (name.length() >= 6 && name.substr(0, 5) == "joint") {
        int joint_num = std::stoi(name.substr(5)) - 1;  // joint1 -> index 0
        if (joint_num >= 0 && joint_num < static_cast<int>(NUM_JOINTS)) {
          ordered_positions[joint_num] = msg->position[i];
        }
      }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joint_positions_ = ordered_positions;
  }

  /**
   * @brief Callback for grasp command messages
   * @param msg Float64 message containing grasp width command
   */
  void grasp_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::vector<double> current_positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_positions = current_joint_positions_;
    }

    // Wait for joint states before processing commands
    if (current_positions.size() < NUM_JOINTS) {
      RCLCPP_WARN(this->get_logger(), "Waiting for joint states before processing commands");
      return;
    }

    // Debug: print current positions
    RCLCPP_INFO(this->get_logger(), "Current joint positions: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
      current_positions[0], current_positions[1], current_positions[2], current_positions[3],
      current_positions[4], current_positions[5], current_positions[6], current_positions[7]);

    const auto joint_commands = controller_.get_commands(msg->data, current_positions);
    auto impedance_cmd = create_impedance_command(msg->data, joint_commands);
    impedance_pub_->publish(impedance_cmd);
  }

  /**
   * @brief Create impedance command message from joint commands
   * @param grasp_command Original grasp width command
   * @param joint_commands Joint position commands from controller (8 elements expected)
   * @return Complete impedance command message
   */
  plato2_interfaces::msg::ImpedanceCommands create_impedance_command(
    double grasp_command,
    const std::vector<double>& joint_commands)
  {
    plato2_interfaces::msg::ImpedanceCommands cmd;

    cmd.position = joint_commands;
    cmd.velocity = std::vector<double>(NUM_JOINTS, 0.0);
    cmd.stiffness = default_stiffness_;
    cmd.damping = default_damping_;
    cmd.effort_ff = std::vector<double>(NUM_JOINTS, 0.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Grasp command: %.4f -> positions[2-5]: [%.6f, %.6f, %.6f, %.6f]",
      grasp_command,
      joint_commands[2], joint_commands[3], joint_commands[4], joint_commands[5]
    );

    return cmd;
  }

  // Member variables
  ParallelGraspController controller_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
  std::vector<double> default_stiffness_;
  std::vector<double> default_damping_;

  // Current joint state (protected by mutex)
  std::mutex state_mutex_;
  std::vector<double> current_joint_positions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParallelGraspNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
