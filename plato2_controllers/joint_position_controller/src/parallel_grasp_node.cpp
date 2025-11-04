/**
 * @file parallel_grasp_node.cpp
 * @brief ROS2 node for parallel grasp control
 *
 * Subscribes to /plato2/parallel_grasp_controller/command (Float64)
 * Maps grasp width command (u) to joint positions (q3, q4, q5, q6)
 * Publishes to /plato2/joint_impedance_controller/commands
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include "joint_position_controller/parallel_grasp_controller.hpp"

using std::placeholders::_1;

class ParallelGraspNode : public rclcpp::Node
{
public:
  ParallelGraspNode()
  : Node("parallel_grasp_node")
  {
    initialize_subscriber();
    initialize_publisher();
    initialize_default_gains();

    RCLCPP_INFO(this->get_logger(), "Parallel grasp node initialized successfully");
  }

private:
  // Constants
  static constexpr size_t NUM_JOINTS = 8;
  static constexpr size_t LEGACY_COMMAND_SIZE = 4;
  static constexpr size_t GRASP_JOINT_START_INDEX = 2;  // Grasp joints start at index 2
  static constexpr size_t QUEUE_SIZE = 10;
  static constexpr double DEFAULT_STIFFNESS = 1.0;
  static constexpr double DEFAULT_DAMPING = 0.1;

  /**
   * @brief Initialize subscriber for grasp commands
   */
  void initialize_subscriber()
  {
    command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/plato2/parallel_grasp_controller/command",
      QUEUE_SIZE,
      std::bind(&ParallelGraspNode::grasp_command_callback, this, _1)
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
   * @brief Callback for grasp command messages
   * @param msg Float64 message containing grasp width command
   */
  void grasp_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double grasp_command = msg->data;
    const auto joint_commands = controller_.get_commands(grasp_command);

    auto impedance_cmd = create_impedance_command(grasp_command, joint_commands);
    impedance_pub_->publish(impedance_cmd);
  }

  /**
   * @brief Create impedance command message from joint commands
   * @param grasp_command Original grasp width command
   * @param joint_commands Joint position commands from controller
   * @return Complete impedance command message
   */
  plato2_interfaces::msg::ImpedanceCommands create_impedance_command(
    double grasp_command,
    const std::vector<double>& joint_commands)
  {
    plato2_interfaces::msg::ImpedanceCommands cmd;

    // Handle full 8-joint command format
    if (joint_commands.size() >= NUM_JOINTS) {
      cmd.position = joint_commands;
      log_full_command(grasp_command, joint_commands);
    }
    // Handle legacy 4-joint command format (map to indices 2-5)
    else {
      cmd.position = create_legacy_command_vector(joint_commands);
      log_legacy_command(grasp_command, joint_commands);
    }

    // Set other command fields
    cmd.velocity = std::vector<double>(NUM_JOINTS, 0.0);
    cmd.stiffness = default_stiffness_;
    cmd.damping = default_damping_;
    cmd.effort_ff = std::vector<double>(NUM_JOINTS, 0.0);

    return cmd;
  }

  /**
   * @brief Create position vector for legacy 4-joint command format
   * @param joint_commands 4-element joint command vector
   * @return 8-element position vector with commands at indices 2-5
   */
  std::vector<double> create_legacy_command_vector(const std::vector<double>& joint_commands)
  {
    std::vector<double> positions(NUM_JOINTS, 0.0);

    if (joint_commands.size() >= LEGACY_COMMAND_SIZE) {
      for (size_t i = 0; i < LEGACY_COMMAND_SIZE; ++i) {
        positions[GRASP_JOINT_START_INDEX + i] = joint_commands[i];
      }
    }

    return positions;
  }

  /**
   * @brief Log full 8-joint command
   */
  void log_full_command(double grasp_command, const std::vector<double>& joint_commands)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Grasp command: %.4f -> positions[2-5]: %.6f, %.6f, %.6f, %.6f",
      grasp_command,
      joint_commands[2], joint_commands[3], joint_commands[4], joint_commands[5]
    );
  }

  /**
   * @brief Log legacy 4-joint command
   */
  void log_legacy_command(double grasp_command, const std::vector<double>& joint_commands)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Grasp command: %.4f -> q3: %.6f, q4: %.6f, q5: %.6f, q6: %.6f",
      grasp_command,
      joint_commands.size() > 0 ? joint_commands[0] : 0.0,
      joint_commands.size() > 1 ? joint_commands[1] : 0.0,
      joint_commands.size() > 2 ? joint_commands[2] : 0.0,
      joint_commands.size() > 3 ? joint_commands[3] : 0.0
    );
  }

  // Member variables
  ParallelGraspController controller_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
  std::vector<double> default_stiffness_;
  std::vector<double> default_damping_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParallelGraspNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
