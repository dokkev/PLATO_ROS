#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "plato_interfaces/msg/impedance_commands.hpp"
#include "sdr_grasp_msgs/msg/tactile.hpp"
#include "sdr_grasp_msgs/msg/grasp_control.hpp"
#include "grasp_srvs/srv/start_adaptive_grasp_ctrl.hpp"
#include "grasp_srvs/srv/start_constant_grasp_ctrl.hpp"
#include "grasp_srvs/srv/stop_grasp_ctrl.hpp"
#include "parallel_grasp_controller/parallel_grasp_controller.hpp"

/**
 * @file grasp_node.hpp
 * @brief Simplified force-controlled parallel grasp with state machine
 *
 * This node coordinates between grasp_force_ctrl (which calculates forces)
 * and the parallel grasp controller (which controls positions).
 *
 * States:
 *  - kIdle: Default open-loop position control
 *  - kContact: Contact detection and confirmation
 *  - kAdaptive: Slip-based adaptive force control (uses grasp_force_ctrl)
 *  - kForce: User-specified force tracking (uses grasp_force_ctrl with fixed force)
 *
 * Architecture:
 *  - Subscribes to desired forces from grasp_force_ctrl
 *  - Calls services on grasp_force_ctrl to switch modes
 *  - Converts force to position commands
 *  - Manages state transitions based on contact status
 */

/**
 * @brief Grasp control states
 */
enum class GraspState {
  kIdle = 0,      // Default motion control
  kContact = 1,   // Contact detection phase
  kAdaptive = 2,  // Adaptive force control (slip-based)
  kForce = 3      // User-defined force tracking
};

/**
 * @brief Contact status from tactile sensors
 */
enum class ContactStatus {
  kNoContact = 0,
  kFewContact = 1,
  kEnoughContact = 2
};

/**
 * @brief Simplified tactile data structure (only what we need)
 */
struct TactileData {
  ContactStatus contact_state;
  rclcpp::Time timestamp;

  TactileData()
    : contact_state(ContactStatus::kNoContact),
      timestamp(0, 0, RCL_ROS_TIME) {}
};

/**
 * @brief Main node for force-controlled parallel grasp coordination
 */
class ParallelGraspForceControlNode : public rclcpp::Node {
public:
  ParallelGraspForceControlNode();

private:
  // === Initialization ===
  void initialize_subscribers();
  void initialize_publishers();
  void initialize_services();
  void initialize_service_clients();
  void load_parameters();

  // === Callbacks ===
  void grasp_command_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void tactile_0_callback(const sdr_grasp_msgs::msg::Tactile::SharedPtr msg);
  void tactile_1_callback(const sdr_grasp_msgs::msg::Tactile::SharedPtr msg);
  void force_reference_callback(const std_msgs::msg::Float64::SharedPtr msg);

  // Grasp force control callbacks (from grasp_force_ctrl nodes)
  void grasp_force_0_callback(const sdr_grasp_msgs::msg::GraspControl::SharedPtr msg);
  void grasp_force_1_callback(const sdr_grasp_msgs::msg::GraspControl::SharedPtr msg);

  // Service callbacks
  void start_adaptive_grasp_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void start_force_tracking_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stop_grasp_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // === Main Control Loop ===
  void control_loop_timer_callback();
  void update_state_machine();

  // === State Handlers ===
  void handle_idle_state();
  void handle_contact_state();
  void handle_adaptive_state();
  void handle_force_state();

  // === State Transitions ===
  void transition_to_idle();
  void transition_to_contact();
  void transition_to_adaptive();
  void transition_to_force();

  // === Contact Detection ===
  bool detect_initial_contact();
  bool confirm_stable_contact();
  bool detect_contact_loss();

  // === Force Control Service Calls ===
  void call_start_adaptive_grasp();
  void call_start_constant_grasp(double force);
  void call_stop_grasp();

  // === Command Generation ===
  double force_to_position_command(double force);

  // === Utilities ===
  void publish_state_info();
  std::string state_to_string(GraspState state);

  // === Member Variables ===

  // State
  GraspState current_state_;
  GraspState previous_state_;
  bool state_changed_;

  // Controller instance
  ParallelGraspController controller_;

  // Tactile data (simplified - only contact status needed)
  TactileData tactile_0_;
  TactileData tactile_1_;
  std::mutex tactile_mutex_;

  // Joint state
  sensor_msgs::msg::JointState current_joint_state_;
  std::mutex joint_state_mutex_;
  bool joint_state_received_;

  // Force data (from grasp_force_ctrl)
  double desired_force_0_;  // From grasp_force_ctrl_0
  double desired_force_1_;  // From grasp_force_ctrl_1
  std::mutex force_mutex_;

  // Control variables
  double grasp_width_command_;
  double force_reference_;      // User-specified force for kForce state
  double current_force_;        // Max of the two finger forces
  double base_contact_position_;

  // Timing and counters
  rclcpp::Time last_control_time_;
  double control_dt_;
  double stable_contact_timer_;
  int contact_lost_counter_;
  int stable_contact_counter_;

  // State transition flags
  bool start_adaptive_requested_;
  bool start_force_tracking_requested_;
  bool stop_requested_;

  // === Parameters ===
  struct Parameters {
    // State machine timing
    double contact_stable_time;
    double contact_lost_timeout;
    double control_rate;

    // Force limits
    double min_force_limit;
    double max_force_limit;

    // Mapping
    double force_to_position_gain;
    double force_to_effort_gain;

    // Impedance parameters for different states
    std::vector<double> default_stiffness;
    std::vector<double> default_damping;
    std::vector<double> adaptive_stiffness;
    std::vector<double> adaptive_damping;
    std::vector<double> force_tracking_stiffness;
    std::vector<double> force_tracking_damping;
  } params_;

  // === ROS Interfaces ===

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grasp_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tactile_0_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tactile_1_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_ref_sub_;

  // Subscribe to grasp force control output
  rclcpp::Subscription<sdr_grasp_msgs::msg::GraspControl>::SharedPtr grasp_force_0_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::GraspControl>::SharedPtr grasp_force_1_sub_;

  // Publishers
  rclcpp::Publisher<plato_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;

  // Services (offered by this node)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_adaptive_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_force_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_grasp_srv_;

  // Service clients (to call grasp_force_ctrl)
  rclcpp::Client<grasp_srvs::srv::StartAdaptiveGraspCtrl>::SharedPtr start_adaptive_client_;
  rclcpp::Client<grasp_srvs::srv::StartConstantGraspCtrl>::SharedPtr start_constant_client_;
  rclcpp::Client<grasp_srvs::srv::StopGraspCtrl>::SharedPtr stop_grasp_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  // === Constants ===
  static constexpr size_t NUM_JOINTS = 8;
  static constexpr double DEFAULT_CONTROL_RATE = 100.0;  // Hz
};
