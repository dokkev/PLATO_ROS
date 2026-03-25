#ifndef POSTURE_CONTROLLER_NODE_HPP
#define POSTURE_CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "plato_interfaces/msg/impedance_commands.hpp"
#include <vector>
#include <thread>
#include <atomic>

class PostureControllerNode : public rclcpp::Node {
public:
  PostureControllerNode();
  ~PostureControllerNode();

private:
  // Publisher for impedance commands
  rclcpp::Publisher<plato_interfaces::msg::ImpedanceCommands>::SharedPtr commands_publisher_;

  // Joint posture definitions (8x1 arrays)
  std::vector<double> poking_posture_;
  std::vector<double> fingernail_grasp_open_;
  std::vector<double> fingernail_grasp_closed_;
  std::vector<double> fingerpad_grasp_open_;
  std::vector<double> fingerpad_grasp_closed_;

  // Current posture state
  std::vector<double> current_posture_;

  // Control parameters
  static constexpr int NUM_JOINTS = 8;
  static constexpr double DT = 0.01;  // time step
  
  // Trajectory timing parameters (in seconds)
  double transition_time_ = 1.0;    // Time to transition between postures
  double closing_time_ = 1.0;       // Time to close the hand during grasp
  
  // Impedance parameters for thumb (joints 0, 1)
  double thumb_stiffness_ = 0.0;
  double thumb_damping_ = 0.0;
  
  // Impedance parameters for fingers (MCPs at joints 2,4,6 and PIPs at joints 3,5,7)
  double finger_stiffness_ = 3.0;
  double finger_damping_ = 0.2;
  
  // Effort feedforward for closed grasps
  double fingernail_grasp_effort_ff_ = 0.0;  // Effort feedforward for fingernail grasp closed
  double fingerpad_grasp_effort_ff_ = 0.0;   // Effort feedforward for fingerpad grasp closed
  
  // Grasp state tracking
  enum GraspMode { NONE, FINGERNAIL, FINGERPAD };
  GraspMode current_grasp_mode_ = NONE;
  bool is_grasp_closed_ = false;  // true if current grasp is closed, false if open

  // Thread for keyboard input
  std::thread keyboard_thread_;
  std::atomic<bool> running_{true};

  // Methods
  void initialize_postures();
  void keyboard_input_loop();
  void transition_to_posture(const std::vector<double>& target_posture);
  std::vector<std::vector<double>> minimum_jerk_trajectory(
    const std::vector<double>& init_pos,
    const std::vector<double>& target_pos,
    double total_time,
    double dt
  );
  void publish_impedance_command(const std::vector<double>& positions);
};

#endif  // POSTURE_CONTROLLER_NODE_HPP
