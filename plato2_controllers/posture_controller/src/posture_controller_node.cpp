#include "posture_controller/posture_controller_node.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

PostureControllerNode::PostureControllerNode()
  : rclcpp::Node("posture_controller_node") {
  
  // Create publisher for impedance commands
  commands_publisher_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
    "/plato2/joint_impedance_controller/commands",
    10
  );

  // Initialize postures with zeros (to be filled manually)
  initialize_postures();

  RCLCPP_INFO(this->get_logger(), "Posture Controller Node initialized.");
  RCLCPP_INFO(this->get_logger(), "Controls:");
  RCLCPP_INFO(this->get_logger(), "  '1' - Poking posture");
  RCLCPP_INFO(this->get_logger(), "  '2' - Fingernail Grasp (defaults to open)");
  RCLCPP_INFO(this->get_logger(), "  '3' - Fingerpad Grasp (defaults to open)");
  RCLCPP_INFO(this->get_logger(), "  SPACE - Toggle open/close for current grasp");
  RCLCPP_INFO(this->get_logger(), "  'q' - Quit");
  RCLCPP_INFO(this->get_logger(), "Transition time: %.2fs | Closing time: %.2fs", transition_time_, closing_time_);
  RCLCPP_INFO(this->get_logger(), "Thumb - K:%.1f B:%.2f | Fingers - K:%.1f B:%.2f", 
    thumb_stiffness_, thumb_damping_, finger_stiffness_, finger_damping_);
  RCLCPP_INFO(this->get_logger(), "Effort FF - Fingernail:%.2f | Fingerpad:%.2f", 
    fingernail_grasp_effort_ff_, fingerpad_grasp_effort_ff_);

  // Start keyboard input thread
  keyboard_thread_ = std::thread(&PostureControllerNode::keyboard_input_loop, this);
}

PostureControllerNode::~PostureControllerNode() {
  running_ = false;
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
}

void PostureControllerNode::initialize_postures() {
  // Initialize all postures with zeros (NUM_JOINTS = 8)
  poking_posture_ = {0.00, 0.00, 0.10, -1.96, 0.00, 0.00, 0.97, 1.43};
  fingernail_grasp_open_ = {0.00, 0.00, 0.39, -1.06, -0.07, 1.11, 0.97, 1.43};
  fingernail_grasp_closed_ = {0.00, 0.00, 0.26, -0.94, 0.08, 0.96, 0.97, 1.43};
  fingerpad_grasp_open_ = {0.00, 0.00, -0.25, 0.38, 0.78, -0.65, 0.97, 1.43};
  fingerpad_grasp_closed_ = {0.00, 0.00, -0.51, 0.72, 0.89, -0.63, 0.97, 1.43};

  // Initialize current posture to home position (zeros)
  current_posture_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void PostureControllerNode::keyboard_input_loop() {
  while (running_) {
    char input;
    std::cin.get(input);

    if (input == 'q' || input == 'Q') {
      RCLCPP_INFO(this->get_logger(), "Shutting down...");
      running_ = false;
      break;
    } else if (input == '1') {
      RCLCPP_INFO(this->get_logger(), "Transitioning to Poking posture...");
      current_grasp_mode_ = NONE;
      is_grasp_closed_ = false;
      transition_to_posture(poking_posture_);
    } else if (input == '2') {
      RCLCPP_INFO(this->get_logger(), "Transitioning to Fingernail Grasp (OPEN)...");
      current_grasp_mode_ = FINGERNAIL;
      is_grasp_closed_ = false;
      transition_to_posture(fingernail_grasp_open_);
    } else if (input == '3') {
      RCLCPP_INFO(this->get_logger(), "Transitioning to Fingerpad Grasp (OPEN)...");
      current_grasp_mode_ = FINGERPAD;
      is_grasp_closed_ = false;
      transition_to_posture(fingerpad_grasp_open_);
    } else if (input == ' ') {
      // Space bar toggles open/close for current grasp
      if (current_grasp_mode_ == FINGERNAIL) {
        if (is_grasp_closed_) {
          RCLCPP_INFO(this->get_logger(), "Opening Fingernail Grasp...");
          is_grasp_closed_ = false;
          transition_to_posture(fingernail_grasp_open_);
        } else {
          RCLCPP_INFO(this->get_logger(), "Closing Fingernail Grasp...");
          is_grasp_closed_ = true;
          transition_to_posture(fingernail_grasp_closed_);
        }
      } else if (current_grasp_mode_ == FINGERPAD) {
        if (is_grasp_closed_) {
          RCLCPP_INFO(this->get_logger(), "Opening Fingerpad Grasp...");
          is_grasp_closed_ = false;
          transition_to_posture(fingerpad_grasp_open_);
        } else {
          RCLCPP_INFO(this->get_logger(), "Closing Fingerpad Grasp...");
          is_grasp_closed_ = true;
          transition_to_posture(fingerpad_grasp_closed_);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Space bar only works with Fingernail (2) or Fingerpad (3) grasps.");
      }
    } else if (input != '\n') {
      RCLCPP_WARN(this->get_logger(), "Invalid input. Press '1', '2', '3', SPACE, or 'q'.");
    }
  }
}

void PostureControllerNode::transition_to_posture(const std::vector<double>& target_posture) {
  // Determine which timing to use based on whether we're closing a grasp
  double duration = transition_time_;
  if (is_grasp_closed_ && (current_grasp_mode_ == FINGERNAIL || current_grasp_mode_ == FINGERPAD)) {
    duration = closing_time_;
  }

  // Generate minimum jerk trajectory
  auto trajectory = minimum_jerk_trajectory(
    current_posture_,
    target_posture,
    duration,
    DT
  );

  // Execute trajectory
  for (const auto& pos : trajectory) {
    if (!running_) break;
    publish_impedance_command(pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(DT * 1000)));
  }

  // Update current posture
  current_posture_ = target_posture;
  RCLCPP_INFO(this->get_logger(), "Posture transition completed.");
}

std::vector<std::vector<double>> PostureControllerNode::minimum_jerk_trajectory(
  const std::vector<double>& init_pos,
  const std::vector<double>& target_pos,
  double total_time,
  double dt) {
  
  std::vector<std::vector<double>> trajectory;

  if (total_time == 0.0) {
    trajectory.push_back(target_pos);
    return trajectory;
  }

  int num_steps = static_cast<int>(total_time / dt);

  for (int step = 1; step <= num_steps; ++step) {
    double t = step * dt;
    double normalized_t = t / total_time;

    // Minimum jerk polynomial: s(t) = 10*t^3 - 15*t^4 + 6*t^5
    double s = 10.0 * std::pow(normalized_t, 3) -
               15.0 * std::pow(normalized_t, 4) +
               6.0 * std::pow(normalized_t, 5);

    // Interpolate position
    std::vector<double> interpolated(init_pos.size());
    for (size_t i = 0; i < init_pos.size(); ++i) {
      interpolated[i] = init_pos[i] + (target_pos[i] - init_pos[i]) * s;
    }

    trajectory.push_back(interpolated);
  }

  return trajectory;
}

void PostureControllerNode::publish_impedance_command(const std::vector<double>& positions) {
  auto cmd = plato2_interfaces::msg::ImpedanceCommands();

  cmd.position = positions;
  cmd.velocity = std::vector<double>(NUM_JOINTS, 0.0);

  // Set stiffness values
  cmd.stiffness = std::vector<double>(NUM_JOINTS, 0.0);
  // Thumb stiffness (joints 0, 1)
  if (NUM_JOINTS >= 2) {
    cmd.stiffness[0] = thumb_stiffness_;
    cmd.stiffness[1] = thumb_stiffness_;
  }
  // Finger stiffness (MCP -> [2,4,6], PIP -> [3,5,7])
  for (int idx : {2, 4, 6}) {
    if (idx < NUM_JOINTS) {
      cmd.stiffness[idx] = finger_stiffness_;
    }
  }
  for (int idx : {3, 5, 7}) {
    if (idx < NUM_JOINTS) {
      cmd.stiffness[idx] = finger_stiffness_;
    }
  }

  // Set damping values
  cmd.damping = std::vector<double>(NUM_JOINTS, 0.0);
  // Thumb damping (joints 0, 1)
  if (NUM_JOINTS >= 2) {
    cmd.damping[0] = thumb_damping_;
    cmd.damping[1] = thumb_damping_;
  }
  // Finger damping (MCP -> [2,4,6], PIP -> [3,5,7])
  for (int idx : {2, 4, 6}) {
    if (idx < NUM_JOINTS) {
      cmd.damping[idx] = finger_damping_;
    }
  }
  for (int idx : {3, 5, 7}) {
    if (idx < NUM_JOINTS) {
      cmd.damping[idx] = finger_damping_;
    }
  }

  // Set effort feedforward values
  cmd.effort_ff = std::vector<double>(NUM_JOINTS, 0.0);
  
  // Apply effort feedforward for closed grasps on finger joints
  if (is_grasp_closed_) {
    double effort_ff = 0.0;
    if (current_grasp_mode_ == FINGERNAIL) {
      effort_ff = fingernail_grasp_effort_ff_;
    } else if (current_grasp_mode_ == FINGERPAD) {
      effort_ff = fingerpad_grasp_effort_ff_;
    }
    
    // Apply to finger joints (MCP -> [2,4,6], PIP -> [3,5,7])
    for (int idx : {2, 4, 6}) {
      if (idx < NUM_JOINTS) {
        cmd.effort_ff[idx] = effort_ff;
      }
    }
    for (int idx : {3, 5, 7}) {
      if (idx < NUM_JOINTS) {
        cmd.effort_ff[idx] = effort_ff;
      }
    }
  }

  commands_publisher_->publish(cmd);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PostureControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
