#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <string>
#include <algorithm>

struct PID {
    double kp;
    double ki;
    double kd;
    double ki_cap; // Cap for integral term to prevent windup

    PID(double kp, double ki, double kd, double ki_cap)
        : kp(kp), ki(ki), kd(kd), ki_cap(ki_cap) {}
};

class PositionPIDController : public rclcpp::Node {
public:
    PositionPIDController(const std::vector<PID>& pid_params, double tolerance)
        : Node("position_pid_controller"), pid_params_(pid_params), tolerance_(tolerance) {
        // Initialize desired positions for each joint with default values
        desired_positions_ = std::vector<double>
            { 
                0.0, // Motor0 default position
                -0.658, // Motor1 default position
                0.968, // Motor2 default position
                0.0, // Motor3 default position
                0.0, // Motor4 default position
                -1.129, // Motor5 default position
                0.0, // Motor6 default position
                0.0, // Motor7 default position
                -1.129  // Motor8 default position 

            };

        // Setup subscriber and publisher
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/plato/joint_states", 10, std::bind(&PositionPIDController::jointStateCallback, this, std::placeholders::_1));

        effort_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/plato/plato_effort_controller/commands", 10);

        // Initialize error vectors
        error_integral_ = std::vector<double>(9, 0.0);
        previous_error_ = std::vector<double>(9, 0.0);
        joint_states_prev_ = std::vector<double>(9, 0.0);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std_msgs::msg::Float64MultiArray effort_command;
        effort_command.data.resize(9);

        // joint state 5 and 6 are flipped so we need to swap them
        std::swap(msg->position[5], msg->position[6]);

        

        for (size_t i = 0; i < 9; ++i) {
            

            double error = desired_positions_[i] - msg->position[i];
            double error_derivative = error - previous_error_[i];
            error_integral_[i] += error;

 
            

            // Implement ki_cap
            error_integral_[i] = std::clamp(error_integral_[i], -pid_params_[i].ki_cap, pid_params_[i].ki_cap);

            // PID formula for each motor
            double effort = pid_params_[i].kp * error + pid_params_[i].ki * error_integral_[i] + pid_params_[i].kd * error_derivative;

            // Check global tolerance for all joints
            if (std::abs(error) < tolerance_) {
                effort = 0.0; // Considered at goal position, stop applying effort
            }

            effort_command.data[i] = effort;
            previous_error_[i] = error;
            joint_states_prev_[i] = msg->position[i];
        }

        // Publish effort commands
        effort_command_publisher_->publish(effort_command);
    }

    // ROS elements
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_command_publisher_;

    // PID parameters for each joint
    std::vector<PID> pid_params_;
    double tolerance_; // Global goal tolerance for all motors
    std::vector<double> desired_positions_;
    std::vector<double> error_integral_;
    std::vector<double> previous_error_;
    std::vector<double> joint_states_prev_;
};

int main(int argc, char** argv) {
    double kp = 0.01;
    double ki_cap = 10.0;
    std::vector<PID> pid_params = {
        {kp, 0.0, 0.00, ki_cap}, // Motor 0 PID parameters
        {kp, 0.00, 0.00, ki_cap}, // Motor 1 PID parameters
        {kp, 1.00, 0.00, ki_cap}, // Motor 2 PID parameters
        {0, 0.00, 0.00, ki_cap}, // Motor 3 PID parameters
        {0, 0.00, 0.00, ki_cap}, // Motor 4 PID parameters
        {0, 0.00, 0.00, ki_cap}, // Motor 5 PID parameters
        {0.0, 0.00, 0.00, ki_cap}, // Motor 6 PID parameters
        {0.0, 0.00, 0.00, ki_cap}, // Motor 7 PID parameters
        {0.0, 0.00, 0.00, ki_cap}  // Motor 8 PID parameters

        
    };
    double tolerance = 0.01; // Global tolerance for all motors

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionPIDController>(pid_params, tolerance);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
