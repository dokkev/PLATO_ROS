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
    PositionPIDController()
        : Node("position_pid_controller") {

        // Initialize desired positions for each joint with default values
        desired_positions_ = std::vector<double>
            { 
                0.0, // Motor0 default position
                -0.658, // Motor1 default position
                0.69, // Motor2 default position
                0.0, // Motor3 default position
                0.0, // Motor4 default position
                -1.129, // Motor5 default position
                0.0, // Motor6 default position
                0.0, // Motor7 default position
                -1.129  // Motor8 default position 

            };

        desired_positions2_ = std::vector<double>
            { 
                -0.042, // Motor0
                -0.617 , // Motor1
                1.2, // Motor2
                -0.137, // Motor3
                -0.400, // Motor4
                -1.219, // Motor5
                0.0, // Motor6
                0.0, // Motor7
                0.0  // Motor8
            };

        // Setup subscriber and publisher
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/plato/joint_states", 10, std::bind(&PositionPIDController::jointStateCallback, this, std::placeholders::_1));

        position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/plato/plato_position_controller/commands", 10);

        // Initialize error vectors
        error_integral_ = std::vector<double>(9, 0.0);
        previous_error_ = std::vector<double>(9, 0.0);
        joint_states_prev_ = std::vector<double>(9, 0.0);
        joint_velocity_prev_ = std::vector<double>(9, 0.0);
        joint_acceleration_prev_ = std::vector<double>(9, 0.0);
        effort_prev_ = std::vector<double>(9, 0.0);
        int counter = 0;
    }

private:
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std_msgs::msg::Float64MultiArray position_command;
        position_command.data.resize(9);

        position_command.data[0] = desired_positions_[0];
        position_command.data[1] = desired_positions_[1];
        position_command.data[2] = desired_positions_[2];
        position_command.data[3] = desired_positions_[3];
        position_command.data[4] = desired_positions_[4];
        position_command.data[5] = desired_positions_[5];
        position_command.data[6] = desired_positions_[6];
        position_command.data[7] = desired_positions_[7];
        position_command.data[8] = desired_positions_[8];


        // Publish effort commands
        position_command_publisher_->publish(position_command);



    }


    // ROS elements
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;

    // PID parameters for each joint
    std::vector<PID> pid_params_;
    double tolerance_; // Global goal tolerance for all motors
    double output_ramp = 0.05;
    std::vector<double> desired_positions_;
    std::vector<double> desired_positions2_;
    std::vector<double> desired_positions3_;
    std::vector<double> error_integral_;
    std::vector<double> previous_error_;
    std::vector<double> joint_states_prev_;
    std::vector<double> joint_velocity_prev_;
    std::vector<double> joint_acceleration_prev_;
    std::vector<double> effort_prev_;
    int counter = 0;
    rclcpp::Clock ros_clock;

};

int main(int argc, char** argv) {




    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionPIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
