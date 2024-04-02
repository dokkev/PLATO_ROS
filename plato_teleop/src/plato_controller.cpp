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

        desired_positions2_ = std::vector<double>
            { 
                -0.042, // Motor0
                -0.617 , // Motor1
                1.219, // Motor2
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

        effort_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/plato/plato_effort_controller/commands", 10);

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
        std_msgs::msg::Float64MultiArray effort_command;
        effort_command.data.resize(9);


        // Calculate time difference
        rclcpp::Time current_time_stamp = ros_clock.now();

        // joint state 5 and 6 are flipped so we need to swap them
        std::swap(msg->position[5], msg->position[6]);

   
        if (counter < 1000){

            desired_positions2_[1] = desired_positions2_[1] - 0.0000;
            desired_positions2_[2] = desired_positions2_[2] - 0.001;

            desired_positions2_[4] = desired_positions2_[4] + 0.0000;
            desired_positions2_[5] = desired_positions2_[5] + 0.001;
  

        }
        else if (counter >= 1000 && counter < 2000){
        desired_positions2_[1] = desired_positions2_[1] + 0.0000;
        desired_positions2_[2] = desired_positions2_[2] + 0.001;

        desired_positions2_[4] = desired_positions2_[4] - 0.0000;
        desired_positions2_[5] = desired_positions2_[5] - 0.001;

        

        }
        else if (counter == 2000){
            counter = 0;
        
        }
      


        

        

        for (size_t i = 0; i < 9; ++i) {

            // apply low pass filter to joint states
            // msg->position[i] = 0.1 * msg->position[i] + 0.9 * joint_states_prev_[i];
            double alpha = 0.2;
            msg->position[i] = alpha * msg->position[i] + (1 - alpha) * joint_states_prev_[i];
            
            double error = desired_positions2_[i] - msg->position[i];
            double error_derivative = error - previous_error_[i];
            error_integral_[i] += error;

            // velocity estimation
            double dt = (current_time_stamp - previous_time_stamp).seconds();
            double joint_velocity = (msg->position[i] - joint_states_prev_[i]) / dt;
            double joint_acceleration = (joint_velocity - joint_acceleration_prev_[i]) / dt;
            double joint_jerk = (joint_acceleration - joint_acceleration_prev_[i]) / dt;
            joint_acceleration_prev_[i] = joint_velocity; 
            

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
            effort_prev_[i] = effort;

            

        }

        counter = counter + 1;

        // Publish effort commands
        effort_command_publisher_->publish(effort_command);

        previous_time_stamp = current_time_stamp;
    }


    // ROS elements
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_command_publisher_;

    // PID parameters for each joint
    std::vector<PID> pid_params_;
    double tolerance_; // Global goal tolerance for all motors
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
    rclcpp::Time previous_time_stamp;
};

int main(int argc, char** argv) {

    double ki_cap = 2.0;

    double kp0 = 0.25;  double ki0 = 0.00;  double kd0 = 1.0;
    double kp1 = 0.03;  double ki1 = 0.0;   double kd1 = 0.9;
    double kp2 = 0.05;  double ki2 = 0.0;   double kd2 = 0.8;
    
    //////////////////////////////////////////////////////////

    double kp3 = 0.3;   double ki3 = 0.0;   double kd3 = 1.0;
    double kp4 = 0.03;  double ki4 = 0.0;   double kd4 = 0.9;
    double kp5 = 0.05;  double ki5 = 0.00;  double kd5 = 0.08;

    //////////////////////////////////////////////////////////

    double kp6 = 0.0;   double ki6 = 0.0;   double kd6 = 0.0;
    double kp7 = 0.0;   double ki7 = 0.0;   double kd7 = 0.0;
    double kp8 = 0.0;   double ki8 = 0.0;   double kd8 = 0.0;






    std::vector<PID> pid_params = {
        {kp0, ki0, kd0, ki_cap}, // Motor 0 PID parameters
        {kp1, ki1, kd1, ki_cap}, // Motor 1 PID parameters
        {kp2, ki2, kd2, ki_cap}, // Motor 2 PID parameters
        {kp3, ki3, kd3, ki_cap}, // Motor 3 PID parameters
        {kp4, ki4, kd4, ki_cap}, // Motor 4 PID parameters
        {kp5, ki5, kd5, ki_cap}, // Motor 5 PID parameters
        {kp6, ki6, kd6, ki_cap}, // Motor 6 PID parameters
        {kp7, ki7, kd7, ki_cap}, // Motor 7 PID parameters
        {kp8, ki8, kd8, ki_cap}  // Motor 8 PID parameters

        
    };
    double tolerance = 0.001; // Global tolerance for all motors

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionPIDController>(pid_params, tolerance);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
