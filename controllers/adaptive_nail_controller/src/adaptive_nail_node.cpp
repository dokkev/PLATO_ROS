#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <plato_interfaces/msg/impedance_commands.hpp>
#include <atomic>
#include <cmath>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <map>
#include <iostream>

// Joint Indices for Command Array (0-based)
const int IDX_MCP_1 = 2; // Thumb MCP (Joint 3)
const int IDX_PIP_1 = 3; // Thumb PIP (Joint 4)
const int IDX_MCP_2 = 4; // Index MCP (Joint 5)
const int IDX_PIP_2 = 5; // Index PIP (Joint 6)

enum class GraspState {
    IDLE,
    STEP_1_OPEN,
    STEP_2_APPROACH_CONTACT,
    STEP_3_PREP_ADAPT,
    STEP_4_ADAPTIVE_WRAP,
    STEP_5_LOCK_GRIP,
    SAFETY_STOP
};

class AdaptiveGraspNode : public rclcpp::Node {
public:
    AdaptiveGraspNode()
    : Node("adaptive_grasp_node"), state_(GraspState::IDLE)
    {
        // --- Parameters ---
        // Gains
        this->declare_parameter("stiffness_high", 3.0);
        this->declare_parameter("damping_high", 0.2);
        this->declare_parameter("stiffness_low", 0.5);
        this->declare_parameter("damping_low", 0.1);
        
        // Motion
        this->declare_parameter("mcp_closing_rate", 0.2); 
        this->declare_parameter("pip_force_gain", 0.001);
        
        // Open Positions
        this->declare_parameter("open_mcp_1", -0.36);   // Thumb MCP
        this->declare_parameter("open_pip_1", 0.37);    // Thumb PIP
        this->declare_parameter("open_mcp_2", -0.36);   // Index MCP
        this->declare_parameter("open_pip_2", 0.37);    // Index PIP 
        
        // NOISE HANDLING
        this->declare_parameter("force_deadband", 0.2);      // +/- Newtons to ignore
        this->declare_parameter("force_smoothing_factor", 0.2); // 0.0=No change, 1.0=All new data. Lower = Smoother.
        
        // Operational Thresholds
        this->declare_parameter("contact_force_threshold", 0.5); 
        this->declare_parameter("lock_torque_threshold", 0.03);  
        
        // SAFETY LIMITS 
        this->declare_parameter("safety_force_limit", 5.0);  
        this->declare_parameter("safety_torque_limit", 0.5); 
        
        this->declare_parameter("feedforward_effort", 0.0);      
        this->declare_parameter("force_component", "norm"); 
        this->declare_parameter("ft_topic", "/plato2/ft_sensor2/wrench");
        
        // Joint names
        this->declare_parameter("joint_mcp_1", "joint3");
        this->declare_parameter("joint_pip_1", "joint4");
        this->declare_parameter("joint_mcp_2", "joint5");
        this->declare_parameter("joint_pip_2", "joint6");
        
        // Joint limits
        this->declare_parameter("mcp_min_limit", -0.5);
        this->declare_parameter("mcp_max_limit", 1.57);
        this->declare_parameter("pip_min_limit", -0.5);
        this->declare_parameter("pip_max_limit", 1.57);

        // Load Params
        stiffness_high_ = this->get_parameter("stiffness_high").as_double();
        damping_high_ = this->get_parameter("damping_high").as_double();
        stiffness_low_ = this->get_parameter("stiffness_low").as_double();
        damping_low_ = this->get_parameter("damping_low").as_double();
        mcp_closing_rate_ = this->get_parameter("mcp_closing_rate").as_double();
        pip_force_gain_ = this->get_parameter("pip_force_gain").as_double();
        
        open_mcp_1_ = this->get_parameter("open_mcp_1").as_double();
        open_pip_1_ = this->get_parameter("open_pip_1").as_double();
        open_mcp_2_ = this->get_parameter("open_mcp_2").as_double();
        open_pip_2_ = this->get_parameter("open_pip_2").as_double();
        
        force_deadband_ = this->get_parameter("force_deadband").as_double();
        force_alpha_ = this->get_parameter("force_smoothing_factor").as_double();
        
        contact_force_thresh_ = this->get_parameter("contact_force_threshold").as_double();
        lock_torque_thresh_ = this->get_parameter("lock_torque_threshold").as_double();
        safety_force_limit_ = this->get_parameter("safety_force_limit").as_double();
        safety_torque_limit_ = this->get_parameter("safety_torque_limit").as_double();
        ff_effort_val_ = this->get_parameter("feedforward_effort").as_double();
        force_comp_str_ = this->get_parameter("force_component").as_string();
        std::string ft_topic = this->get_parameter("ft_topic").as_string();
        
        joint_mcp_1_name_ = this->get_parameter("joint_mcp_1").as_string();
        joint_pip_1_name_ = this->get_parameter("joint_pip_1").as_string();
        joint_mcp_2_name_ = this->get_parameter("joint_mcp_2").as_string();
        joint_pip_2_name_ = this->get_parameter("joint_pip_2").as_string();
        
        mcp_min_limit_ = this->get_parameter("mcp_min_limit").as_double();
        mcp_max_limit_ = this->get_parameter("mcp_max_limit").as_double();
        pip_min_limit_ = this->get_parameter("pip_min_limit").as_double();
        pip_max_limit_ = this->get_parameter("pip_max_limit").as_double();

        // Initialize Command State
        cmd_pos_.resize(8, 0.0);
        cmd_stiff_.resize(8, stiffness_high_);
        cmd_damp_.resize(8, damping_high_);
        
        // Set inactive joints (1, 2, 7, 8) to zero gains
        cmd_stiff_[0] = 0.0; cmd_damp_[0] = 0.0;  // Joint 1
        cmd_stiff_[1] = 0.0; cmd_damp_[1] = 0.0;  // Joint 2
        cmd_stiff_[6] = 0.0; cmd_damp_[6] = 0.0;  // Joint 7
        cmd_stiff_[7] = 0.0; cmd_damp_[7] = 0.0;  // Joint 8
        
        // --- Subscribers & Publishers ---
        ft_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            ft_topic, rclcpp::SensorDataQoS(),
            std::bind(&AdaptiveGraspNode::ft_callback, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/plato2/joint_states_smoothed", rclcpp::SensorDataQoS(),
            std::bind(&AdaptiveGraspNode::joint_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<plato_interfaces::msg::ImpedanceCommands>(
            "/plato2/joint_impedance_controller/commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&AdaptiveGraspNode::control_loop, this));

        running_ = true;
        keyboard_thread_ = std::thread(&AdaptiveGraspNode::keyboard_loop, this);

        RCLCPP_INFO(this->get_logger(), "Adaptive Node Ready. Deadband: %.2f N, SmoothAlpha: %.2f", force_deadband_, force_alpha_);
    }

    ~AdaptiveGraspNode() {
        running_ = false;
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        restore_terminal();
    }

private:
    void ft_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        current_wrench_ = *msg;
        
        // 1. Calculate Raw Metric
        double raw_force = 0.0;
        if (force_comp_str_ == "fx") raw_force = std::abs(msg->wrench.force.x);
        else if (force_comp_str_ == "fy") raw_force = std::abs(msg->wrench.force.y);
        else if (force_comp_str_ == "fz") raw_force = std::abs(msg->wrench.force.z);
        else raw_force = std::sqrt(pow(msg->wrench.force.x, 2) + pow(msg->wrench.force.y, 2) + pow(msg->wrench.force.z, 2));

        // 2. Apply Low Pass Filter (Exponential Moving Average)
        // filtered = alpha * raw + (1 - alpha) * filtered_prev
        if (first_sample_) {
            current_force_val_ = raw_force;
            first_sample_ = false;
        } else {
            current_force_val_ = (force_alpha_ * raw_force) + ((1.0 - force_alpha_) * current_force_val_);
        }

        current_torque_val_ = std::sqrt(pow(msg->wrench.torque.x, 2) + pow(msg->wrench.torque.y, 2) + pow(msg->wrench.torque.z, 2));
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_map_[msg->name[i]] = i;
        }
        current_joint_state_ = *msg;
        joint_state_received_ = true;
    }

    double get_joint_pos(const std::string& name) {
        if (!joint_state_received_ || joint_map_.find(name) == joint_map_.end()) return 0.0;
        return current_joint_state_.position[joint_map_[name]];
    }
    
    double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
    
    void setup_terminal() {
        tcgetattr(STDIN_FILENO, &original_term_);
        termios newt = original_term_;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        terminal_configured_ = true;
    }
    
    void restore_terminal() {
        if (terminal_configured_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_term_);
            terminal_configured_ = false;
        }
    }

    void control_loop() {
        if (!joint_state_received_) return;
        
        std::lock_guard<std::mutex> lock(state_mutex_);

        // --- SAFETY WATCHDOG ---
        if (state_ != GraspState::SAFETY_STOP) {
            if (current_force_val_ > safety_force_limit_ || current_torque_val_ > safety_torque_limit_) {
                RCLCPP_ERROR(this->get_logger(), 
                    "!!! SAFETY LIMIT EXCEEDED !!! Force: %.2f N, Torque: %.2f Nm. DROPPING GAINS.", 
                    current_force_val_, current_torque_val_);
                state_ = GraspState::SAFETY_STOP;
            }
        }

        switch (state_) {
            case GraspState::SAFETY_STOP:
            {
                set_gains(0.0, 0.0, 0.0, 0.0);
                cmd_pos_[IDX_MCP_1] = get_joint_pos(joint_mcp_1_name_);
                cmd_pos_[IDX_MCP_2] = get_joint_pos(joint_mcp_2_name_);
                cmd_pos_[IDX_PIP_1] = get_joint_pos(joint_pip_1_name_);
                cmd_pos_[IDX_PIP_2] = get_joint_pos(joint_pip_2_name_);
                break;
            }

            case GraspState::IDLE:
                break;

            case GraspState::STEP_1_OPEN:
            {
                set_gains(stiffness_high_, damping_high_, stiffness_high_, damping_high_);
                // Thumb positions
                cmd_pos_[IDX_MCP_1] = open_mcp_1_;
                cmd_pos_[IDX_PIP_1] = open_pip_1_;
                // Index positions
                cmd_pos_[IDX_MCP_2] = open_mcp_2_;
                cmd_pos_[IDX_PIP_2] = open_pip_2_;
                break;
            }

            case GraspState::STEP_2_APPROACH_CONTACT:
            {
                set_gains(stiffness_high_, damping_high_, 0.0, 0.0);
                
                if (current_force_val_ < contact_force_thresh_) {
                    // Continue closing MCP joints with limits
                    cmd_pos_[IDX_MCP_1] = clamp(cmd_pos_[IDX_MCP_1] + (mcp_closing_rate_ * 0.01), 
                                                mcp_min_limit_, mcp_max_limit_);
                    cmd_pos_[IDX_MCP_2] = clamp(cmd_pos_[IDX_MCP_2] + (mcp_closing_rate_ * 0.01), 
                                                mcp_min_limit_, mcp_max_limit_);
                } else {
                    // Contact detected, stop movement
                    if (!contact_detected_) {
                        RCLCPP_INFO(this->get_logger(), "Contact Detected (%.2f N). MCP stopped.", current_force_val_);
                        contact_detected_ = true;
                    }
                }
                break;
            }

            case GraspState::STEP_3_PREP_ADAPT:
            {
                target_force_ = current_force_val_;
                mcp_closing_rate_ = this->get_parameter("mcp_closing_rate").as_double();
                set_gains(stiffness_high_, damping_high_, stiffness_low_, damping_low_);
                cmd_pos_[IDX_PIP_1] = get_joint_pos(joint_pip_1_name_);
                cmd_pos_[IDX_PIP_2] = get_joint_pos(joint_pip_2_name_);
                
                if(prep_print_count_++ % 100 == 0) {
                     RCLCPP_INFO(this->get_logger(), "Target Force Set: %.3f N. Deadband: +/- %.2f N", target_force_, force_deadband_);
                }
                break;
            }

            case GraspState::STEP_4_ADAPTIVE_WRAP:
            {
                // A. Close MCP with limits
                double dt = 0.01;
                cmd_pos_[IDX_MCP_1] = clamp(cmd_pos_[IDX_MCP_1] + (mcp_closing_rate_ * dt), 
                                            mcp_min_limit_, mcp_max_limit_);
                cmd_pos_[IDX_MCP_2] = clamp(cmd_pos_[IDX_MCP_2] + (mcp_closing_rate_ * dt), 
                                            mcp_min_limit_, mcp_max_limit_);

                // B. PIP Force Maintenance with DEADBAND
                double force_error = current_force_val_ - target_force_;

                // --- DEADBAND LOGIC ---
                // If error is within +/- deadband, treat it as zero to prevent jitter
                if (std::abs(force_error) < force_deadband_) {
                    force_error = 0.0;
                }
                
                double correction = pip_force_gain_ * force_error;
                // Apply corrections with limits to prevent unbounded drift
                cmd_pos_[IDX_PIP_1] = clamp(cmd_pos_[IDX_PIP_1] + correction, 
                                            pip_min_limit_, pip_max_limit_);
                cmd_pos_[IDX_PIP_2] = clamp(cmd_pos_[IDX_PIP_2] + correction, 
                                            pip_min_limit_, pip_max_limit_);

                // C. Check Operational Torque Threshold
                if (current_torque_val_ > lock_torque_thresh_) {
                     RCLCPP_WARN(this->get_logger(), "Lock Torque Met (%.3f Nm). Locking.", current_torque_val_);
                     state_ = GraspState::STEP_5_LOCK_GRIP;
                }
                break;
            }

            case GraspState::STEP_5_LOCK_GRIP:
            {
                set_gains(stiffness_high_, damping_high_, stiffness_high_, damping_high_);
                std::vector<double> eff(8, 0.0);
                eff[IDX_MCP_1] = ff_effort_val_;
                eff[IDX_MCP_2] = ff_effort_val_;
                eff[IDX_PIP_1] = ff_effort_val_;
                eff[IDX_PIP_2] = ff_effort_val_;
                publish_cmd(cmd_pos_, eff); 
                return; 
            }
        }

        std::vector<double> zero_eff(8, 0.0);
        publish_cmd(cmd_pos_, zero_eff);
    }

    void set_gains(double mcp_k, double mcp_d, double pip_k, double pip_d) {
        cmd_stiff_[IDX_MCP_1] = mcp_k; cmd_damp_[IDX_MCP_1] = mcp_d;
        cmd_stiff_[IDX_MCP_2] = mcp_k; cmd_damp_[IDX_MCP_2] = mcp_d;
        cmd_stiff_[IDX_PIP_1] = pip_k; cmd_damp_[IDX_PIP_1] = pip_d;
        cmd_stiff_[IDX_PIP_2] = pip_k; cmd_damp_[IDX_PIP_2] = pip_d;
    }

    void publish_cmd(const std::vector<double>& pos, const std::vector<double>& eff_ff) {
        plato_interfaces::msg::ImpedanceCommands msg;
        msg.position = pos;
        msg.velocity.resize(8, 0.0);
        msg.stiffness = cmd_stiff_;
        msg.damping = cmd_damp_;
        msg.effort_ff = eff_ff;
        cmd_pub_->publish(msg);
    }

    void keyboard_loop() {
        setup_terminal();

        std::cout << "\nAdaptive Grasper Controller\n";
        std::cout << "'n' -> Next Step\n";
        std::cout << "'r' -> Reset Safety\n";
        std::cout << "'q' -> Quit\n";

        while (running_ && rclcpp::ok()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) == 1) {
                if (c == 'q') {
                    rclcpp::shutdown();
                } else if (c == 'n') {
                    advance_state();
                } else if (c == 'r') {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (state_ == GraspState::SAFETY_STOP) {
                        state_ = GraspState::IDLE;
                        RCLCPP_INFO(this->get_logger(), "Safety Reset. System IDLE.");
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        restore_terminal();
    }

    void advance_state() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (state_ == GraspState::SAFETY_STOP) {
            RCLCPP_WARN(this->get_logger(), "Cannot advance! Safety Stop Active. Press 'r' to reset.");
            return;
        }
        
        switch (state_) {
            case GraspState::IDLE: 
                state_ = GraspState::STEP_1_OPEN;
                contact_detected_ = false;
                prep_print_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "-> STEP 1: OPEN HAND");
                break;
            case GraspState::STEP_1_OPEN: 
                state_ = GraspState::STEP_2_APPROACH_CONTACT;
                contact_detected_ = false;
                RCLCPP_INFO(this->get_logger(), "-> STEP 2: APPROACH / CONTACT");
                break;
            case GraspState::STEP_2_APPROACH_CONTACT: 
                state_ = GraspState::STEP_3_PREP_ADAPT;
                prep_print_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "-> STEP 3: PREP ADAPT (Read Force)");
                break;
            case GraspState::STEP_3_PREP_ADAPT: 
                state_ = GraspState::STEP_4_ADAPTIVE_WRAP; 
                RCLCPP_INFO(this->get_logger(), "-> STEP 4: ADAPTIVE WRAP (Force Hold)");
                break;
            case GraspState::STEP_4_ADAPTIVE_WRAP: 
                state_ = GraspState::STEP_5_LOCK_GRIP; 
                RCLCPP_INFO(this->get_logger(), "-> STEP 5: LOCK GRIP (Manual Trigger)");
                break;
            case GraspState::STEP_5_LOCK_GRIP: 
                state_ = GraspState::STEP_1_OPEN;
                contact_detected_ = false;
                prep_print_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "-> Resetting to STEP 1");
                break;
            case GraspState::SAFETY_STOP:
                RCLCPP_WARN(this->get_logger(), "In SAFETY_STOP state. Press 'r' to reset.");
                break;
        }
    }

    // Members
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<plato_interfaces::msg::ImpedanceCommands>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
    
    // Thread safety
    std::mutex state_mutex_;
    
    // Data
    GraspState state_;
    geometry_msgs::msg::WrenchStamped current_wrench_;
    sensor_msgs::msg::JointState current_joint_state_;
    std::map<std::string, int> joint_map_;
    bool joint_state_received_{false};
    bool first_sample_{true};
    bool contact_detected_{false};
    int prep_print_count_{0};
    double current_force_val_{0.0};
    double current_torque_val_{0.0};
    double target_force_{0.0};

    // Command State
    std::vector<double> cmd_pos_;
    std::vector<double> cmd_stiff_;
    std::vector<double> cmd_damp_;

    // Params
    double stiffness_high_, damping_high_;
    double stiffness_low_, damping_low_;
    double mcp_closing_rate_, pip_force_gain_;
    double contact_force_thresh_, lock_torque_thresh_;
    double safety_force_limit_, safety_torque_limit_;
    double ff_effort_val_;
    double force_deadband_;
    double force_alpha_;
    double mcp_min_limit_, mcp_max_limit_;
    double pip_min_limit_, pip_max_limit_;
    std::string force_comp_str_;
    std::string joint_mcp_1_name_, joint_pip_1_name_;
    std::string joint_mcp_2_name_, joint_pip_2_name_;
    
    // Manual Open Positions for each joint (loaded from parameters)
    double open_mcp_1_;
    double open_pip_1_;
    double open_mcp_2_;
    double open_pip_2_;
    
    // Terminal management
    termios original_term_;
    bool terminal_configured_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdaptiveGraspNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}