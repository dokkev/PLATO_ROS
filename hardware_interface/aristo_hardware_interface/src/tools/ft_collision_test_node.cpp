#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <plato2_interfaces/msg/impedance_commands.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <cmath>
#include <chrono>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>

class FTCollisionTestNode : public rclcpp::Node {
public:
    FTCollisionTestNode()
    : Node("ft_collision_test_node"),
                is_moving_(false),
                collision_detected_(false),
                current_joint5_position_(0.0)
    {
        // Declare parameters
                this->declare_parameter<double>("joint5_velocity", 0.05);  // rad/s
                this->declare_parameter<double>("joint5_start", -0.36); // rad
                    this->declare_parameter<double>("joint6_start", 0.37);   // rad (baseline when decoupling)
        this->declare_parameter<double>("force_threshold", 0.6);   // N
        this->declare_parameter<double>("torque_threshold", 0.02);  // Nm
                this->declare_parameter<double>("joint5_min", -1.0);       // rad
                this->declare_parameter<double>("joint5_max", 1.0);        // rad
        this->declare_parameter<double>("control_rate", 100.0);    // Hz
        this->declare_parameter<bool>("auto_start", false);
        this->declare_parameter<std::string>("ft_topic", "/plato2/ft_sensor2/wrench");
                this->declare_parameter<bool>("decouple_joint6", true); // cancel PIP effect to keep fingertip world-stationary
        this->declare_parameter<std::string>("force_component", "norm"); // norm|fx|fy|fz
        this->declare_parameter<std::string>("torque_component", "norm"); // norm|tx|ty|tz
        
        // Get parameters
                joint5_velocity_ = this->get_parameter("joint5_velocity").as_double();
                joint5_start_ = this->get_parameter("joint5_start").as_double();
                joint6_start_ = this->get_parameter("joint6_start").as_double();
        force_threshold_ = this->get_parameter("force_threshold").as_double();
        torque_threshold_ = this->get_parameter("torque_threshold").as_double();
                joint5_min_ = this->get_parameter("joint5_min").as_double();
                joint5_max_ = this->get_parameter("joint5_max").as_double();
        double control_rate = this->get_parameter("control_rate").as_double();
        bool auto_start = this->get_parameter("auto_start").as_bool();
        ft_topic_ = this->get_parameter("ft_topic").as_string();
                decouple_joint6_ = this->get_parameter("decouple_joint6").as_bool();
        force_component_ = this->get_parameter("force_component").as_string();
        torque_component_ = this->get_parameter("torque_component").as_string();

                current_joint5_position_ = joint5_start_;
                initial_joint5_ = joint5_start_;
                initial_joint6_ = joint6_start_;
        
        // Subscribe to FT sensor data published by ft_sensor_bringup
        ft_sensor_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            ft_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&FTCollisionTestNode::ft_sensor_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to smoothed joint states for effort tracking
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/plato2/joint_states_smoothed",
            rclcpp::SensorDataQoS(),
            std::bind(&FTCollisionTestNode::joint_state_callback, this, std::placeholders::_1)
        );
        
        // Publisher for impedance commands
        impedance_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
            "/plato2/joint_impedance_controller/commands",
            10
        );
        
        // Timer for control loop
        auto period = std::chrono::duration<double>(1.0 / control_rate);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&FTCollisionTestNode::control_loop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "=== FT Collision Test Node Started ===");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Joint 5 Velocity: %.3f rad/s", joint5_velocity_);
        RCLCPP_INFO(this->get_logger(), "  - Joint 5 Start: %.3f rad", joint5_start_);
        RCLCPP_INFO(this->get_logger(), "  - Joint 6 Start (baseline): %.3f rad", joint6_start_);
        RCLCPP_INFO(this->get_logger(), "  - Force Threshold: %.2f N", force_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - Torque Threshold: %.3f Nm", torque_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - Joint 5 Range: [%.2f, %.2f] rad", joint5_min_, joint5_max_);
        RCLCPP_INFO(this->get_logger(), "  - Control Rate: %.1f Hz", control_rate);
        RCLCPP_INFO(this->get_logger(), "  - FT Topic: %s", ft_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Decouple Joint6 (world hold): %s", decouple_joint6_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - Force Component: %s", force_component_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Torque Component: %s", torque_component_.c_str());
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Commands:");
        RCLCPP_INFO(this->get_logger(), "  's' - Start/Resume movement");
        RCLCPP_INFO(this->get_logger(), "  'p' - Pause movement");
        RCLCPP_INFO(this->get_logger(), "  'r' - Reset collision flag");
        RCLCPP_INFO(this->get_logger(), "  '+' - Increase velocity by 0.05 rad/s");
        RCLCPP_INFO(this->get_logger(), "  '-' - Decrease velocity by 0.05 rad/s");
        RCLCPP_INFO(this->get_logger(), "  'q' - Quit");
        RCLCPP_INFO(this->get_logger(), "");
        
        if (auto_start) {
            is_moving_ = true;
            RCLCPP_INFO(this->get_logger(), "Auto-start enabled. Movement started.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Press 's' to start movement");
        }   
        
        // Keyboard input thread
        keyboard_thread_ = std::thread(&FTCollisionTestNode::keyboard_input, this);
    }
    
    ~FTCollisionTestNode() {
        running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        
        // Stop the robot
        auto msg = plato2_interfaces::msg::ImpedanceCommands();
        msg.position.resize(8, 0.0);
        msg.velocity.resize(8, 0.0);
        msg.stiffness.resize(8, 2.0);
        msg.damping.resize(8, 0.1);
        msg.effort_ff.resize(8, 0.0);
        impedance_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Node shutting down...");
    }

private:
    void ft_sensor_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        // Store latest FT sensor reading
        latest_wrench_ = msg;
        
        // Calculate force and torque magnitudes
        double force_mag = select_force_value(msg->wrench);
        double torque_mag = select_torque_value(msg->wrench);
        
        // Check for collision
        if (is_moving_ && !collision_detected_) {
            if (force_mag > force_threshold_ || torque_mag > torque_threshold_) {
                collision_detected_ = true;
                is_moving_ = false;
                collision_time_ = this->now();
                
                RCLCPP_WARN(this->get_logger(), 
                    "\n!!! COLLISION DETECTED (FT) !!!\n"
                    "  Force: %.3f N (threshold: %.2f N)\n"
                    "  Torque: %.3f Nm (threshold: %.3f Nm)\n"
                    "  Joint 5 Position: %.3f rad\n"
                    "  Waiting 2 seconds before returning to start...",
                    force_mag, force_threshold_,
                    torque_mag, torque_threshold_,
                    current_joint5_position_);
            }
        }
    }
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Track joint5 and joint6 effort range for monitoring
        if (msg->effort.size() > 0 && msg->name.size() == msg->effort.size()) {
            // Find joint5 and joint6 indices by name
            int joint5_idx = -1;
            int joint6_idx = -1;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "joint5") {
                    joint5_idx = i;
                } else if (msg->name[i] == "joint6") {
                    joint6_idx = i;
                }
            }
            
            if (joint5_idx < 0 || joint6_idx < 0) {
                return;
            }
            
            double joint5_effort = msg->effort[joint5_idx];
            double joint6_effort = msg->effort[joint6_idx];
            
            // Initialize or update min/max tracking
            if (!joint5_effort_initialized_) {
                joint5_effort_min_ = joint5_effort;
                joint5_effort_max_ = joint5_effort;
                joint5_effort_initialized_ = true;
            } else {
                joint5_effort_min_ = std::min(joint5_effort_min_, joint5_effort);
                joint5_effort_max_ = std::max(joint5_effort_max_, joint5_effort);
            }
            
            if (!joint6_effort_initialized_) {
                joint6_effort_min_ = joint6_effort;
                joint6_effort_max_ = joint6_effort;
                joint6_effort_initialized_ = true;
            } else {
                joint6_effort_min_ = std::min(joint6_effort_min_, joint6_effort);
                joint6_effort_max_ = std::max(joint6_effort_max_, joint6_effort);
            }
        }
    }
    
    void control_loop() {
        // Handle return-to-start after 2-second delay
        if (collision_detected_ && !returning_to_start_) {
            double elapsed = (this->now() - collision_time_).seconds();
            if (elapsed >= 2.0) {
                returning_to_start_ = true;
                RCLCPP_INFO(this->get_logger(), "Commanding immediate return to start position.");
            }
        }
        
        if (returning_to_start_) {
            // Command immediate return to start
            auto msg = plato2_interfaces::msg::ImpedanceCommands();
            msg.position.resize(8, 0.0);
            msg.velocity.resize(8, 0.0);
            msg.stiffness.resize(8, 2.0);
            msg.damping.resize(8, 0.1);
            msg.effort_ff.resize(8, 0.0);
            
            msg.position[4] = joint5_start_;  // Command to start position
            double joint6_cmd = initial_joint6_ - (joint5_start_ - initial_joint5_);
            if (decouple_joint6_) {
                msg.position[5] = joint6_cmd;
            } else {
                msg.position[5] = initial_joint6_;
            }
            
            impedance_pub_->publish(msg);
            returning_to_start_ = false;
            RCLCPP_INFO(this->get_logger(), "Return-to-start command sent.");
            return;
        }
        
        if (!is_moving_ || collision_detected_) {
            return;
        }
        
        double dt = 1.0 / this->get_parameter("control_rate").as_double();
        current_joint5_position_ += joint5_velocity_ * dt;
        
        // Check limits
        if (current_joint5_position_ >= joint5_max_) {
            current_joint5_position_ = joint5_max_;
            joint5_velocity_ = -std::abs(joint5_velocity_);
            RCLCPP_INFO(this->get_logger(), "Reached max limit. Reversing direction.");
        } else if (current_joint5_position_ <= joint5_min_) {
            current_joint5_position_ = joint5_min_;
            joint5_velocity_ = std::abs(joint5_velocity_);
            RCLCPP_INFO(this->get_logger(), "Reached min limit. Reversing direction.");
        }
        
        // Publish impedance command
        auto msg = plato2_interfaces::msg::ImpedanceCommands();
        msg.position.resize(8, 0.0);
        msg.velocity.resize(8, 0.0);
        msg.stiffness.resize(8, 2.0);  // Moderate stiffness
        msg.damping.resize(8, 0.1);     // Moderate damping
        msg.effort_ff.resize(8, 0.0);
        
        // Set joint 5 (index 4, since joint numbering starts from 1) position
        msg.position[4] = current_joint5_position_;

        // Joint6 handling:
        // - decouple_joint6_: cancel PIP effect to keep fingertip world-stationary by commanding opposite around initial offsets
        //   joint6_cmd = initial_joint6_ - (joint5 - initial_joint5_)
        double joint6_cmd = initial_joint6_ - (current_joint5_position_ - initial_joint5_);
        if (decouple_joint6_) {
            msg.position[5] = joint6_cmd;  // Joint6 at index 5
        } else {
            // Hold at provided baseline to avoid commanding zero unexpectedly
            msg.position[5] = initial_joint6_;  // Joint6 at index 5
        }
        
        impedance_pub_->publish(msg);
        
        // Periodic status update (every 1 second while moving)
        static auto last_status_time = this->now();
        if ((this->now() - last_status_time).seconds() >= 1.0) {
            if (latest_wrench_) {
                double force_mag = select_force_value(latest_wrench_->wrench);
                double torque_mag = select_torque_value(latest_wrench_->wrench);
                double joint5_range = joint5_effort_max_ - joint5_effort_min_;
                double joint6_range = joint6_effort_max_ - joint6_effort_min_;
                
                RCLCPP_INFO(this->get_logger(), 
                    "Moving: J5=%.3f rad, Vel=%.3f rad/s | FT: Force=%.3f N (thr:%.2f) Torque=%.3f Nm (thr:%.3f) | Effort: J5_range=%.3f Nm, J6_range=%.3f Nm",
                    current_joint5_position_, joint5_velocity_, 
                    force_mag, force_threshold_,
                    torque_mag, torque_threshold_,
                    joint5_range, joint6_range);
            }
            last_status_time = this->now();
        }
    }

    void publish_stop_command() {
        auto msg = plato2_interfaces::msg::ImpedanceCommands();
        msg.position.assign(8, 0.0);
        msg.velocity.assign(8, 0.0);
        msg.stiffness.assign(8, 2.0);
        msg.damping.assign(8, 0.1);
        msg.effort_ff.assign(8, 0.0);
        impedance_pub_->publish(msg);
    }

    double select_force_value(const geometry_msgs::msg::Wrench & w) const {
        if (force_component_ == "fx") return std::abs(w.force.x);
        if (force_component_ == "fy") return std::abs(w.force.y);
        if (force_component_ == "fz") return std::abs(w.force.z);
        // default norm
        return std::sqrt(w.force.x*w.force.x + w.force.y*w.force.y + w.force.z*w.force.z);
    }

    double select_torque_value(const geometry_msgs::msg::Wrench & w) const {
        if (torque_component_ == "tx") return std::abs(w.torque.x);
        if (torque_component_ == "ty") return std::abs(w.torque.y);
        if (torque_component_ == "tz") return std::abs(w.torque.z);
        // default norm
        return std::sqrt(w.torque.x*w.torque.x + w.torque.y*w.torque.y + w.torque.z*w.torque.z);
    }
    
    void keyboard_input() {
        // Configure terminal for non-blocking input
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        
        char c;
        while (running_ && rclcpp::ok()) {
            if (read(STDIN_FILENO, &c, 1) == 1) {
                switch(c) {
                    case 's':
                    case 'S':
                        if (!is_moving_) {
                            if (collision_detected_) {
                                RCLCPP_WARN(this->get_logger(), 
                                    "Cannot start: Collision flag set. Press 'r' to reset first.");
                            } else {
                                is_moving_ = true;
                                // Reset effort trackers for new motion cycle
                                joint5_effort_min_ = 0.0;
                                joint5_effort_max_ = 0.0;
                                joint6_effort_min_ = 0.0;
                                joint6_effort_max_ = 0.0;
                                joint5_effort_initialized_ = false;
                                joint6_effort_initialized_ = false;
                                RCLCPP_INFO(this->get_logger(), "Movement STARTED");
                            }
                        }
                        break;
                    
                    case 'p':
                    case 'P':
                        if (is_moving_) {
                            is_moving_ = false;
                            RCLCPP_INFO(this->get_logger(), "Movement PAUSED");
                        }
                        break;
                    
                    case 'r':
                    case 'R':
                        collision_detected_ = false;
                        returning_to_start_ = false;
                        RCLCPP_INFO(this->get_logger(), "Collision flag RESET");
                        break;
                    
                    case '+':
                    case '=':
                        joint5_velocity_ = (joint5_velocity_ > 0) ? 
                            joint5_velocity_ + 0.05 : joint5_velocity_ - 0.05;
                        RCLCPP_INFO(this->get_logger(), "Velocity: %.3f rad/s", joint5_velocity_);
                        break;
                    
                    case '-':
                    case '_':
                        joint5_velocity_ = (joint5_velocity_ > 0) ? 
                            joint5_velocity_ - 0.05 : joint5_velocity_ + 0.05;
                        if (std::abs(joint5_velocity_) < 0.05) {
                            joint5_velocity_ = (joint5_velocity_ > 0) ? 0.05 : -0.05;
                        }
                        RCLCPP_INFO(this->get_logger(), "Velocity: %.3f rad/s", joint5_velocity_);
                        break;
                    
                    case 'q':
                    case 'Q':
                        RCLCPP_INFO(this->get_logger(), "Quit requested");
                        rclcpp::shutdown();
                        running_ = false;
                        break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
    
    // Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sensor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    std::atomic<bool> is_moving_;
    std::atomic<bool> collision_detected_;
    std::atomic<bool> returning_to_start_{false};
    std::atomic<bool> running_{true};
    double current_joint5_position_;
    double joint5_velocity_;
    double joint5_start_;
    double joint6_start_;
    double initial_joint5_;
    double initial_joint6_;
    rclcpp::Time collision_time_;
    geometry_msgs::msg::WrenchStamped::SharedPtr latest_wrench_;
    std::string ft_topic_;
    bool decouple_joint6_;
    std::string force_component_;
    std::string torque_component_;
    
    // Parameters
    double force_threshold_;
    double torque_threshold_;
    double joint5_min_;
    double joint5_max_;
    
    // Joint effort tracking
    double joint5_effort_min_{0.0};
    double joint5_effort_max_{0.0};
    bool joint5_effort_initialized_{false};
    double joint6_effort_min_{0.0};
    double joint6_effort_max_{0.0};
    bool joint6_effort_initialized_{false};
    
    // Keyboard thread
    std::thread keyboard_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTCollisionTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
