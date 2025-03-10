#include <memory>
#include <thread>
#include <atomic>
#include <map>
#include <algorithm> // For std::clamp
#include <array>    // For std::array
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include <termios.h>
#include <unistd.h>
#include <mutex>


// Function to configure terminal for immediate keyboard input
void configureTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

// Function to restore terminal settings
void restoreTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

class PositionToImpedanceConverter : public rclcpp::Node {
public:
    PositionToImpedanceConverter()
    : Node("position_control_node"), running_(true), has_received_command_(false)
    {
        // Initialize preset values
        initializePresets();
        
        // Set initial values
        current_stiffness_preset_ = "normal";
        current_effort_preset_ = "zero";
        updateStiffnessFromPreset(current_stiffness_preset_);
        updateEffortFromPreset(current_effort_preset_);

        // Initialize filter parameters
        filter_alpha_ = this->declare_parameter("filter_alpha", 0.2);
        filtered_position_ = std::vector<double>(8, 0.0);
        
        // Create subscription for position commands
        position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/plato2/plato2_position_controller/commands", 10,
            std::bind(&PositionToImpedanceConverter::position_callback, this, std::placeholders::_1));

        // Create publisher for impedance commands
        impedance_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
            "/plato2/joint_impedance_controller/commands", 10);

        // Start keyboard input thread
        keyboard_thread_ = std::thread(&PositionToImpedanceConverter::keyboardInput, this);

        // Create subscription for position commands
        position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/plato2/plato2_position_controller/commands", 10,
            std::bind(&PositionToImpedanceConverter::position_callback, this, std::placeholders::_1));

        // Create publisher for impedance commands
        impedance_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
            "/plato2/joint_impedance_controller/commands", 10);

        // Start keyboard input thread
        keyboard_thread_ = std::thread(&PositionToImpedanceConverter::keyboardInput, this);

        RCLCPP_INFO(this->get_logger(), "Position to Impedance Converter Node started");
        RCLCPP_INFO(this->get_logger(), "Low-Pass Filter enabled with alpha: %f", filter_alpha_);
        RCLCPP_INFO(this->get_logger(), "Joint limits enforced: %s", enforce_joint_limits_ ? "true" : "false");
        
        // Publish initial command with default positions
        republicCommand();
        
        printHelp();
    }

    ~PositionToImpedanceConverter() {
        running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        restoreTerminal();
    }

private:
    // Structure to hold joint limit information
    struct JointLimit {
        double lower;
        double upper;
        std::string name;
    };
    
    void initializePresets() {
        // Stiffness presets
        stiffness_presets_["zero"] = std::vector<double>(8, 0.0);
        stiffness_presets_["soft"] = std::vector<double>(8, 0.5);
        stiffness_presets_["normal"] = std::vector<double>(8, 1.5);
        stiffness_presets_["stiff"] = std::vector<double>(8, 2.5);

        // Effort feedforward presets
        effort_presets_["zero"] = std::vector<double>(8, 0.0);
        effort_presets_["low"] = std::vector<double>{0.0, 0.0, 0.0, -0.4, 0.0, 0.4, 0.0, 0.0};
        effort_presets_["index_pinch"] = std::vector<double>{0.0, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0};
        effort_presets_["middle_pinch"] = std::vector<double>{0.0, 0.0, 0.0, -0.6, 0.0, 0.0, 0.0, 0.6};
        effort_presets_["power_grasp"] = std::vector<double>{0.0, 0.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0};
        effort_presets_["mcp"] = std::vector<double>{0.0, 0.0, -0.2, -0.2, 0.2, 0.2, 0.0, 0.0};


        // thumb: 3,4   index 5,6 
        stiffness_presets_["flick_ready"] = std::vector<double>{0.0, 0.0, 5.0, 5.0, 1.0, 1.0, 0.0, 0.0};
        effort_presets_["flick_ready"] = std::vector<double>{0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0};

        stiffness_presets_["flick"] = std::vector<double>{0.0, 0.0, 0.5, 0.5, 5.0, 5.0, 0.0, 0.0};
        effort_presets_["flick"] = std::vector<double>{0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0};
    }

    // Apply low-pass filter to position data and enforce joint limits
    void applyLowPassFilter(const std::vector<double>& new_position) {
        try {
            // Ensure the new position has the right size
            if (new_position.size() < 8) {
                RCLCPP_WARN(this->get_logger(), "New position vector size (%zu) is less than expected (8)", new_position.size());
                return;
            }
            
            // For the first command, initialize filtered_position_ with the new position
            if (!has_received_command_) {
                filtered_position_ = new_position;
                has_received_command_ = true;
                RCLCPP_INFO(this->get_logger(), "First position command received, initializing filtered position");
            } else {
                // Apply the filter: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
                for (size_t i = 0; i < new_position.size() && i < filtered_position_.size(); i++) {
                    filtered_position_[i] = filter_alpha_ * new_position[i] + (1.0 - filter_alpha_) * filtered_position_[i];
                }
            }
            
            // Apply joint limits after filtering
            applyJointLimits(filtered_position_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in applyLowPassFilter: %s", e.what());
        }
    }
    
    // Apply joint limits to position data
    void applyJointLimits(std::vector<double>& position) {
        // Skip if joint limits are disabled
        if (!enforce_joint_limits_) {
            return;
        }
        
        if (position.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Position vector size (%zu) is less than expected (8), cannot apply joint limits", position.size());
            return;
        }
        
        // Apply joint limits using the predefined limits array
        for (size_t i = 0; i < std::min(position.size(), joint_limits_.size()); i++) {
            const double original_value = position[i];
            position[i] = std::clamp(position[i], joint_limits_[i].lower, joint_limits_[i].upper);
            
            // Log if a position was limited
            if (original_value != position[i]) {
                RCLCPP_DEBUG(this->get_logger(), 
                            "Joint %zu (%s) position limited from %.4f to %.4f [limits: %.4f, %.4f]",
                            i+1, joint_limits_[i].name.c_str(), original_value, position[i], 
                            joint_limits_[i].lower, joint_limits_[i].upper);
            }
        }
    }

    void updateStiffnessFromPreset(const std::string& preset) {
        try {
            if (stiffness_presets_.find(preset) != stiffness_presets_.end()) {
                stiffness_ = stiffness_presets_[preset];
                current_stiffness_preset_ = preset;
                RCLCPP_INFO(this->get_logger(), "Switched stiffness to preset: %s", preset.c_str());
                republicCommand();
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown stiffness preset: %s", preset.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in updateStiffnessFromPreset: %s", e.what());
        }
    }

    void updateEffortFromPreset(const std::string& preset) {
        try {
            if (effort_presets_.find(preset) != effort_presets_.end()) {
                effort_ff_ = effort_presets_[preset];
                current_effort_preset_ = preset;
                RCLCPP_INFO(this->get_logger(), "Switched effort feedforward to preset: %s", preset.c_str());
                republicCommand();
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown effort preset: %s", preset.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in updateEffortFromPreset: %s", e.what());
        }
    }

    void republicCommand() {
        try {
            // Check if stiffness and effort_ff are properly initialized
            if (stiffness_.size() < 8 || effort_ff_.size() < 8 || filtered_position_.size() < 8) {
                RCLCPP_ERROR(this->get_logger(), "Cannot republish command: vectors not properly initialized");
                return;
            }
            
            auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
        
            impedance_msg->stiffness = stiffness_;
            
            // Resize damping to match the size of stiffness
            impedance_msg->damping.resize(stiffness_.size());
            for (size_t i = 0; i < stiffness_.size(); i++) {
                impedance_msg->damping[i] = stiffness_[i] * 1.2;
            }
            
            impedance_msg->position = filtered_position_;  // Use filtered position
            impedance_msg->velocity = std::vector<double>(8, 0.0);
            impedance_msg->effort_ff = effort_ff_;
            
            impedance_pub_->publish(*impedance_msg);
            RCLCPP_DEBUG(this->get_logger(), "Republished impedance command with updated parameters");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in republishCommand: %s", e.what());
        }
    }
    
    void printHelp() {
        RCLCPP_INFO(this->get_logger(), "Keyboard Controls:");
        RCLCPP_INFO(this->get_logger(), "0: Zero stiffness");
        RCLCPP_INFO(this->get_logger(), "1: Soft stiffness");
        RCLCPP_INFO(this->get_logger(), "2: Normal stiffness");
        RCLCPP_INFO(this->get_logger(), "3: Stiff stiffness");
        RCLCPP_INFO(this->get_logger(), "4: Index Pinch");
        RCLCPP_INFO(this->get_logger(), "5: Middle Pinch");
        RCLCPP_INFO(this->get_logger(), "6: Power grasp");
        RCLCPP_INFO(this->get_logger(), "f: Adjust filter alpha (cycles through presets)");
        RCLCPP_INFO(this->get_logger(), "l: Toggle joint limit enforcement");
        RCLCPP_INFO(this->get_logger(), "h: Show this help");
        RCLCPP_INFO(this->get_logger(), "q: Quit");
    }

    void keyboardInput() {
        try {
            configureTerminal();
            char c;
            while (running_ && read(STDIN_FILENO, &c, 1) == 1) {
                try {
                    std::lock_guard<std::mutex> lock(mutex_);  // Lock mutex before modifying shared variables
            
                    switch (c) {
                        case '0':
                            updateStiffnessFromPreset("zero");
                            updateEffortFromPreset("zero");
                            break;
                        case '1':
                            updateStiffnessFromPreset("soft");
                            updateEffortFromPreset("zero");
                            break;
                        case '2':
                            updateStiffnessFromPreset("normal");
                            updateEffortFromPreset("zero");
                            break;
                        case '3':
                            updateStiffnessFromPreset("stiff");
                            updateEffortFromPreset("zero");
                            break;
                        case '4':
                            updateEffortFromPreset("index_pinch");
                            break;
                        case '5':
                            updateEffortFromPreset("middle_pinch");
                            break;
                        case '6':
                            updateEffortFromPreset("power_grasp");
                            break;
                        case 'f':
                            // Cycle through filter alpha presets
                            if (filter_alpha_ == 0.8) filter_alpha_ = 0.1;
                            else if (filter_alpha_ == 0.5) filter_alpha_ = 0.8;
                            else if (filter_alpha_ == 0.2) filter_alpha_ = 0.5;
                            else if (filter_alpha_ == 0.1) filter_alpha_ = 0.2;
                            else filter_alpha_ = 0.2; // Default
                            RCLCPP_INFO(this->get_logger(), "Changed filter alpha to: %f", filter_alpha_);
                            break;
                        case 'l':
                            enforce_joint_limits_ = !enforce_joint_limits_;
                            RCLCPP_INFO(this->get_logger(), "Joint limit enforcement %s", 
                                        enforce_joint_limits_ ? "enabled" : "disabled");
                            break;
                        case 'h':
                            printHelp();
                            break;
                        case 'q':
                            RCLCPP_INFO(this->get_logger(), "Shutting down...");
                            running_ = false;
                            rclcpp::shutdown();
                            break;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception in keyboardInput switch: %s", e.what());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in keyboardInput: %s", e.what());
        }
        
        // Always restore terminal settings before exiting
        try {
            restoreTerminal();
        } catch (...) {
            // Silently ignore any errors during restore
        }
    }
    
    void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        try {
            std::lock_guard<std::mutex> lock(mutex_);  // Lock mutex
        
            // Copy position data
            if (msg->data.size() < 8) {
                RCLCPP_WARN(this->get_logger(), "Received position command with %zu joints (expected 8), padding with zeros", msg->data.size());
                last_position_ = std::vector<double>(8, 0.0);
                for (size_t i = 0; i < msg->data.size(); i++) {
                    last_position_[i] = msg->data[i];
                }
            } else {
                last_position_ = msg->data;
            }
            
            // Apply low-pass filter and joint limits
            applyLowPassFilter(last_position_);
        
            // Create and publish impedance message
            auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
        
            impedance_msg->stiffness = stiffness_;
            impedance_msg->damping.resize(stiffness_.size());
            for (size_t i = 0; i < stiffness_.size(); i++) {
                impedance_msg->damping[i] = stiffness_[i] * 1.2;
            }
        
            impedance_msg->position = filtered_position_;  // Use filtered position
            impedance_msg->velocity = std::vector<double>(8, 0.0);
            impedance_msg->effort_ff = effort_ff_;
        
            impedance_pub_->publish(*impedance_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Published impedance command with filtered position");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in position_callback: %s", e.what());
        }
    }
    
    std::atomic<bool> running_;
    std::atomic<bool> has_received_command_;
    std::atomic<bool> enforce_joint_limits_{true};
    std::mutex mutex_;
    std::thread keyboard_thread_;
    std::vector<double> stiffness_;
    std::vector<double> effort_ff_;
    std::vector<double> last_position_;
    std::vector<double> filtered_position_;  // Store filtered position
    double filter_alpha_;  // Filter coefficient: 0 (no new data) to 1 (no filtering)
    std::string current_stiffness_preset_;
    std::string current_effort_preset_;
    std::map<std::string, std::vector<double>> stiffness_presets_;
    std::map<std::string, std::vector<double>> effort_presets_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
    rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
    
    // Joint limit definitions based on the URDF file
    std::array<JointLimit, 8> joint_limits_ = {{
        {-0.349066, 1.22173, "thumb_metacarpal"},     // joint1
        {-0.785398, 0.785398, "thumb_actuators"},     // joint2
        {-1.047198, 1.047198, "thumb_proximal"},      // joint3
        {-2.094395, 0.0, "thumb_distal"},             // joint4
        {-1.047198, 1.047198, "index_proximal"},      // joint5
        {0.0, 2.094395, "index_distal"},              // joint6
        {-1.047198, 1.047198, "middle_proximal"},     // joint7
        {0.0, 2.094395, "middle_distal"}              // joint8
    }};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionToImpedanceConverter>());
    rclcpp::shutdown();
    return 0;
}