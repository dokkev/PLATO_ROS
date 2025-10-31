#include <memory>
#include <thread>
#include <atomic>
#include <map>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include <termios.h>
#include <unistd.h>


// joint 1:  pos :0.0 vel: 0.0 eff: 0.0 stiff: 3.0 damp: 0.2
// joint 2:  pos :0.0 vel: 0.0 eff: 0.0 stiff: 3.0 damp: 0.2
// joint 3 :  pos :-0.398 vel: 0.0 eff: 0.0 stiff: 1.0 damp: 0.1
// joint 4:  pos :0.0 vel: 0.0 eff: 0.0 stiff: 1.0 damp: 0.1
// joint 5 :  pos :0.992 vel: 0.0 eff: 0.0 stiff: 1.0 damp: 0.1
// joint 6 :  pos :0.0 vel: 0.0 eff: 0.0 stiff: 1.0 damp: 0.1
// joint 7 :  pos :0.0 vel: 0.0 eff: 0.0 stiff: 1.0 damp: 0.1
// joint 8 :  pos :0.0 vel: 0.0 eff: 0.0 stiff: 1.0 damp: 0.1




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
        current_stiffness_preset_ = "medium";
        current_damping_preset_ = "low";

        // Initialize stiffness and effort vectors with default values
        stiffness_ = stiffness_presets_["medium"];
        damping_ = damping_presets_["low"];
        effort_ff_ = std::vector<double>(8, 0.0);
        last_position_ = std::vector<double>(8, 0.0);

        // Create subscription for position commands
        position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/plato2/plato2_position_controller/commands", 10,
            std::bind(&PositionToImpedanceConverter::position_callback, this, std::placeholders::_1));

        // Create publisher for impedance commands with a volatile QoS profile
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        impedance_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
            "/plato2/joint_impedance_controller/commands", qos_profile);

        // Start keyboard input thread
        keyboard_thread_ = std::thread(&PositionToImpedanceConverter::keyboardInput, this);

        RCLCPP_INFO(this->get_logger(), "Position to Impedance Converter Node started");
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
    void initializePresets() {
        // Stiffness presets
        stiffness_presets_["zero"] = std::vector<double>(8, 0.0);
        stiffness_presets_["low"] = std::vector<double>(8, 0.2);
        stiffness_presets_["medium"] = std::vector<double>(8, 2.0);
        stiffness_presets_["high"] = std::vector<double>(8, 3.5);

        damping_presets_["zero"] = std::vector<double>(8, 0.0);
        damping_presets_["low"] = std::vector<double>(8, 0.1);
        damping_presets_["medium"] = std::vector<double>(8, 0.5);
        damping_presets_["high"] = std::vector<double>(8, 2.0);

        effort_presets_["zero"] = std::vector<double>(8, 0.0);
    }

    void updateStiffnessFromPreset(const std::string& preset) {
        std::lock_guard<std::mutex> lock(param_mutex_);
        if (stiffness_presets_.find(preset) != stiffness_presets_.end()) {
            stiffness_ = stiffness_presets_[preset];
            current_stiffness_preset_ = preset;
            RCLCPP_INFO(this->get_logger(), "Switched stiffness to preset: %s", preset.c_str());
        }
    }

    void updateEffortFromPreset(const std::string& preset) {
        std::lock_guard<std::mutex> lock(param_mutex_);
        if (effort_presets_.find(preset) != effort_presets_.end()) {
            effort_ff_ = effort_presets_[preset];
            current_effort_preset_ = preset;
            RCLCPP_INFO(this->get_logger(), "Switched effort feedforward to preset: %s", preset.c_str());
        }
    }

    void updateDampingFromPreset(const std::string& preset) {
        std::lock_guard<std::mutex> lock(param_mutex_);
        if (damping_presets_.find(preset) != damping_presets_.end()) {
            damping_ = damping_presets_[preset];
            current_damping_preset_ = preset;
            RCLCPP_INFO(this->get_logger(), "Switched damping to preset: %s", preset.c_str());
        }
    }

    void printHelp() {
        RCLCPP_INFO(this->get_logger(), "Keyboard Controls:");
        RCLCPP_INFO(this->get_logger(), "0: Zero stiffness");
        RCLCPP_INFO(this->get_logger(), "1: Low stiffness");
        RCLCPP_INFO(this->get_logger(), "2: Medium stiffness");
        RCLCPP_INFO(this->get_logger(), "3: High stiffness");
        RCLCPP_INFO(this->get_logger(), "4: High damping only");
        RCLCPP_INFO(this->get_logger(), "h: Help");
        RCLCPP_INFO(this->get_logger(), "q: Quit");
    }

    void keyboardInput() {
        configureTerminal();
        char c;
        while (running_ && read(STDIN_FILENO, &c, 1) == 1) {
            switch (c) {
                case '0':
                    updateStiffnessFromPreset("zero");
                    updateDampingFromPreset("zero");

                    break;
                case '1':
                    updateStiffnessFromPreset("low");
                    updateDampingFromPreset("low");
                    break;
                case '2':
                    updateStiffnessFromPreset("medium");
                    updateDampingFromPreset("medium");  
                    break;
                case '3':
                    updateStiffnessFromPreset("high");
                    updateDampingFromPreset("medium");
                    break;
                case '4':
                    updateStiffnessFromPreset("zero");
                    updateDampingFromPreset("high");
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
        }
    }

    void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Store the last received position
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            last_position_ = msg->data;
            has_received_command_ = true;
        }

        // Create and populate the impedance message
        auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
        
        // Lock while accessing the shared variables
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            impedance_msg->stiffness = stiffness_;
            impedance_msg->damping = damping_;
            
            impedance_msg->position = last_position_;
            impedance_msg->velocity = std::vector<double>(8, 0.0);
            impedance_msg->effort_ff = std::vector<double>(8, 0.0);
        }
        
        // Publish the message
        impedance_pub_->publish(*impedance_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published impedance command");
    }

    std::atomic<bool> running_;
    std::atomic<bool> has_received_command_;
    std::thread keyboard_thread_;
    std::mutex param_mutex_;  // Mutex to protect shared data
    
    std::vector<double> stiffness_;
    std::vector<double> damping_;
    std::vector<double> effort_ff_;
    std::vector<double> last_position_;
    
    std::string current_stiffness_preset_;
    std::string current_effort_preset_;
    std::string current_damping_preset_;
    
    std::map<std::string, std::vector<double>> damping_presets_;
    std::map<std::string, std::vector<double>> stiffness_presets_;
    std::map<std::string, std::vector<double>> effort_presets_;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_sub_;
    rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionToImpedanceConverter>());
    rclcpp::shutdown();
    return 0;
}