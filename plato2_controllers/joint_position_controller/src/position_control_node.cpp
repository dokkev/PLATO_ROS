#include <memory>
#include <thread>
#include <atomic>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include <termios.h>
#include <unistd.h>

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
        stiffness_presets_["soft"] = std::vector<double>(8, 0.0);
        stiffness_presets_["normal"] = std::vector<double>(8, 1.2);
        stiffness_presets_["stiff"] = std::vector<double>(8, 2.4);

        // Effort feedforward presets
        effort_presets_["zero"] = std::vector<double>(8, 0.0);
        effort_presets_["low"] = std::vector<double>{0.0, 0.0, 0.0, -0.4, 0.0, 0.4, 0.0, 0.0};
        effort_presets_["high"] = std::vector<double>{0.0, 0.0, 0.0, -0.4, 1.2, 1.2, 0.0, 0.0};
        effort_presets_["mcp"] = std::vector<double>{0.0, 0.0, -0.2, -0.2, 0.2, 0.2, 0.0, 0.0};
    }

    void updateStiffnessFromPreset(const std::string& preset) {
        if (stiffness_presets_.find(preset) != stiffness_presets_.end()) {
            stiffness_ = stiffness_presets_[preset];
            RCLCPP_INFO(this->get_logger(), "Switched stiffness to preset: %s", preset.c_str());
            republishCommand();
        }
    }

    void updateEffortFromPreset(const std::string& preset) {
        if (effort_presets_.find(preset) != effort_presets_.end()) {
            effort_ff_ = effort_presets_[preset];
            RCLCPP_INFO(this->get_logger(), "Switched effort feedforward to preset: %s", preset.c_str());
            republishCommand();
        }
    }

    void republishCommand() {
        if (has_received_command_) {
            auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
            
            impedance_msg->stiffness = stiffness_;
  
            impedance_msg->damping = stiffness_;
            impedance_msg->position = last_position_;
            impedance_msg->velocity = std::vector<double>(8, 0.0);
            impedance_msg->effort_ff = effort_ff_;

            impedance_pub_->publish(*impedance_msg);
            RCLCPP_DEBUG(this->get_logger(), "Republished impedance command with updated parameters");
        }
    }

    void printHelp() {
        RCLCPP_INFO(this->get_logger(), "Keyboard Controls:");
        RCLCPP_INFO(this->get_logger(), "1: Soft stiffness");
        RCLCPP_INFO(this->get_logger(), "2: Normal stiffness");
        RCLCPP_INFO(this->get_logger(), "3: Stiff stiffness");
        RCLCPP_INFO(this->get_logger(), "4: Zero effort feedforward");
        RCLCPP_INFO(this->get_logger(), "5: Low effort feedforward");
        RCLCPP_INFO(this->get_logger(), "6: High effort feedforward");
        RCLCPP_INFO(this->get_logger(), "7: MCP Low effort feedforward");
        RCLCPP_INFO(this->get_logger(), "h: Show this help");
        RCLCPP_INFO(this->get_logger(), "q: Quit");
    }

    void keyboardInput() {
        configureTerminal();
        char c;
        while (running_ && read(STDIN_FILENO, &c, 1) == 1) {
            switch (c) {
                case '1':
                    updateStiffnessFromPreset("soft");
                    break;
                case '2':
                    updateStiffnessFromPreset("normal");
                    break;
                case '3':
                    updateStiffnessFromPreset("stiff");
                    break;
                case '4':
                    updateEffortFromPreset("zero");
                    break;
                case '5':
                    updateEffortFromPreset("low");
                    break;
                case '6':
                    updateEffortFromPreset("high");
                    break;
                case '7':
                    updateEffortFromPreset("mcp");
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
        last_position_ = msg->data;
        has_received_command_ = true;

        auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
        
        impedance_msg->stiffness = stiffness_;
        impedance_msg->damping = std::vector<double>(8, 1.0);  // Fixed damping
        impedance_msg->position = last_position_;
        impedance_msg->velocity = std::vector<double>(8, 0.0);
        impedance_msg->effort_ff = effort_ff_;

        impedance_pub_->publish(*impedance_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published impedance command");
    }

    std::atomic<bool> running_;
    std::atomic<bool> has_received_command_;
    std::thread keyboard_thread_;
    std::vector<double> stiffness_;
    std::vector<double> effort_ff_;
    std::vector<double> last_position_;
    std::string current_stiffness_preset_;
    std::string current_effort_preset_;
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