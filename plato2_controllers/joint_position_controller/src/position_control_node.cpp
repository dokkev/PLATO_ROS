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
        // Initialize vectors with correct size
        stiffness_ = std::vector<double>(8, 0.0);
        effort_ff_ = std::vector<double>(8, 0.0);
        last_position_ = std::vector<double>(8, 0.0);
        filtered_position_ = std::vector<double>(8, 0.0);
        
        // Initialize filter parameters
        filter_alpha_ = this->declare_parameter("filter_alpha", 0.2);
        RCLCPP_INFO(this->get_logger(), "Low-pass filter alpha: %f", filter_alpha_);
        
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
        stiffness_presets_["zero"] = std::vector<double>(8, 0.0);
        stiffness_presets_["soft"] = std::vector<double>(8, 0.5);
        stiffness_presets_["normal"] = std::vector<double>(8, 1.5);
        stiffness_presets_["stiff"] = std::vector<double>(8, 2.5);

        // Effort feedforward presets
        effort_presets_["zero"] = std::vector<double>(8, 0.0);
        effort_presets_["low"] = std::vector<double>{0.0, 0.0, 0.0, -0.4, 0.0, 0.4, 0.0, 0.0};
        effort_presets_["power_grasp"] = std::vector<double>{0.0, 0.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0};
        effort_presets_["mcp"] = std::vector<double>{0.0, 0.0, -0.2, -0.2, 0.2, 0.2, 0.0, 0.0};

        // thumb: 3,4   index 5,6 
        stiffness_presets_["flick_ready"] = std::vector<double>{0.0, 0.0, 5.0, 5.0, 1.0, 1.0, 0.0, 0.0};
        effort_presets_["flick_ready"] = std::vector<double>{0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0};

        stiffness_presets_["flick"] = std::vector<double>{0.0, 0.0, 0.5, 0.5, 5.0, 5.0, 0.0, 0.0};
        effort_presets_["flick"] = std::vector<double>{0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0};
    }

    void updateStiffnessFromPreset(const std::string& preset) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (stiffness_presets_.find(preset) != stiffness_presets_.end()) {
            stiffness_ = stiffness_presets_[preset];
            current_stiffness_preset_ = preset;
            RCLCPP_INFO(this->get_logger(), "Switched stiffness to preset: %s", preset.c_str());
            republishCommand();
        }
    }

    void updateEffortFromPreset(const std::string& preset) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (effort_presets_.find(preset) != effort_presets_.end()) {
            effort_ff_ = effort_presets_[preset];
            current_effort_preset_ = preset;
            RCLCPP_INFO(this->get_logger(), "Switched effort feedforward to preset: %s", preset.c_str());
            republishCommand();
        }
    }

    // Apply low-pass filter to position data
    void applyLowPassFilter(const std::vector<double>& new_position, std::vector<double>& filtered_position) {
        // First-time initialization
        if (!has_received_command_) {
            filtered_position = new_position;
            return;
        }

        // Apply first-order low-pass filter: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
        for (size_t i = 0; i < new_position.size() && i < filtered_position.size(); i++) {
            filtered_position[i] = filter_alpha_ * new_position[i] + (1.0 - filter_alpha_) * filtered_position[i];
        }
    }

    void republishCommand() {
        // Already locked by caller
        if (has_received_command_) {
            auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
            
            // Initialize all vectors to the correct size
            impedance_msg->stiffness = stiffness_;
            impedance_msg->damping.resize(8);
            
            for (size_t i = 0; i < stiffness_.size(); i++) {
                impedance_msg->damping[i] = stiffness_[i];
            }
            
            impedance_msg->position = filtered_position_;
            impedance_msg->velocity.resize(8, 0.0);
            impedance_msg->effort_ff = effort_ff_;

            // Set the position of 0 and 1 to 0.0
            impedance_msg->position[0] = 0.0;
            impedance_msg->position[1] = 0.0;

            impedance_pub_->publish(*impedance_msg);
            RCLCPP_DEBUG(this->get_logger(), "Republished impedance command with updated parameters");
        }
    }

    void printHelp() {
        RCLCPP_INFO(this->get_logger(), "Keyboard Controls:");
        RCLCPP_INFO(this->get_logger(), "0: Zero stiffness");
        RCLCPP_INFO(this->get_logger(), "1: Soft stiffness");
        RCLCPP_INFO(this->get_logger(), "2: Normal stiffness");
        RCLCPP_INFO(this->get_logger(), "3: Stiff stiffness");
        RCLCPP_INFO(this->get_logger(), "4: Flick Ready");
        RCLCPP_INFO(this->get_logger(), "5: Flick");
        RCLCPP_INFO(this->get_logger(), "6: Power grasp");
        RCLCPP_INFO(this->get_logger(), "7: MCP Low effort feedforward");
        RCLCPP_INFO(this->get_logger(), "a: Decrease filter alpha (smoother)");
        RCLCPP_INFO(this->get_logger(), "d: Increase filter alpha (more responsive)");
        RCLCPP_INFO(this->get_logger(), "h: Show this help");
        RCLCPP_INFO(this->get_logger(), "q: Quit");
    }

    void keyboardInput() {
        configureTerminal();
        char c;
        while (running_ && read(STDIN_FILENO, &c, 1) == 1) {
            switch (c) {
                case '0':
                    updateStiffnessFromPreset("zero");
                    updateEffortFromPreset("zero");
                    break;
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
                    updateStiffnessFromPreset("flick_ready");
                    updateEffortFromPreset("flick_ready");
                    break;
                case '5':
                    updateStiffnessFromPreset("flick");
                    updateEffortFromPreset("flick");
                    break;
                case '6':
                    updateEffortFromPreset("power_grasp");
                    break;
                case '7':
                    updateEffortFromPreset("mcp");
                    break;
                case 'a':
                    // Decrease alpha (smoother but slower response)
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        filter_alpha_ = std::max(0.05, filter_alpha_ - 0.05);
                        RCLCPP_INFO(this->get_logger(), "Filter alpha decreased to: %f", filter_alpha_);
                    }
                    break;
                case 'd':
                    // Increase alpha (more responsive but less smooth)
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        filter_alpha_ = std::min(1.0, filter_alpha_ + 0.05);
                        RCLCPP_INFO(this->get_logger(), "Filter alpha increased to: %f", filter_alpha_);
                    }
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
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Check if the received message has the expected size
        if (msg->data.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Received position command with wrong size: %zu (expected at least 8)", 
                      msg->data.size());
            return;
        }
        
        // Store the last received position
        last_position_ = msg->data;
        
        // Apply low-pass filter to smooth the position data
        applyLowPassFilter(last_position_, filtered_position_);
        has_received_command_ = true;

        auto impedance_msg = std::make_unique<plato2_interfaces::msg::ImpedanceCommands>();
        
        impedance_msg->stiffness = stiffness_;
        impedance_msg->damping.resize(8);
        
        for (size_t i = 0; i < stiffness_.size(); i++) {
            impedance_msg->damping[i] = stiffness_[i]; // Fixed damping
        }

        // Use filtered position instead of raw position
        impedance_msg->position = filtered_position_;
        impedance_msg->velocity.resize(8, 0.0);
        impedance_msg->effort_ff = effort_ff_;

        // Set the position of 0 and 1 to 0.0
        impedance_msg->position[0] = 0.0;
        impedance_msg->position[1] = 0.0;

        impedance_pub_->publish(*impedance_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published impedance command with filtered position");
    }

    std::atomic<bool> running_;
    std::atomic<bool> has_received_command_;
    std::thread keyboard_thread_;
    std::mutex data_mutex_;
    std::vector<double> stiffness_;
    std::vector<double> effort_ff_;
    std::vector<double> last_position_;
    std::vector<double> filtered_position_;
    double filter_alpha_;  // Low-pass filter parameter (0.0-1.0)
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