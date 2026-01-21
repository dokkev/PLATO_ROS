#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <deque>
#include <vector>
#include <numeric>
#include <cmath>

class JointStateMovingAverageNode : public rclcpp::Node {
public:
    JointStateMovingAverageNode()
    : Node("joint_state_moving_average_node")
    {
        // Declare parameters
        this->declare_parameter<int>("moving_average_window", 5);  // Number of samples to average
        this->declare_parameter<std::string>("input_topic", "/plato2/joint_states");
        this->declare_parameter<std::string>("output_topic", "/plato2/joint_states_smoothed");
        
        // Get parameters
        int window_size = this->get_parameter("moving_average_window").as_int();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // Clamp window size to at least 1
        window_size = std::max(1, window_size);
        moving_average_window_ = window_size;
        
        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            input_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&JointStateMovingAverageNode::joint_state_callback, this, std::placeholders::_1)
        );
        
        // Publisher for smoothed joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            output_topic,
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "=== Joint State Moving Average Node Started ===");
        RCLCPP_INFO(this->get_logger(), "  Input Topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output Topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Moving Average Window: %d", moving_average_window_);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Initialize buffers on first message
        if (position_buffers_.empty() && !msg->position.empty()) {
            position_buffers_.resize(msg->position.size());
            position_sums_.resize(msg->position.size(), 0.0);
        }
        if (velocity_buffers_.empty() && !msg->velocity.empty()) {
            velocity_buffers_.resize(msg->velocity.size());
            velocity_sums_.resize(msg->velocity.size(), 0.0);
        }
        if (effort_buffers_.empty() && !msg->effort.empty()) {
            effort_buffers_.resize(msg->effort.size());
            effort_sums_.resize(msg->effort.size(), 0.0);
        }
        
        // Update position buffers and sums
        for (size_t i = 0; i < msg->position.size(); ++i) {
            if (position_buffers_[i].size() >= moving_average_window_) {
                position_sums_[i] -= position_buffers_[i].front();
                position_buffers_[i].pop_front();
            }
            position_buffers_[i].push_back(msg->position[i]);
            position_sums_[i] += msg->position[i];
        }
        
        // Update velocity buffers and sums
        for (size_t i = 0; i < msg->velocity.size(); ++i) {
            if (velocity_buffers_[i].size() >= moving_average_window_) {
                velocity_sums_[i] -= velocity_buffers_[i].front();
                velocity_buffers_[i].pop_front();
            }
            velocity_buffers_[i].push_back(msg->velocity[i]);
            velocity_sums_[i] += msg->velocity[i];
        }
        
        // Update effort buffers and sums
        for (size_t i = 0; i < msg->effort.size(); ++i) {
            if (effort_buffers_[i].size() >= moving_average_window_) {
                effort_sums_[i] -= effort_buffers_[i].front();
                effort_buffers_[i].pop_front();
            }
            effort_buffers_[i].push_back(msg->effort[i]);
            effort_sums_[i] += msg->effort[i];
        }
        
        // Create output message with smoothed values
        auto output = std::make_shared<sensor_msgs::msg::JointState>();
        output->header = msg->header;
        output->name = msg->name;
        
        // Compute and set smoothed position values
        output->position.resize(msg->position.size());
        for (size_t i = 0; i < msg->position.size(); ++i) {
            output->position[i] = position_sums_[i] / position_buffers_[i].size();
        }
        
        // Compute and set smoothed velocity values
        output->velocity.resize(msg->velocity.size());
        for (size_t i = 0; i < msg->velocity.size(); ++i) {
            output->velocity[i] = velocity_sums_[i] / velocity_buffers_[i].size();
        }
        
        // Compute and set smoothed effort values
        output->effort.resize(msg->effort.size());
        for (size_t i = 0; i < msg->effort.size(); ++i) {
            output->effort[i] = effort_sums_[i] / effort_buffers_[i].size();
        }
        
        joint_state_pub_->publish(*output);
    }
    
    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // Moving average state
    std::vector<std::deque<double>> position_buffers_;
    std::vector<double> position_sums_;
    std::vector<std::deque<double>> velocity_buffers_;
    std::vector<double> velocity_sums_;
    std::vector<std::deque<double>> effort_buffers_;
    std::vector<double> effort_sums_;
    int moving_average_window_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateMovingAverageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
