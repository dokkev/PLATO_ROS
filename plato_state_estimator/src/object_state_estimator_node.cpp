#include "plato_state_estimator/object_state_estimator_node.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <cstdint>

namespace plato_state_estimator {

ObjectStateEstimatorNode::ObjectStateEstimatorNode()
    : Node("object_state_estimator")
    , update_rate_(100.0)
{
    // Load configuration from ROS parameters
    auto config = loadConfigFromParameters();

    // Create core estimator
    estimator_ = std::make_unique<ObjectStateEstimator>(config);

    // Create subscribers for tactile sensors
    tactile0_sub_ = this->create_subscription<sdr_grasp_msgs::msg::Tactile>(
        "/tactile_0/tactile_states", 10,
        std::bind(&ObjectStateEstimatorNode::tactile0Callback, this, std::placeholders::_1));

    tactile1_sub_ = this->create_subscription<sdr_grasp_msgs::msg::Tactile>(
        "/tactile_1/tactile_states", 10,
        std::bind(&ObjectStateEstimatorNode::tactile1Callback, this, std::placeholders::_1));

    // Create publishers
    minimal_force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/object_state/minimal_force", 10);

    estimated_wrench_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        "/object_state/estimated_wrench", 10);

    measured_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/object_state/measured_force", 10);

    // Create update timer
    auto update_period = std::chrono::duration<double>(1.0 / update_rate_);
    update_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
        std::bind(&ObjectStateEstimatorNode::updateTimerCallback, this));

    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Object State Estimator Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "  Translational slip threshold: %.2f mm",
                config.translational_slip_threshold);
    RCLCPP_INFO(this->get_logger(), "  Rotational slip threshold: %.3f rad",
                config.rotational_slip_threshold);
    RCLCPP_INFO(this->get_logger(), "  Max force limit: %.1f N", config.max_force_limit);
}

ObjectStateEstimatorConfig ObjectStateEstimatorNode::loadConfigFromParameters() {
    ObjectStateEstimatorConfig config;

    // Declare and get parameters
    this->declare_parameter<double>("E_star", config.E_star);
    this->declare_parameter<double>("G_star", config.G_star);
    this->declare_parameter<double>("C_n", config.C_n);
    this->declare_parameter<double>("lambda_n", config.lambda_n);
    this->declare_parameter<int>("n", config.n);
    this->declare_parameter<double>("translational_slip_threshold", config.translational_slip_threshold);
    this->declare_parameter<double>("rotational_slip_threshold", config.rotational_slip_threshold);
    this->declare_parameter<double>("update_rate", update_rate_);
    this->declare_parameter<double>("min_contact_force", config.min_contact_force);
    this->declare_parameter<double>("max_force_limit", config.max_force_limit);
    this->declare_parameter<double>("pid_tx_p", config.pid_tx_p);
    this->declare_parameter<double>("pid_tx_i", config.pid_tx_i);
    this->declare_parameter<double>("pid_tx_d", config.pid_tx_d);
    this->declare_parameter<double>("pid_theta_p", config.pid_theta_p);
    this->declare_parameter<double>("pid_theta_i", config.pid_theta_i);
    this->declare_parameter<double>("pid_theta_d", config.pid_theta_d);

    // Get parameter values
    config.E_star = this->get_parameter("E_star").as_double();
    config.G_star = this->get_parameter("G_star").as_double();
    config.C_n = this->get_parameter("C_n").as_double();
    config.lambda_n = this->get_parameter("lambda_n").as_double();
    config.n = this->get_parameter("n").as_int();
    config.translational_slip_threshold = this->get_parameter("translational_slip_threshold").as_double();
    config.rotational_slip_threshold = this->get_parameter("rotational_slip_threshold").as_double();
    update_rate_ = this->get_parameter("update_rate").as_double();
    config.min_contact_force = this->get_parameter("min_contact_force").as_double();
    config.max_force_limit = this->get_parameter("max_force_limit").as_double();
    config.pid_tx_p = this->get_parameter("pid_tx_p").as_double();
    config.pid_tx_i = this->get_parameter("pid_tx_i").as_double();
    config.pid_tx_d = this->get_parameter("pid_tx_d").as_double();
    config.pid_theta_p = this->get_parameter("pid_theta_p").as_double();
    config.pid_theta_i = this->get_parameter("pid_theta_i").as_double();
    config.pid_theta_d = this->get_parameter("pid_theta_d").as_double();

    return config;
}

void ObjectStateEstimatorNode::tactile0Callback(
    const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg)
{
    tactile0_msg_ = msg;
}

void ObjectStateEstimatorNode::tactile1Callback(
    const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg)
{
    tactile1_msg_ = msg;
}

void ObjectStateEstimatorNode::updateTimerCallback() {
    // Check if we have data from both sensors
    if (!tactile0_msg_ || !tactile1_msg_) {
        return;
    }

    // Calculate dt
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    // Validate dt
    if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / update_rate_;  // Use nominal period
    }

    // Convert ROS messages to core data structures
    TactileData tactile0 = convertTactileMsg(tactile0_msg_);
    TactileData tactile1 = convertTactileMsg(tactile1_msg_);

    // Track measured force using the best available tactile source
    double measured_force = 0.0;
    int8_t measured_source = -1;  // -1 = none, 0 = tactile0, 1 = tactile1
    bool sensor0_contact = tactile0.contact_state >= TactileData::FEW_CONTACTS;
    bool sensor1_contact = tactile1.contact_state >= TactileData::FEW_CONTACTS;
    if (sensor0_contact && sensor1_contact) {
        if (tactile0.force_z >= tactile1.force_z) {
            measured_force = tactile0.force_z;
            measured_source = 0;
        } else {
            measured_force = tactile1.force_z;
            measured_source = 1;
        }
    } else if (sensor0_contact) {
        measured_force = tactile0.force_z;
        measured_source = 0;
    } else if (sensor1_contact) {
        measured_force = tactile1.force_z;
        measured_source = 1;
    }

    // Update estimator
    ObjectStateEstimatorOutput output = estimator_->update(tactile0, tactile1, dt);

    // Publish results if we have valid data
    if (output.has_valid_data) {
        // Publish minimal force
        auto force_msg = std_msgs::msg::Float32();
        force_msg.data = static_cast<float>(output.minimal_force);
        minimal_force_pub_->publish(force_msg);

        // Publish estimated wrench
        auto wrench_msg = geometry_msgs::msg::Wrench();
        wrench_msg.force.x = output.force_x;
        wrench_msg.force.y = output.force_y;
        wrench_msg.force.z = output.minimal_force;  // Normal force
        wrench_msg.torque.z = output.moment_z;
        estimated_wrench_pub_->publish(wrench_msg);

        // Publish measured force (normal) used for feedback
        auto measured_msg = std_msgs::msg::Float64();
        measured_msg.data = measured_force;
        measured_force_pub_->publish(measured_msg);

        // Log slip state changes (optional, for debugging)
        static SlipState last_state = SlipState::NO_CONTACT;
        if (output.slip_state != last_state) {
            const char* state_str = "UNKNOWN";
            switch (output.slip_state) {
                case SlipState::NO_CONTACT: state_str = "NO_CONTACT"; break;
                case SlipState::PARTIAL_CONTACT: state_str = "PARTIAL_CONTACT"; break;
                case SlipState::STABLE_GRASP: state_str = "STABLE_GRASP"; break;
                case SlipState::TRANSLATIONAL_SLIP: state_str = "TRANSLATIONAL_SLIP"; break;
                case SlipState::ROTATIONAL_SLIP: state_str = "ROTATIONAL_SLIP"; break;
                case SlipState::COMBINED_SLIP: state_str = "COMBINED_SLIP"; break;
            }
            RCLCPP_INFO(this->get_logger(), "Slip state changed to: %s", state_str);
            last_state = output.slip_state;
        }
    }
}

TactileData ObjectStateEstimatorNode::convertTactileMsg(
    const std::shared_ptr<sdr_grasp_msgs::msg::Tactile>& msg)
{
    TactileData data;

    data.contact_state = msg->contact_state;

    // Shear displacement
    data.shear_x = msg->shear_displacement.x;
    data.shear_y = msg->shear_displacement.y;
    data.shear_theta = msg->shear_displacement.theta;

    // Forces
    // Only normal force is available; tangential components are assumed zero.
    data.force_x = 0.0;
    data.force_y = 0.0;
    data.force_z = msg->force.z;

    // Convert ROS2 timestamp to seconds
    data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    return data;
}

}  // namespace plato_state_estimator

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<plato_state_estimator::ObjectStateEstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
