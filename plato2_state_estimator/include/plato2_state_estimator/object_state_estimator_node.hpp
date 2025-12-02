#ifndef PLATO2_STATE_ESTIMATOR_OBJECT_STATE_ESTIMATOR_NODE_HPP
#define PLATO2_STATE_ESTIMATOR_OBJECT_STATE_ESTIMATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>

#include "plato2_state_estimator/object_state_estimator.hpp"

#include <memory>
#include <string>

// Forward declare tactile message types (from nari_touch)
namespace sdr_grasp_msgs {
namespace msg {
struct Tactile;
}
}

namespace plato2_state_estimator {

/**
 * @brief ROS2 node wrapper for ObjectStateEstimator
 *
 * This node handles ROS communication and delegates core logic
 * to the ObjectStateEstimator class.
 */
class ObjectStateEstimatorNode : public rclcpp::Node {
public:
    ObjectStateEstimatorNode();
    ~ObjectStateEstimatorNode() = default;

private:
    // ROS callbacks
    void tactile0Callback(const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg);
    void tactile1Callback(const std::shared_ptr<sdr_grasp_msgs::msg::Tactile> msg);
    void updateTimerCallback();

    // Helper functions
    TactileData convertTactileMsg(const std::shared_ptr<sdr_grasp_msgs::msg::Tactile>& msg);
    ObjectStateEstimatorConfig loadConfigFromParameters();

    // ROS subscribers
    rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tactile0_sub_;
    rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tactile1_sub_;

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr minimal_force_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr estimated_wrench_pub_;

    // Timer for periodic updates
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Core estimator (ROS-independent)
    std::unique_ptr<ObjectStateEstimator> estimator_;

    // Sensor data storage
    std::shared_ptr<sdr_grasp_msgs::msg::Tactile> tactile0_msg_;
    std::shared_ptr<sdr_grasp_msgs::msg::Tactile> tactile1_msg_;

    // Time tracking
    rclcpp::Time last_update_time_;
    double update_rate_;  // Hz
};

}  // namespace plato2_state_estimator

#endif  // PLATO2_STATE_ESTIMATOR_OBJECT_STATE_ESTIMATOR_NODE_HPP
