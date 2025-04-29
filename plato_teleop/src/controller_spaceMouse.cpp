#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

class OptimoSpaceMouseController : public rclcpp::Node
{
public:
    OptimoSpaceMouseController() : Node("optimo_space_mouse_controller")
    {

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "spaceMouseMotion", 10,
            std::bind(&OptimoSpaceMouseController::space_mouse_motion_callback, this, std::placeholders::_1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/optimo/servo/twist_cmd", 10);
    }

private:
    // Function to publish twist for translation control
    void publish_translation_twist(double x, double y, double z)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = "world"; // Adjust if necessary
        twist_msg->twist.linear.x = x * translation_scale_;
        twist_msg->twist.linear.y = y * translation_scale_;
        twist_msg->twist.linear.z = z * translation_scale_;
        twist_pub_->publish(std::move(twist_msg));
    }

    // Function to publish twist for rotation control
    void publish_rotation_twist(double roll, double pitch, double yaw)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = "world"; // Adjust if necessary
        twist_msg->twist.angular.x = roll * rotation_scale_;
        twist_msg->twist.angular.y = pitch * rotation_scale_;
        twist_msg->twist.angular.z = yaw * rotation_scale_;
        twist_pub_->publish(std::move(twist_msg));
    }
    /* This function decides whether to translate or rotate */
    std::string decide_movement(const sensor_msgs::msg::Joy::SharedPtr &msg)
    {
        // Calculate magnitudes of translation and rotation
        double translation_magnitude = calculate_translation_magnitude(msg);
        double rotation_magnitude = calculate_rotation_magnitude(msg);

        // Determine which set of actions to take based on thresholds and magnitudes
        if (translation_magnitude > translation_threshold_ && translation_magnitude > rotation_magnitude)
        {
            return "translation";
        }
        else if (rotation_magnitude > rotation_threshold_)
        {
            return "rotation";
        }
        return "none"; // Neither exceeds the threshold
    }

    void space_mouse_motion_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::string action = decide_movement(msg);
    
        if (action == "none")
        {
            // RCLCPP_INFO(this->get_logger(), "[none] No significant motion detected.");
            publish_translation_twist(0, 0, 0);
            publish_rotation_twist(0, 0, 0);
            return;
        }
        else if (action == "translation")
        {
            publish_translation_twist(msg->axes[0], msg->axes[1], msg->axes[2]);
            // RCLCPP_INFO(this->get_logger(),
            //     "[translation] x: %.3f, y: %.3f, z: %.3f",
            //     msg->axes[0], msg->axes[1], msg->axes[2]);
        }
        else if (action == "rotation")
        {
            publish_rotation_twist(msg->axes[3], msg->axes[4], msg->axes[5]);
            // RCLCPP_INFO(this->get_logger(),
            //     "[rotation] rx: %.3f, ry: %.3f, rz: %.3f",
            //     msg->axes[3], msg->axes[4], msg->axes[5]);
        }
    }
    
    // Function to calculate the magnitude of translation
    double calculate_translation_magnitude(const sensor_msgs::msg::Joy::SharedPtr &msg)
    {
        return std::sqrt(std::pow(msg->axes[0], 2) + std::pow(msg->axes[1], 2) + std::pow(msg->axes[2], 2));
    }

    // Function to calculate the magnitude of rotation
    double calculate_rotation_magnitude(const sensor_msgs::msg::Joy::SharedPtr &msg)
    {
        return std::sqrt(std::pow(msg->axes[3], 2) + std::pow(msg->axes[4], 2) + std::pow(msg->axes[5], 2));
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    const double translation_scale_ = 1; // Adjust to control sensitivity
    const double rotation_scale_ = 0.3;    // Adjust to control sensitivity

    const double translation_threshold_ = 0.3; // Set appropriate translation threshold
    const double rotation_threshold_ = 0.3;    // Set appropriate rotation threshold
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OptimoSpaceMouseController>();
    // node->initialize_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
