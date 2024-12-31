#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class OptimoSpaceMouseController : public rclcpp::Node
{
public:
    OptimoSpaceMouseController() : Node("optimo_space_mouse_controller")
    {
        this->declare_parameter<std::string>("planning_group", "optimo_arm");
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "spaceMouseMotion", 10,
            std::bind(&OptimoSpaceMouseController::space_mouse_motion_callback, this, std::placeholders::_1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    }

    void initialize_move_group()
    {
        auto planning_group = this->get_parameter("planning_group").as_string();
        move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), planning_group);
    }

private:
    // Function to publish twist for translation control
    void publish_translation_twist(double x, double y, double z)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = "link0"; // Adjust if necessary
        twist_msg->twist.linear.x = x * translation_scale_;
        twist_msg->twist.linear.y = y * translation_scale_;
        twist_msg->twist.linear.z = z * translation_scale_;
        twist_pub_->publish(std::move(twist_msg));
    }

    // Function to publish twist for rotation control
    void publish_rotation_twist(double roll, double yaw, double pitch)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = "link0"; // Adjust if necessary
        twist_msg->twist.angular.x = - roll * rotation_scale_;
        twist_msg->twist.angular.y = pitch * rotation_scale_;
        twist_msg->twist.angular.z = - yaw * rotation_scale_;
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
            RCLCPP_INFO(this->get_logger(), "No significant motion detected.");
            publish_translation_twist(0, 0, 0);
            publish_rotation_twist(0, 0, 0);
            return; // Exit the callback if no significant motion is detected
        }
        else if (action == "translation")
        {
            // Apply translation logic here
            RCLCPP_INFO(this->get_logger(), "Performing translation.");
            RCLCPP_INFO(this->get_logger(), "x dir: [%f], y dir: [%f], z dir[%f]",
                        msg->axes[0],
                        msg->axes[1],
                        msg->axes[2]);
            publish_translation_twist(msg->axes[0], msg->axes[1], msg->axes[2]);
        }
        else if (action == "rotation")
        {
            // Apply rotation logic here
            RCLCPP_INFO(this->get_logger(), "Performing rotation.");
            RCLCPP_INFO(this->get_logger(), "Roll: [%f], Yaw: [%f], Pitch[%f]",
                        msg->axes[5],
                        msg->axes[4],
                        msg->axes[3]);
            publish_rotation_twist(msg->axes[5], msg->axes[4], msg->axes[3]);
        }

        /*         // Get the current pose
                geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose();

                // Initialize target_pose based on current_pose
                geometry_msgs::msg::Pose target_pose = current_pose.pose;

                // Apply translation adjustments directly to target_pose based on SpaceMouse input
                target_pose.position.x += msg->axes[0] * translation_scale_;
                target_pose.position.y += msg->axes[1] * translation_scale_;
                target_pose.position.z += msg->axes[2] * translation_scale_;

                // Obtain current orientation in RPY
                tf2::Quaternion current_orientation;
                tf2::fromMsg(current_pose.pose.orientation, current_orientation);
                double current_roll, current_pitch, current_yaw;
                tf2::Matrix3x3(current_orientation).getRPY(current_roll, current_pitch, current_yaw);

                // Adjust orientation based on the SpaceMouse input and update target_pose
                current_roll += msg->axes[3] * rotation_scale_;
                current_pitch += msg->axes[4] * rotation_scale_;
                current_yaw += msg->axes[5] * rotation_scale_;

                // Convert the updated RPY back to a quaternion
                tf2::Quaternion new_orientation;
                new_orientation.setRPY(current_roll, current_pitch, current_yaw);
                target_pose.orientation = tf2::toMsg(new_orientation.normalize());

                // Compute the Cartesian path using waypoints from current_pose to target_pose
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(current_pose.pose); // Start with current pose
                waypoints.push_back(target_pose);       // Target pose based on SpaceMouse input

                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0; // Threshold for discontinuities in the path
                const double eef_step = 0.01;      // Resolution of the path
                double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

                // Execute the trajectory if a complete path was computed
                if (fraction == 1.0)
                {
                    move_group_interface_->execute(trajectory);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to compute a complete Cartesian path");
                } */
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

    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    const double translation_scale_ = 1; // Adjust to control sensitivity
    const double rotation_scale_ = 1;    // Adjust to control sensitivity

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
