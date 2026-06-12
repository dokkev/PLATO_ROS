#ifndef PLATO_STATE_ESTIMATOR_OBJECT_PRIOR_ESTIMATOR_NODE_HPP
#define PLATO_STATE_ESTIMATOR_OBJECT_PRIOR_ESTIMATOR_NODE_HPP

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mppi_core/contact/contact_kinematics.hpp>
#include <mppi_core/object/object_prior_estimator.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdr_grasp_msgs/msg/tactile.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace plato_state_estimator
{

// ROS wrapper for the tactile object-prior particle estimator in mppi_core.
class ObjectPriorEstimatorNode : public rclcpp::Node
{
public:
  ObjectPriorEstimatorNode();
  ~ObjectPriorEstimatorNode() override = default;

private:
  using TactileMsg = sdr_grasp_msgs::msg::Tactile;

  void jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg);
  void tactileCallback(std::size_t index, TactileMsg::SharedPtr msg);
  void updateTimerCallback();

  bool loadRobotModel();
  bool buildTactileContexts();
  bool buildJointConfiguration(Eigen::VectorXd * q);

  mppi_core::ObjectPrior loadObjectPriorFromParameters();
  mppi_core::ObjectPriorEstimatorConfig loadEstimatorConfigFromParameters();
  mppi_core::TactileState convertTactileMsg(
    const TactileMsg & msg,
    std::size_t index);

  std::vector<mppi_core::HemisphereGeometry> makeHemisphereGeometry();
  void publishEstimate(
    const mppi_core::ObjectBeliefInitializationResult & result,
    const std::vector<mppi_core::TactileState,
    Eigen::aligned_allocator<mppi_core::TactileState>> & tactile_sensors);
  void publishMarkers(
    const mppi_core::ObjectBeliefInitializationResult & result,
    const std::vector<mppi_core::TactileState,
    Eigen::aligned_allocator<mppi_core::TactileState>> & tactile_sensors,
    const rclcpp::Time & stamp);
  void logCorrection(
    const mppi_core::ObjectBeliefInitializationResult & result,
    std::size_t active_sensor_count,
    std::size_t active_hemisphere_count,
    const rclcpp::Time & stamp);
  void logCloseSensorGapRejection(
    std::size_t active_sensor_count,
    std::size_t active_hemisphere_count);

  std::string robot_description_;
  std::string robot_description_path_;
  std::string joint_state_topic_;
  std::string object_pose_topic_;
  std::string particle_pose_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string world_frame_{"base_link"};

  std::vector<std::string> tactile_topics_;
  std::vector<std::string> tactile_frame_names_;
  std::vector<double> tactile_normal_axis_signs_;

  double update_rate_{100.0};
  double cop_scale_m_{1.0e-3};
  double hemisphere_radius_m_{0.003};
  double hemisphere_contact_threshold_n_{0.1};
  bool publish_markers_{true};
  bool publish_prior_marker_when_no_belief_{true};
  bool hide_corrected_markers_without_contact_{true};
  bool log_corrections_{true};
  double correction_log_interval_s_{0.5};
  bool has_correction_log_time_{false};
  rclcpp::Time last_correction_log_time_;
  std::size_t last_logged_correction_update_count_{0};

  pinocchio::Model model_;
  std::vector<std::unique_ptr<pinocchio::Data>> kinematics_data_;
  std::vector<mppi_core::PinocchioContactKinematicsContext> kinematics_;
  std::vector<mppi_core::TactileSensorContext> tactile_contexts_;
  bool model_loaded_{false};

  mppi_core::ObjectPriorEstimator estimator_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<rclcpp::Subscription<TactileMsg>::SharedPtr> tactile_subs_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::vector<TactileMsg::SharedPtr> latest_tactile_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_poses_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace plato_state_estimator

#endif  // PLATO_STATE_ESTIMATOR_OBJECT_PRIOR_ESTIMATOR_NODE_HPP
