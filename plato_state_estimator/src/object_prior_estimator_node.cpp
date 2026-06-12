#include "plato_state_estimator/object_prior_estimator_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <mppi_core/object/object_contact_belief.hpp>
#include <mppi_core/object/object_prior.hpp>
#include <mppi_core/tactile/nari_touch_adapter.hpp>
#include <mppi_core/tactile/nari_touch_state.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace plato_state_estimator
{
namespace
{

std::vector<std::string> DefaultTactileTopics()
{
  return {"/tactile_0/tactile_states", "/tactile_1/tactile_states"};
}

std::vector<std::string> DefaultTactileFrameNames()
{
  return {"thumb_distal_tactile", "index_distal_tactile"};
}

std::vector<double> DefaultNormalAxisSigns()
{
  return {1.0, 1.0};
}

double NormalizedAxisSign(const double sign)
{
  return std::isfinite(sign) && sign < 0.0 ? -1.0 : 1.0;
}

Eigen::Vector3d Vector3FromParam(
  const std::vector<double> & values,
  const Eigen::Vector3d & fallback)
{
  if (values.size() != 3U) {
    return fallback;
  }
  return Eigen::Vector3d{values[0], values[1], values[2]};
}

Eigen::Matrix3d RpyToRotation(const Eigen::Vector3d & rpy_rad)
{
  const Eigen::AngleAxisd roll(rpy_rad.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy_rad.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy_rad.z(), Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

mppi_core::ObjectGeometryType ParseObjectGeometryType(const std::string & value)
{
  std::string lower = value;
  std::transform(
    lower.begin(), lower.end(), lower.begin(),
    [](const unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (lower == "box") {
    return mppi_core::ObjectGeometryType::kBox;
  }
  if (lower == "sphere") {
    return mppi_core::ObjectGeometryType::kSphere;
  }
  if (lower == "cylinder") {
    return mppi_core::ObjectGeometryType::kCylinder;
  }
  if (lower == "mesh") {
    return mppi_core::ObjectGeometryType::kMesh;
  }
  if (lower == "urdf") {
    return mppi_core::ObjectGeometryType::kUrdf;
  }
  return mppi_core::ObjectGeometryType::kUnknown;
}

mppi_core::NariTouchContactState ConvertContactState(const int state)
{
  switch (state) {
    case sdr_grasp_msgs::msg::Tactile::ENOUGH_CONTACTS:
      return mppi_core::NariTouchContactState::kEnoughContacts;
    case sdr_grasp_msgs::msg::Tactile::FEW_CONTACTS:
      return mppi_core::NariTouchContactState::kFewContacts;
    default:
      return mppi_core::NariTouchContactState::kNoContact;
  }
}

double StampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();

  Eigen::Quaterniond q(pose.linear());
  q.normalize();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

std_msgs::msg::ColorRGBA Color(
  const float r,
  const float g,
  const float b,
  const float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

geometry_msgs::msg::Point PointMsg(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

visualization_msgs::msg::Marker BaseMarker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  const int id,
  const int type)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  marker.pose.orientation.w = 1.0;
  return marker;
}

visualization_msgs::msg::Marker DeleteMarker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  const int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

void ApplyObjectGeometryScale(
  const mppi_core::ObjectGeometryHandle & geometry,
  visualization_msgs::msg::Marker * marker)
{
  if (marker == nullptr) {
    return;
  }
  if (geometry.type == mppi_core::ObjectGeometryType::kSphere) {
    marker->type = visualization_msgs::msg::Marker::SPHERE;
    const double diameter = 2.0 * std::max(0.0, geometry.primitive_size_m.x());
    marker->scale.x = diameter;
    marker->scale.y = diameter;
    marker->scale.z = diameter;
  } else if (geometry.type == mppi_core::ObjectGeometryType::kCylinder) {
    marker->type = visualization_msgs::msg::Marker::CYLINDER;
    const double diameter = 2.0 * std::max(0.0, geometry.primitive_size_m.x());
    marker->scale.x = diameter;
    marker->scale.y = diameter;
    marker->scale.z = std::max(0.001, geometry.primitive_size_m.z());
  } else {
    marker->scale.x = std::max(0.001, geometry.primitive_size_m.x());
    marker->scale.y = std::max(0.001, geometry.primitive_size_m.y());
    marker->scale.z = std::max(0.001, geometry.primitive_size_m.z());
  }
}

void AddBoxOutlinePoints(
  const geometry_msgs::msg::Vector3 & scale,
  visualization_msgs::msg::Marker * marker)
{
  if (marker == nullptr) {
    return;
  }
  const double hx = 0.5 * std::max(0.001, scale.x);
  const double hy = 0.5 * std::max(0.001, scale.y);
  const double hz = 0.5 * std::max(0.001, scale.z);
  const std::array<Eigen::Vector3d, 8> corners{
    Eigen::Vector3d{-hx, -hy, -hz},
    Eigen::Vector3d{ hx, -hy, -hz},
    Eigen::Vector3d{ hx,  hy, -hz},
    Eigen::Vector3d{-hx,  hy, -hz},
    Eigen::Vector3d{-hx, -hy,  hz},
    Eigen::Vector3d{ hx, -hy,  hz},
    Eigen::Vector3d{ hx,  hy,  hz},
    Eigen::Vector3d{-hx,  hy,  hz},
  };
  constexpr std::array<std::pair<int, int>, 12> edges{
    std::pair<int, int>{0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };
  marker->points.clear();
  marker->points.reserve(edges.size() * 2U);
  for (const auto & edge : edges) {
    marker->points.push_back(PointMsg(corners[edge.first]));
    marker->points.push_back(PointMsg(corners[edge.second]));
  }
}

}  // namespace

ObjectPriorEstimatorNode::ObjectPriorEstimatorNode()
: Node("object_prior_estimator")
{
  update_rate_ = this->declare_parameter<double>("update_rate", update_rate_);
  world_frame_ = this->declare_parameter<std::string>("world_frame", world_frame_);
  robot_description_ = this->declare_parameter<std::string>("robot_description", "");
  robot_description_path_ =
    this->declare_parameter<std::string>("robot_description_path", "");

  joint_state_topic_ =
    this->declare_parameter<std::string>("joint_state_topic", "/plato2/joint_states");
  tactile_topics_ =
    this->declare_parameter<std::vector<std::string>>("tactile_topics", DefaultTactileTopics());
  tactile_frame_names_ = this->declare_parameter<std::vector<std::string>>(
    "tactile_frame_names", DefaultTactileFrameNames());
  tactile_normal_axis_signs_ = this->declare_parameter<std::vector<double>>(
    "tactile_normal_axis_signs", DefaultNormalAxisSigns());

  object_pose_topic_ =
    this->declare_parameter<std::string>("object_pose_topic", "~/object_pose");
  particle_pose_topic_ =
    this->declare_parameter<std::string>("particle_pose_topic", "~/particle_poses");
  status_topic_ = this->declare_parameter<std::string>("status_topic", "~/status");
  marker_topic_ = this->declare_parameter<std::string>("marker_topic", "~/markers");
  publish_markers_ = this->declare_parameter<bool>("publish_markers", publish_markers_);
  publish_prior_marker_when_no_belief_ = this->declare_parameter<bool>(
    "publish_prior_marker_when_no_belief",
    publish_prior_marker_when_no_belief_);
  hide_corrected_markers_without_contact_ = this->declare_parameter<bool>(
    "hide_corrected_markers_without_contact",
    hide_corrected_markers_without_contact_);
  log_corrections_ = this->declare_parameter<bool>("log_corrections", log_corrections_);
  correction_log_interval_s_ = this->declare_parameter<double>(
    "correction_log_interval_s",
    correction_log_interval_s_);

  cop_scale_m_ = this->declare_parameter<double>("cop_scale_m", cop_scale_m_);
  hemisphere_radius_m_ =
    this->declare_parameter<double>("hemisphere_radius_m", hemisphere_radius_m_);
  hemisphere_contact_threshold_n_ = this->declare_parameter<double>(
    "hemisphere_contact_threshold_n", hemisphere_contact_threshold_n_);

  if (tactile_topics_.empty()) {
    tactile_topics_ = DefaultTactileTopics();
  }
  while (tactile_frame_names_.size() < tactile_topics_.size()) {
    const auto defaults = DefaultTactileFrameNames();
    const std::size_t index = tactile_frame_names_.size();
    tactile_frame_names_.push_back(
      index < defaults.size() ? defaults[index] : "tactile_sensor");
  }
  while (tactile_normal_axis_signs_.size() < tactile_topics_.size()) {
    tactile_normal_axis_signs_.push_back(1.0);
  }

  const auto prior = loadObjectPriorFromParameters();
  const auto estimator_config = loadEstimatorConfigFromParameters();
  if (!estimator_.Configure(prior, estimator_config)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Object prior estimator is not configured; check object prior parameters");
  }

  model_loaded_ = loadRobotModel() && buildTactileContexts();

  object_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(object_pose_topic_, 10);
  particle_poses_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(particle_pose_topic_, 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ObjectPriorEstimatorNode::jointStateCallback, this, std::placeholders::_1));

  latest_tactile_.resize(tactile_topics_.size());
  tactile_subs_.reserve(tactile_topics_.size());
  for (std::size_t i = 0; i < tactile_topics_.size(); ++i) {
    tactile_subs_.push_back(
      this->create_subscription<TactileMsg>(
        tactile_topics_[i],
        rclcpp::SensorDataQoS(),
        [this, i](TactileMsg::SharedPtr msg) {this->tactileCallback(i, std::move(msg));}));
  }

  const double safe_update_rate = update_rate_ > 0.0 ? update_rate_ : 100.0;
  const auto update_period = std::chrono::duration<double>(1.0 / safe_update_rate);
  update_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
    std::bind(&ObjectPriorEstimatorNode::updateTimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Object prior estimator initialized: rate=%.1fHz model_loaded=%s tactile_sensors=%zu",
    safe_update_rate,
    model_loaded_ ? "true" : "false",
    tactile_topics_.size());
}

mppi_core::ObjectPrior ObjectPriorEstimatorNode::loadObjectPriorFromParameters()
{
  mppi_core::ObjectPrior prior;
  prior.name = this->declare_parameter<std::string>("object_name", "jenga_block");
  prior.geometry.name =
    this->declare_parameter<std::string>("object_geometry_name", prior.name);
  prior.geometry.uri = this->declare_parameter<std::string>(
    "object_geometry_uri", "package://mppi_core/object/jenga_block.urdf");
  prior.geometry.type = ParseObjectGeometryType(
    this->declare_parameter<std::string>("object_geometry_type", "box"));
  prior.geometry.primitive_size_m = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "object_primitive_size_m", std::vector<double>{0.15, 0.05, 0.03}),
    Eigen::Vector3d{0.15, 0.05, 0.03});

  const Eigen::Vector3d xyz = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "object_initial_xyz", std::vector<double>{0.26, 0.04, -0.05}),
    Eigen::Vector3d{0.26, 0.04, -0.05});
  const Eigen::Vector3d rpy = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "object_initial_rpy", std::vector<double>{0.0, 0.0, 0.0}),
    Eigen::Vector3d::Zero());
  prior.initial_pose_world = Eigen::Isometry3d::Identity();
  prior.initial_pose_world.translation() = xyz;
  prior.initial_pose_world.linear() = RpyToRotation(rpy);

  prior.position_std_m = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "object_position_std_m", std::vector<double>{0.02, 0.01, 0.02}),
    Eigen::Vector3d{0.02, 0.01, 0.02});
  prior.rpy_std_rad = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "object_rpy_std_rad", std::vector<double>{0.1, 0.1, 0.2}),
    Eigen::Vector3d{0.1, 0.1, 0.2});

  prior.geometry.valid = true;
  prior.valid = true;
  if (!mppi_core::IsValidObjectPrior(prior)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid object prior parameters");
    return mppi_core::ObjectPrior{};
  }
  return prior;
}

mppi_core::ObjectPriorEstimatorConfig
ObjectPriorEstimatorNode::loadEstimatorConfigFromParameters()
{
  mppi_core::ObjectPriorEstimatorConfig config;
  config.reject_close_sensor_gap_as_fingertip_touch =
    this->declare_parameter<bool>(
    "reject_close_sensor_gap_as_fingertip_touch",
    config.reject_close_sensor_gap_as_fingertip_touch);
  config.sensor_gap_min_object_extent_scale = this->declare_parameter<double>(
    "sensor_gap_min_object_extent_scale",
    config.sensor_gap_min_object_extent_scale);
  config.sensor_gap_min_margin_m = this->declare_parameter<double>(
    "sensor_gap_min_margin_m",
    config.sensor_gap_min_margin_m);

  auto & belief = config.belief;
  const auto particle_count =
    this->declare_parameter<int64_t>("belief_particle_count", 32);
  const auto random_seed =
    this->declare_parameter<int64_t>("belief_random_seed", 17);
  const auto min_contact_count =
    this->declare_parameter<int64_t>("belief_min_contact_count", 1);
  belief.particle_count = static_cast<std::size_t>(
    std::max<int64_t>(0, particle_count));
  belief.random_seed = static_cast<std::uint32_t>(
    std::max<int64_t>(0, random_seed));
  belief.min_contact_count = static_cast<std::size_t>(
    std::max<int64_t>(0, min_contact_count));
  belief.fallback_position_sample_std_m = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "belief_fallback_position_sample_std_m", std::vector<double>{0.01, 0.01, 0.01}),
    belief.fallback_position_sample_std_m);
  belief.fallback_rpy_sample_std_rad = Vector3FromParam(
    this->declare_parameter<std::vector<double>>(
      "belief_fallback_rpy_sample_std_rad", std::vector<double>{0.05, 0.05, 0.05}),
    belief.fallback_rpy_sample_std_rad);
  belief.surface_distance_sigma_m =
    this->declare_parameter<double>("belief_surface_distance_sigma_m", 0.005);
  belief.normal_alignment_sigma =
    this->declare_parameter<double>("belief_normal_alignment_sigma", 0.25);
  belief.prior_position_sigma_m =
    this->declare_parameter<double>("belief_prior_position_sigma_m", 0.02);
  belief.prior_rotation_sigma_rad =
    this->declare_parameter<double>("belief_prior_rotation_sigma_rad", 0.35);
  belief.contact_force_threshold_n =
    this->declare_parameter<double>("belief_contact_force_threshold_n", 0.0);
  belief.contact_force_weight_scale_n =
    this->declare_parameter<double>("belief_contact_force_weight_scale_n", 1.0);
  belief.use_thumb_index_contact_width =
    this->declare_parameter<bool>("belief_use_thumb_index_contact_width", true);
  belief.contact_width_sigma_m =
    this->declare_parameter<double>("belief_contact_width_sigma_m", 0.005);
  belief.w_surface = this->declare_parameter<double>("belief_w_surface", 1.0);
  belief.w_normal = this->declare_parameter<double>("belief_w_normal", 0.5);
  belief.w_prior = this->declare_parameter<double>("belief_w_prior", 0.1);
  belief.w_contact_width =
    this->declare_parameter<double>("belief_w_contact_width", 0.5);
  return config;
}

bool ObjectPriorEstimatorNode::loadRobotModel()
{
  try {
    model_ = pinocchio::Model{};
    if (!robot_description_.empty()) {
      pinocchio::urdf::buildModelFromXML(robot_description_, model_);
    } else if (!robot_description_path_.empty()) {
      pinocchio::urdf::buildModel(robot_description_path_, model_);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "No robot_description or robot_description_path parameter was provided");
      return false;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load Pinocchio model: %s", error.what());
    return false;
  }

  if (model_.nq == 0 || model_.nv == 0) {
    RCLCPP_ERROR(this->get_logger(), "Loaded Pinocchio model has no joints");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded Pinocchio model: nq=%d nv=%d frames=%zu",
    model_.nq,
    model_.nv,
    model_.frames.size());
  return true;
}

bool ObjectPriorEstimatorNode::buildTactileContexts()
{
  if (tactile_frame_names_.size() < tactile_topics_.size() ||
    tactile_normal_axis_signs_.size() < tactile_topics_.size())
  {
    return false;
  }

  kinematics_.clear();
  kinematics_.resize(tactile_topics_.size());
  kinematics_data_.clear();
  kinematics_data_.reserve(tactile_topics_.size());
  tactile_contexts_.clear();
  tactile_contexts_.reserve(tactile_topics_.size());

  for (std::size_t i = 0; i < tactile_topics_.size(); ++i) {
    const auto frame_id = model_.getFrameId(tactile_frame_names_[i]);
    if (frame_id >= model_.frames.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Tactile frame '%s' was not found in the robot model",
        tactile_frame_names_[i].c_str());
      return false;
    }

    kinematics_data_.push_back(std::make_unique<pinocchio::Data>(model_));
    kinematics_[i].model = &model_;
    kinematics_[i].data = kinematics_data_.back().get();
    kinematics_[i].sensor_frame_id = frame_id;
    kinematics_[i].normal_axis_sign =
      NormalizedAxisSign(tactile_normal_axis_signs_[i]);

    mppi_core::TactileSensorContext context;
    context.sensor_index = static_cast<int>(i);
    context.kinematics = &kinematics_[i];
    context.hemispheres = makeHemisphereGeometry();
    tactile_contexts_.push_back(std::move(context));
  }

  return true;
}

std::vector<mppi_core::HemisphereGeometry>
ObjectPriorEstimatorNode::makeHemisphereGeometry()
{
  std::vector<mppi_core::HemisphereGeometry> geometry;
  const auto unit_positions = mppi_core::NariTouchUnitPositionsM();
  geometry.reserve(unit_positions.size());
  for (std::size_t i = 0; i < unit_positions.size(); ++i) {
    mppi_core::HemisphereGeometry hemisphere;
    hemisphere.hemisphere_index = i;
    hemisphere.center_sensor_m =
      Eigen::Vector3d{unit_positions[i].x(), unit_positions[i].y(), 0.0};
    hemisphere.normal_sensor = Eigen::Vector3d::UnitZ();
    hemisphere.radius_m = hemisphere_radius_m_;
    if (i > 0) {
      hemisphere.neighbors.push_back(i - 1U);
    }
    if (i + 1U < unit_positions.size()) {
      hemisphere.neighbors.push_back(i + 1U);
    }
    geometry.push_back(std::move(hemisphere));
  }
  return geometry;
}

void ObjectPriorEstimatorNode::jointStateCallback(
  sensor_msgs::msg::JointState::SharedPtr msg)
{
  latest_joint_state_ = std::move(msg);
}

void ObjectPriorEstimatorNode::tactileCallback(
  const std::size_t index,
  TactileMsg::SharedPtr msg)
{
  if (index >= latest_tactile_.size()) {
    return;
  }
  latest_tactile_[index] = std::move(msg);
}

bool ObjectPriorEstimatorNode::buildJointConfiguration(Eigen::VectorXd * q)
{
  if (q == nullptr || !latest_joint_state_ || !model_loaded_) {
    return false;
  }

  *q = pinocchio::neutral(model_);
  std::size_t matched_joint_count = 0;
  const auto & names = latest_joint_state_->name;
  const auto & positions = latest_joint_state_->position;
  const std::size_t count = std::min(names.size(), positions.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(positions[i]) || !model_.existJointName(names[i])) {
      continue;
    }
    const auto joint_id = model_.getJointId(names[i]);
    if (joint_id >= model_.joints.size() || model_.nqs[joint_id] != 1) {
      continue;
    }
    (*q)[model_.idx_qs[joint_id]] = positions[i];
    ++matched_joint_count;
  }

  if (matched_joint_count == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "JointState did not contain any joints from the Pinocchio model");
    return false;
  }
  return q->allFinite();
}

mppi_core::TactileState ObjectPriorEstimatorNode::convertTactileMsg(
  const TactileMsg & msg,
  const std::size_t index)
{
  mppi_core::NariTouchState nari;
  nari.valid = true;
  nari.stamp_sec = StampSeconds(msg.header.stamp);
  if (nari.stamp_sec <= 0.0) {
    nari.stamp_sec = this->now().seconds();
  }
  nari.sensor_index = static_cast<int>(index);
  nari.frame_name =
    !msg.frame_name.empty() ? msg.frame_name : tactile_frame_names_[index];
  nari.contact_state = ConvertContactState(msg.contact_state);
  nari.force_n = Eigen::Vector3d{msg.force.x, msg.force.y, msg.force.z};
  nari.shear_displacement_m =
    Eigen::Vector2d{msg.shear_displacement.x, msg.shear_displacement.y};
  nari.rotational_shear_rad = msg.shear_displacement.theta;

  const std::size_t unit_count = std::min(nari.units.size(), msg.units.size());
  for (std::size_t i = 0; i < unit_count; ++i) {
    const double normal_force_n = msg.units[i].normal_force;
    const bool force_contact =
      std::isfinite(normal_force_n) &&
      normal_force_n >= std::max(0.0, hemisphere_contact_threshold_n_);
    nari.units[i].contact = msg.units[i].contact || force_contact;
    nari.units[i].cop = Eigen::Vector2d{
      static_cast<double>(msg.units[i].cop.x) * cop_scale_m_,
      static_cast<double>(msg.units[i].cop.y) * cop_scale_m_};
    nari.units[i].normal_force_n =
      std::isfinite(normal_force_n) ? std::max(0.0, normal_force_n) : 0.0;
  }

  return mppi_core::ConvertNariTouchToTactileState(nari);
}

void ObjectPriorEstimatorNode::updateTimerCallback()
{
  if (!model_loaded_ || !estimator_.configured()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Object prior estimator waiting for valid model and object prior");
    return;
  }
  if (!latest_joint_state_) {
    return;
  }
  for (const auto & tactile_msg : latest_tactile_) {
    if (!tactile_msg) {
      return;
    }
  }

  Eigen::VectorXd q_meas;
  if (!buildJointConfiguration(&q_meas)) {
    return;
  }

  std::vector<mppi_core::TactileState,
    Eigen::aligned_allocator<mppi_core::TactileState>> tactile_sensors;
  tactile_sensors.reserve(latest_tactile_.size());
  for (std::size_t i = 0; i < latest_tactile_.size(); ++i) {
    tactile_sensors.push_back(convertTactileMsg(*latest_tactile_[i], i));
  }

  const auto result =
    estimator_.Update(q_meas, tactile_sensors, tactile_contexts_);
  publishEstimate(result, tactile_sensors);
}

void ObjectPriorEstimatorNode::publishEstimate(
  const mppi_core::ObjectBeliefInitializationResult & result,
  const std::vector<mppi_core::TactileState,
  Eigen::aligned_allocator<mppi_core::TactileState>> & tactile_sensors)
{
  const auto stamp = this->now();

  std::size_t active_sensor_count = 0;
  std::size_t active_hemisphere_count = 0;
  for (const auto & tactile : tactile_sensors) {
    const std::size_t active = tactile.activeHemisphereCount();
    active_hemisphere_count += active;
    if (active > 0U) {
      ++active_sensor_count;
    }
  }

  if (estimator_.has_belief()) {
    Eigen::Isometry3d representative_pose = Eigen::Isometry3d::Identity();
    if (estimator_.representativePose(&representative_pose)) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = stamp;
      pose_msg.header.frame_id = world_frame_;
      pose_msg.pose = ToPoseMsg(representative_pose);
      object_pose_pub_->publish(pose_msg);
    }

    geometry_msgs::msg::PoseArray particle_msg;
    particle_msg.header.stamp = stamp;
    particle_msg.header.frame_id = world_frame_;
    for (const auto & particle : estimator_.belief().particles) {
      if (!mppi_core::IsValidVirtualObjectState(particle)) {
        continue;
      }
      particle_msg.poses.push_back(ToPoseMsg(particle.pose_world));
    }
    particle_poses_pub_->publish(particle_msg);
  }

  const auto & status = estimator_.status();
  std_msgs::msg::String status_msg;
  std::ostringstream stream;
  stream << "configured=" << (status.configured ? "true" : "false")
         << " updated=" << (status.updated ? "true" : "false")
         << " result_valid=" << (result.valid ? "true" : "false")
         << " belief_valid=" << (status.belief_valid ? "true" : "false")
         << " close_sensor_gap_rejected="
         << (status.close_sensor_gap_rejected ? "true" : "false")
         << " update_count=" << status.update_count
         << " active_sensors=" << active_sensor_count
         << " active_hemispheres=" << active_hemisphere_count
         << " contact_count=" << status.contact_count
         << " particle_count=" << status.particle_count
         << " sensor_gap_m=" << status.sensor_gap_m
         << " min_sensor_gap_m=" << status.min_sensor_gap_m
         << " object_min_extent_m=" << status.object_min_extent_m
         << " best_particle_index=" << status.best_particle_index
         << " best_cost=" << status.best_cost
         << " best_surface_distance_m=" << status.best_surface_distance_m
         << " best_normal_alignment_error=" << status.best_normal_alignment_error;
  status_msg.data = stream.str();
  status_pub_->publish(status_msg);

  logCloseSensorGapRejection(active_sensor_count, active_hemisphere_count);
  logCorrection(result, active_sensor_count, active_hemisphere_count, stamp);
  publishMarkers(result, tactile_sensors, stamp);
}

void ObjectPriorEstimatorNode::logCloseSensorGapRejection(
  const std::size_t active_sensor_count,
  const std::size_t active_hemisphere_count)
{
  const auto & status = estimator_.status();
  if (!status.close_sensor_gap_rejected) {
    return;
  }
  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "object prior correction rejected: fingertips likely touching each other "
    "sensor_gap=%.4fm min_required_gap=%.4fm object_min_extent=%.4fm "
    "contacts=%zu active_sensors=%zu active_hemispheres=%zu",
    status.sensor_gap_m,
    status.min_sensor_gap_m,
    status.object_min_extent_m,
    status.contact_count,
    active_sensor_count,
    active_hemisphere_count);
}

void ObjectPriorEstimatorNode::logCorrection(
  const mppi_core::ObjectBeliefInitializationResult & result,
  const std::size_t active_sensor_count,
  const std::size_t active_hemisphere_count,
  const rclcpp::Time & stamp)
{
  const auto & status = estimator_.status();
  if (!log_corrections_ || !status.updated || !result.valid) {
    return;
  }
  if (last_logged_correction_update_count_ == status.update_count) {
    return;
  }
  if (
    correction_log_interval_s_ > 0.0 && has_correction_log_time_ &&
    (stamp - last_correction_log_time_).seconds() < correction_log_interval_s_)
  {
    return;
  }

  Eigen::Isometry3d corrected_pose = Eigen::Isometry3d::Identity();
  const bool has_pose = estimator_.representativePose(&corrected_pose);
  if (has_pose) {
    RCLCPP_INFO(
      this->get_logger(),
      "object prior corrected: update=%zu contacts=%zu active_sensors=%zu "
      "active_hemispheres=%zu particles=%zu best_cost=%.4f "
      "surface_dist=%.5f normal_err=%.5f pose_xyz=[%.4f %.4f %.4f]",
      status.update_count,
      status.contact_count,
      active_sensor_count,
      active_hemisphere_count,
      status.particle_count,
      status.best_cost,
      status.best_surface_distance_m,
      status.best_normal_alignment_error,
      corrected_pose.translation().x(),
      corrected_pose.translation().y(),
      corrected_pose.translation().z());
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "object prior corrected: update=%zu contacts=%zu active_sensors=%zu "
      "active_hemispheres=%zu particles=%zu best_cost=%.4f "
      "surface_dist=%.5f normal_err=%.5f pose_unavailable=true",
      status.update_count,
      status.contact_count,
      active_sensor_count,
      active_hemisphere_count,
      status.particle_count,
      status.best_cost,
      status.best_surface_distance_m,
      status.best_normal_alignment_error);
  }
  last_logged_correction_update_count_ = status.update_count;
  last_correction_log_time_ = stamp;
  has_correction_log_time_ = true;
}

void ObjectPriorEstimatorNode::publishMarkers(
  const mppi_core::ObjectBeliefInitializationResult & result,
  const std::vector<mppi_core::TactileState,
  Eigen::aligned_allocator<mppi_core::TactileState>> & tactile_sensors,
  const rclcpp::Time & stamp)
{
  if (!publish_markers_ || !marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  std::size_t active_sensor_count = 0;
  std::size_t active_hemisphere_count = 0;
  for (const auto & tactile : tactile_sensors) {
    const std::size_t active = tactile.activeHemisphereCount();
    active_hemisphere_count += active;
    if (active > 0U) {
      ++active_sensor_count;
    }
  }
  const bool has_live_contact =
    active_hemisphere_count > 0U || !result.contacts.empty();

  Eigen::Isometry3d representative_pose = Eigen::Isometry3d::Identity();
  const bool has_corrected_pose =
    estimator_.has_belief() &&
    (!hide_corrected_markers_without_contact_ || has_live_contact) &&
    estimator_.representativePose(&representative_pose);
  if (has_corrected_pose) {
    auto object_marker = BaseMarker(
      world_frame_, stamp, "object_prior_corrected_block", 0,
      visualization_msgs::msg::Marker::CUBE);
    object_marker.pose = ToPoseMsg(representative_pose);
    ApplyObjectGeometryScale(estimator_.belief().geometry, &object_marker);
    object_marker.color = Color(0.0F, 0.95F, 1.0F, 0.78F);
    markers.markers.push_back(object_marker);

    auto outline_marker = BaseMarker(
      world_frame_, stamp, "object_prior_corrected_block_outline", 0,
      visualization_msgs::msg::Marker::LINE_LIST);
    outline_marker.pose = ToPoseMsg(representative_pose);
    outline_marker.scale.x = 0.004;
    outline_marker.color = Color(0.0F, 1.0F, 1.0F, 1.0F);
    AddBoxOutlinePoints(object_marker.scale, &outline_marker);
    markers.markers.push_back(outline_marker);

    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_uncorrected_block", 0));
  } else if (publish_prior_marker_when_no_belief_) {
    auto prior_marker = BaseMarker(
      world_frame_, stamp, "object_prior_uncorrected_block", 0,
      visualization_msgs::msg::Marker::CUBE);
    prior_marker.pose = ToPoseMsg(estimator_.prior().initial_pose_world);
    ApplyObjectGeometryScale(estimator_.prior().geometry, &prior_marker);
    prior_marker.color = Color(0.8F, 0.8F, 0.8F, 0.22F);
    markers.markers.push_back(prior_marker);

    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_corrected_block", 0));
    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_corrected_block_outline", 0));
  } else {
    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_corrected_block", 0));
    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_corrected_block_outline", 0));
    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_uncorrected_block", 0));
  }

  if (estimator_.has_belief() &&
    (!hide_corrected_markers_without_contact_ || has_live_contact))
  {
    auto particle_marker = BaseMarker(
      world_frame_, stamp, "object_prior_particles", 0,
      visualization_msgs::msg::Marker::SPHERE_LIST);
    particle_marker.scale.x = 0.008;
    particle_marker.scale.y = 0.008;
    particle_marker.scale.z = 0.008;
    particle_marker.color = Color(1.0F, 0.8F, 0.1F, 0.65F);
    for (const auto & particle : estimator_.belief().particles) {
      if (mppi_core::IsValidVirtualObjectState(particle)) {
        particle_marker.points.push_back(PointMsg(particle.pose_world.translation()));
      }
    }
    markers.markers.push_back(particle_marker);
  } else {
    markers.markers.push_back(
      DeleteMarker(world_frame_, stamp, "object_prior_particles", 0));
  }

  auto contact_marker = BaseMarker(
    world_frame_, stamp, "object_prior_contacts", 0,
    visualization_msgs::msg::Marker::SPHERE_LIST);
  contact_marker.scale.x = 0.006;
  contact_marker.scale.y = 0.006;
  contact_marker.scale.z = 0.006;
  contact_marker.color = Color(0.0F, 1.0F, 0.25F, 0.9F);

  auto normal_marker = BaseMarker(
    world_frame_, stamp, "object_prior_contact_normals", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  normal_marker.scale.x = 0.0015;
  normal_marker.color = Color(0.0F, 1.0F, 0.25F, 0.8F);

  constexpr double kNormalLengthM = 0.025;
  for (const auto & contact : result.contacts) {
    if (!contact.point_world_m.allFinite() ||
      !contact.normal_world.allFinite() ||
      contact.normal_world.norm() <= 1.0e-12)
    {
      continue;
    }
    const Eigen::Vector3d normal = contact.normal_world.normalized();
    contact_marker.points.push_back(PointMsg(contact.point_world_m));
    normal_marker.points.push_back(PointMsg(contact.point_world_m));
    normal_marker.points.push_back(PointMsg(contact.point_world_m + kNormalLengthM * normal));
  }
  markers.markers.push_back(contact_marker);
  markers.markers.push_back(normal_marker);

  Eigen::Vector3d text_position = estimator_.prior().initial_pose_world.translation();
  if (has_corrected_pose) {
    text_position = representative_pose.translation();
  }
  auto text_marker = BaseMarker(
    world_frame_, stamp, "object_prior_status", 0,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  text_marker.pose.position = PointMsg(text_position + Eigen::Vector3d{0.0, 0.0, 0.08});
  text_marker.scale.z = 0.025;
  text_marker.color = Color(1.0F, 1.0F, 1.0F, 0.9F);
  std::ostringstream text;
  text << std::fixed << std::setprecision(2);
  text << "belief=" << (estimator_.status().belief_valid ? "true" : "false")
       << " corrected_block=" << (has_corrected_pose ? "true" : "false")
       << " live_contact=" << (has_live_contact ? "true" : "false")
       << " contacts=" << result.contacts.size()
       << " active_sensors=" << active_sensor_count;
  for (std::size_t i = 0; i < tactile_sensors.size(); ++i) {
    const auto & tactile = tactile_sensors[i];
    const double force_z_n =
      tactile.total_force_n.allFinite() ? tactile.total_force_n.z() : 0.0;
    text << "\ntactile" << i
         << " force_z=" << force_z_n << "N"
         << " hemispheres=" << tactile.activeHemisphereCount();
  }
  if (tactile_sensors.empty()) {
    text << "\ntactile0 force_z=nanN hemispheres=0"
         << "\ntactile1 force_z=nanN hemispheres=0";
  } else if (tactile_sensors.size() == 1U) {
    text << "\ntactile1 force_z=nanN hemispheres=0";
  }
  text_marker.text = text.str();
  markers.markers.push_back(text_marker);

  marker_pub_->publish(markers);
}

}  // namespace plato_state_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plato_state_estimator::ObjectPriorEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
