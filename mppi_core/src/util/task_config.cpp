// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/task/task_config.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/util/yaml_utils.hpp"

namespace mppi_core {
namespace {

YAML::Node TaskConfigNode(const YAML::Node& root) {
  return yaml_utils::RootSectionOrRoot(root, "task", "Task YAML");
}

void CheckMap(const YAML::Node& params, const char* function_name) {
  if (yaml_utils::HasValue(params) && !params.IsMap()) {
    throw std::invalid_argument(std::string(function_name) +
                                ": params must be a map");
  }
}

Eigen::Vector3d ReadVector3(const YAML::Node& node, const char* key,
                            const Eigen::Vector3d& default_value) {
  const YAML::Node value = yaml_utils::HasValue(node) ? node[key] : YAML::Node();
  if (!yaml_utils::HasValue(value)) {
    return default_value;
  }
  if (!value.IsSequence() || value.size() != 3U) {
    throw std::invalid_argument(std::string("Field '") + key +
                                "' must be a 3-vector");
  }
  return Eigen::Vector3d{value[0].as<double>(), value[1].as<double>(),
                         value[2].as<double>()};
}

Eigen::Matrix3d RpyToRotation(const Eigen::Vector3d& rpy_rad) {
  const Eigen::AngleAxisd roll(rpy_rad.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy_rad.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy_rad.z(), Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

std::string Lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}

ObjectGeometryType ParseObjectGeometryType(const std::string& value,
                                           ObjectGeometryType default_type) {
  if (value.empty()) {
    return default_type;
  }
  const std::string lower = Lowercase(value);
  if (lower == "box") {
    return ObjectGeometryType::kBox;
  }
  if (lower == "sphere") {
    return ObjectGeometryType::kSphere;
  }
  if (lower == "cylinder") {
    return ObjectGeometryType::kCylinder;
  }
  if (lower == "mesh") {
    return ObjectGeometryType::kMesh;
  }
  if (lower == "urdf") {
    return ObjectGeometryType::kUrdf;
  }
  if (lower == "unknown") {
    return ObjectGeometryType::kUnknown;
  }
  throw std::invalid_argument("ParseTaskConfig: unknown object geometry_type '" +
                              value + "'");
}

ObjectPrior ParseObjectPrior(const YAML::Node& object,
                             ObjectPrior defaults) {
  if (!yaml_utils::HasValue(object)) {
    return defaults;
  }
  CheckMap(object, "ParseObjectPrior");
  if (!yaml_utils::ReadBool(object, "enabled", true)) {
    return ObjectPrior{};
  }

  defaults.name = yaml_utils::ReadString(object, "name", defaults.name);
  defaults.geometry.name = yaml_utils::ReadString(
      object, "geometry_name",
      defaults.geometry.name.empty() ? defaults.name : defaults.geometry.name);

  std::string geometry_type = yaml_utils::ReadString(object, "geometry_type", "");
  geometry_type = yaml_utils::ReadString(object, "type", geometry_type);
  defaults.geometry.type =
      ParseObjectGeometryType(geometry_type, defaults.geometry.type);

  defaults.geometry.uri =
      yaml_utils::ReadString(object, "uri", defaults.geometry.uri);
  defaults.geometry.uri =
      yaml_utils::ReadString(object, "mesh_path", defaults.geometry.uri);
  defaults.geometry.primitive_size_m = ReadVector3(
      object, "primitive_size_m", defaults.geometry.primitive_size_m);
  defaults.geometry.primitive_size_m =
      ReadVector3(object, "size_m", defaults.geometry.primitive_size_m);
  defaults.geometry.primitive_size_m =
      ReadVector3(object, "box_size_m", defaults.geometry.primitive_size_m);

  const YAML::Node initial_pose =
      yaml_utils::ReadSection(object, "initial_pose_world");
  const Eigen::Vector3d xyz = ReadVector3(
      initial_pose, "xyz", defaults.initial_pose_world.translation());
  const Eigen::Vector3d rpy = ReadVector3(
      initial_pose, "rpy", Eigen::Vector3d::Zero());
  defaults.initial_pose_world = Eigen::Isometry3d::Identity();
  defaults.initial_pose_world.translation() = xyz;
  defaults.initial_pose_world.linear() = RpyToRotation(rpy);

  YAML::Node pose_noise;
  if (yaml_utils::HasValue(object["initial_pose_uncertainty"])) {
    pose_noise = yaml_utils::ReadSection(object, "initial_pose_uncertainty");
  } else if (yaml_utils::HasValue(object["perturbation"])) {
    pose_noise = yaml_utils::ReadSection(object, "perturbation");
  } else {
    pose_noise = yaml_utils::ReadSection(object, "pose_noise");
  }
  defaults.position_std_m =
      ReadVector3(pose_noise, "xyz_std_m", defaults.position_std_m);
  defaults.rpy_std_rad =
      ReadVector3(pose_noise, "rpy_std_rad", defaults.rpy_std_rad);

  defaults.geometry.valid = true;
  defaults.valid = true;
  if (!IsValidObjectPrior(defaults)) {
    throw std::invalid_argument("ParseTaskConfig: invalid object prior");
  }
  return defaults;
}

}  // namespace

TactileTransitionConfig ApplyTaskToleranceToTransitionConfig(
    const TaskConfig& task, TactileTransitionConfig transition) {
  transition.max_shear_m = task.tolerance.max_shear_m;
  transition.max_rotation_rad = task.tolerance.max_rotation_rad;
  return transition;
}

TaskConfig ParseTaskConfig(const YAML::Node& params, TaskConfig defaults) {
  const YAML::Node safe_params =
      yaml_utils::HasValue(params) ? params : YAML::Node();
  CheckMap(safe_params, "ParseTaskConfig");

  defaults.name =
      yaml_utils::ReadString(safe_params, "name", defaults.name);

  const YAML::Node start = yaml_utils::ReadSection(safe_params, "start");
  defaults.start.min_enough_contact_sensors = yaml_utils::ReadSize(
      start, "min_enough_contact_sensors",
      defaults.start.min_enough_contact_sensors);
  defaults.start.min_active_hemispheres_total = yaml_utils::ReadSize(
      start, "min_active_hemisphere_total",
      defaults.start.min_active_hemispheres_total);

  const YAML::Node objective =
      yaml_utils::ReadSection(safe_params, "objective");
  defaults.cost.min_active_tactile_sensors = yaml_utils::ReadSize(
      objective, "min_active_tactile_sensors",
      defaults.cost.min_active_tactile_sensors);
  defaults.cost.target_active_hemisphere_total = yaml_utils::ReadSize(
      objective, "target_active_hemisphere_total",
      defaults.cost.target_active_hemisphere_total);

  const YAML::Node tolerance =
      yaml_utils::ReadSection(safe_params, "tolerance");
  defaults.tolerance.max_shear_m = yaml_utils::ReadDouble(
      tolerance, "max_shear_m", defaults.tolerance.max_shear_m);
  defaults.tolerance.max_rotation_rad = yaml_utils::ReadDouble(
      tolerance, "max_rotation_rad", defaults.tolerance.max_rotation_rad);

  const YAML::Node cost = yaml_utils::ReadSection(safe_params, "cost");
  defaults.cost.contact_loss_weight = yaml_utils::ReadDouble(
      cost, "contact_loss_weight", defaults.cost.contact_loss_weight);
  defaults.cost.support_weight =
      yaml_utils::ReadDouble(cost, "support_weight",
                             defaults.cost.support_weight);
  defaults.cost.shear_weight =
      yaml_utils::ReadDouble(cost, "shear_weight",
                             defaults.cost.shear_weight);
  defaults.cost.rotation_weight =
      yaml_utils::ReadDouble(cost, "rotation_weight",
                             defaults.cost.rotation_weight);
  defaults.cost.qddot_weight =
      yaml_utils::ReadDouble(cost, "qddot_weight",
                             defaults.cost.qddot_weight);
  defaults.cost.tau_weight =
      yaml_utils::ReadDouble(cost, "tau_weight", defaults.cost.tau_weight);

  defaults.object_prior =
      ParseObjectPrior(yaml_utils::ReadSection(safe_params, "object"),
                       defaults.object_prior);
  return defaults;
}

TaskConfig LoadTaskConfigFromYamlFile(const std::string& yaml_path,
                                      TaskConfig defaults) {
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    return ParseTaskConfig(TaskConfigNode(root), std::move(defaults));
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error("LoadTaskConfigFromYamlFile: failed to load '" +
                             yaml_path + "': " + ex.what());
  }
}

}  // namespace mppi_core
