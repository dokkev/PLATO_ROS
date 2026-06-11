// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/policy/grasp_action_library.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "mppi_core/contact/contact_kinematics.hpp"

namespace mppi_core {
namespace {

struct SensorActionBasis {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  double force_n{0.0};
  Eigen::RowVectorXd normal_jacobian;
  Eigen::Vector3d centroid_world{Eigen::Vector3d::Zero()};
  Eigen::MatrixXd centroid_jacobian_world;
};

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

Eigen::VectorXd DefaultBound(const std::size_t dim, const double value) {
  return Eigen::VectorXd::Constant(static_cast<Eigen::Index>(dim), value);
}

void ValidateVectorDim(const Eigen::VectorXd& value, const std::size_t dim,
                       const char* name) {
  if (value.size() != static_cast<Eigen::Index>(dim)) {
    throw std::invalid_argument(std::string("GraspActionLibraryConfig: ") +
                                name + " dimension mismatch");
  }
}

void ValidateConfig(GraspActionLibraryConfig* config) {
  if (config == nullptr || config->horizon_steps == 0 ||
      config->action_dim == 0 || config->num_action_samples == 0 ||
      !std::isfinite(config->dt) || config->dt <= 0.0 ||
      !IsFiniteAndNonnegative(config->target_min_normal_force_n) ||
      !IsFiniteAndNonnegative(config->force_to_squeeze_gain) ||
      !IsFiniteAndNonnegative(config->force_balance_gain) ||
      !IsFiniteAndNonnegative(config->contact_line_align_gain) ||
      !IsFiniteAndNonnegative(config->squeeze_std) ||
      !IsFiniteAndNonnegative(config->align_std) ||
      !IsFiniteAndNonnegative(config->force_balance_std) ||
      !IsFiniteAndNonnegative(config->thumb_bias_std) ||
      !IsFiniteAndNonnegative(config->index_bias_std) ||
      !IsFiniteAndNonnegative(config->max_squeeze) ||
      !IsFiniteAndNonnegative(config->max_align) ||
      !IsFiniteAndNonnegative(config->max_force_balance) ||
      !IsFiniteAndNonnegative(config->max_thumb_bias) ||
      !IsFiniteAndNonnegative(config->max_index_bias) ||
      !IsFiniteAndNonnegative(config->squeeze_light_accel_scale) ||
      !IsFiniteAndNonnegative(config->squeeze_medium_accel_scale) ||
      !IsFiniteAndNonnegative(config->squeeze_strong_accel_scale) ||
      !IsFiniteAndNonnegative(config->release_accel_scale) ||
      !IsFiniteAndNonnegative(config->align_accel_scale) ||
      !std::isfinite(config->sequence_decay) ||
      !config->close_axis_base.allFinite() ||
      config->close_axis_base.norm() <= 1.0e-12) {
    throw std::invalid_argument(
        "GraspActionLibraryConfig: invalid numeric field");
  }
  if (config->qddot_lower_bound.size() == 0) {
    config->qddot_lower_bound = DefaultBound(
        config->action_dim, -std::numeric_limits<double>::infinity());
  }
  if (config->qddot_upper_bound.size() == 0) {
    config->qddot_upper_bound = DefaultBound(
        config->action_dim, std::numeric_limits<double>::infinity());
  }
  ValidateVectorDim(config->qddot_lower_bound, config->action_dim,
                    "qddot_lower_bound");
  ValidateVectorDim(config->qddot_upper_bound, config->action_dim,
                    "qddot_upper_bound");
  for (Eigen::Index i = 0; i < config->qddot_lower_bound.size(); ++i) {
    if (std::isnan(config->qddot_lower_bound[i]) ||
        std::isnan(config->qddot_upper_bound[i]) ||
        config->qddot_lower_bound[i] > config->qddot_upper_bound[i]) {
      throw std::invalid_argument(
          "GraspActionLibraryConfig: invalid qddot bounds");
    }
  }
}

bool NormalizeInPlace(Eigen::VectorXd* value) {
  if (value == nullptr || !value->allFinite()) {
    return false;
  }
  const double norm = value->norm();
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return false;
  }
  *value /= norm;
  return true;
}

Eigen::VectorXd NormalizedOrZero(const Eigen::VectorXd& value,
                                 const std::size_t dim) {
  Eigen::VectorXd out = value;
  if (out.size() != static_cast<Eigen::Index>(dim) ||
      !NormalizeInPlace(&out)) {
    return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  }
  return out;
}

double ClampSymmetric(const double value, const double limit) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (!std::isfinite(limit)) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

bool ComputeSensorActionBasis(const GraspState& state,
                              const TactileState& tactile,
                              const TactileSensorContext& tactile_context,
                              const std::size_t action_dim,
                              SensorActionBasis* basis) {
  if (basis == nullptr || !state.valid || !IsValid(state.robot) ||
      tactile_context.kinematics == nullptr ||
      !IsValidContactKinematicsContext(*tactile_context.kinematics) ||
      action_dim == 0) {
    return false;
  }

  const auto& kinematics = *tactile_context.kinematics;
  const auto& model = *kinematics.model;
  if (state.robot.q.size() != static_cast<Eigen::Index>(model.nq) ||
      static_cast<Eigen::Index>(action_dim) != model.nv) {
    return false;
  }

  Eigen::Vector2d centroid_sensor_xy = Eigen::Vector2d::Zero();
  std::size_t active_count = 0;
  double force_n = 0.0;
  for (const auto& hemisphere : tactile.hemispheres) {
    if (hemisphere.contact && hemisphere.cop_sensor_m.allFinite()) {
      centroid_sensor_xy += hemisphere.cop_sensor_m;
      force_n += std::max(0.0, hemisphere.normal_force_n);
      ++active_count;
    }
  }
  if (active_count == 0) {
    return false;
  }
  centroid_sensor_xy /= static_cast<double>(active_count);

  const Eigen::VectorXd zero_tangent =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
  const auto motions = ComputeHemisphereMotions(
      state.robot, tactile, zero_tangent, kinematics);
  if (motions.empty()) {
    return false;
  }
  Eigen::VectorXd normal_jacobian_sum =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
  std::size_t normal_row_count = 0;
  for (const auto& motion : motions) {
    if (motion.J_normal.size() ==
        static_cast<Eigen::Index>(action_dim)) {
      normal_jacobian_sum += motion.J_normal.transpose();
      ++normal_row_count;
    }
  }
  if (normal_row_count == 0) {
    return false;
  }

  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, state.robot.q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, state.robot.q);

  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian_sensor(6, model.nv);
  frame_jacobian_sensor.setZero();
  pinocchio::getFrameJacobian(model, data, kinematics.sensor_frame_id,
                              pinocchio::LOCAL, frame_jacobian_sensor);

  const Eigen::Vector3d centroid_sensor{
      centroid_sensor_xy.x(), centroid_sensor_xy.y(), 0.0};
  Eigen::Matrix<double, 3, Eigen::Dynamic> point_jacobian_sensor(3, model.nv);
  if (!ComputeContactPointJacobianSensor(
          frame_jacobian_sensor, centroid_sensor, &point_jacobian_sensor)) {
    return false;
  }

  basis->valid = true;
  basis->force_n = force_n;
  basis->normal_jacobian =
      (normal_jacobian_sum / static_cast<double>(normal_row_count)).transpose();
  basis->centroid_world =
      data.oMf[kinematics.sensor_frame_id].act(centroid_sensor);
  basis->centroid_jacobian_world =
      data.oMf[kinematics.sensor_frame_id].rotation() *
      point_jacobian_sensor;
  return basis->normal_jacobian.allFinite() &&
         basis->centroid_world.allFinite() &&
         basis->centroid_jacobian_world.allFinite();
}

}  // namespace

struct GraspActionLibrary::GraspActionBasis {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  bool has_thumb{false};
  bool has_index{false};
  bool has_align{false};

  double thumb_force_n{0.0};
  double index_force_n{0.0};
  double align_error_m{0.0};

  Eigen::VectorXd squeeze;
  Eigen::VectorXd align;
  Eigen::VectorXd force_balance;
  Eigen::VectorXd thumb_bias;
  Eigen::VectorXd index_bias;
};

GraspActionLibrary::GraspActionLibrary(GraspActionLibraryConfig config)
    : config_(std::move(config)), rng_(config_.random_seed) {
  ValidateConfig(&config_);
}

std::vector<ActionSequence> GraspActionLibrary::BuildCandidates(
    const GraspState& state, const RolloutContext& context) {
  std::vector<ActionSequence> candidates;
  const Eigen::VectorXd zero =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  candidates.push_back(BuildDecayedSequence(zero));

  GraspActionBasis basis;
  if (!BuildActionBasis(state, context, &basis)) {
    return candidates;
  }

  const GraspCorrectiveAction nominal = BuildNominalAction(basis);
  candidates.push_back(BuildDecayedSequence(MapActionToQddot(nominal, basis)));
  if (config_.include_basis_probe_actions) {
    AddBasisProbeActions(basis, &candidates);
  }
  for (std::size_t i = 0; i < config_.num_action_samples; ++i) {
    const GraspCorrectiveAction sample = SampleActionAround(nominal);
    candidates.push_back(BuildDecayedSequence(MapActionToQddot(sample, basis)));
  }
  return candidates;
}

ActionSequence GraspActionLibrary::BuildDecayedSequence(
    const Eigen::Ref<const Eigen::VectorXd>& first_action) const {
  if (first_action.size() != static_cast<Eigen::Index>(config_.action_dim)) {
    throw std::invalid_argument(
        "GraspActionLibrary::BuildDecayedSequence: action dimension mismatch");
  }

  ActionSequence sequence(config_.action_dim, config_.horizon_steps);
  for (std::size_t step = 0; step < config_.horizon_steps; ++step) {
    const double decay =
        std::pow(config_.sequence_decay, static_cast<double>(step));
    sequence.setAction(step, ClampAction(first_action * decay));
  }
  return sequence;
}

Eigen::VectorXd GraspActionLibrary::ClampAction(
    const Eigen::VectorXd& action) const {
  Eigen::VectorXd clamped = action;
  for (Eigen::Index i = 0; i < clamped.size(); ++i) {
    clamped[i] = std::clamp(
        clamped[i], config_.qddot_lower_bound[i],
        config_.qddot_upper_bound[i]);
  }
  return clamped;
}

bool GraspActionLibrary::BuildActionBasis(
    const GraspState& state, const RolloutContext& context,
    GraspActionBasis* basis) const {
  if (basis == nullptr || !state.valid ||
      state.tactile_sensors.size() != context.tactile_contexts.size()) {
    return false;
  }

  std::vector<SensorActionBasis> sensor_bases;
  sensor_bases.reserve(state.tactile_sensors.size());
  for (std::size_t i = 0; i < state.tactile_sensors.size(); ++i) {
    SensorActionBasis sensor_basis;
    if (ComputeSensorActionBasis(
            state, state.tactile_sensors[i], context.tactile_contexts[i],
            config_.action_dim, &sensor_basis)) {
      sensor_bases.push_back(std::move(sensor_basis));
    }
  }
  if (sensor_bases.empty()) {
    return false;
  }

  Eigen::VectorXd squeeze_sum =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  for (const auto& sensor_basis : sensor_bases) {
    squeeze_sum += sensor_basis.normal_jacobian.transpose();
  }
  basis->squeeze = NormalizedOrZero(squeeze_sum, config_.action_dim);
  basis->valid = basis->squeeze.norm() > 0.0;
  if (!basis->valid) {
    return false;
  }

  if (sensor_bases.size() >= 1U) {
    basis->has_thumb = true;
    basis->thumb_force_n = sensor_bases[0].force_n;
    basis->thumb_bias =
        NormalizedOrZero(sensor_bases[0].normal_jacobian.transpose(),
                         config_.action_dim);
  } else {
    basis->thumb_bias =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  }
  if (sensor_bases.size() >= 2U) {
    basis->has_index = true;
    basis->index_force_n = sensor_bases[1].force_n;
    basis->index_bias =
        NormalizedOrZero(sensor_bases[1].normal_jacobian.transpose(),
                         config_.action_dim);
    basis->force_balance =
        NormalizedOrZero(basis->thumb_bias - basis->index_bias,
                         config_.action_dim);

    const Eigen::Vector3d close_axis = config_.close_axis_base.normalized();
    const Eigen::Vector3d d =
        sensor_bases[1].centroid_world - sensor_bases[0].centroid_world;
    const Eigen::Vector3d tangent_error = d - d.dot(close_axis) * close_axis;
    basis->align_error_m = tangent_error.norm();
    const Eigen::MatrixXd j_diff =
        sensor_bases[1].centroid_jacobian_world -
        sensor_bases[0].centroid_jacobian_world;
    const Eigen::VectorXd align_gradient = j_diff.transpose() * tangent_error;
    basis->align =
        NormalizedOrZero(-align_gradient, config_.action_dim);
    basis->has_align = basis->align.norm() > 0.0;
  } else {
    basis->index_bias =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
    basis->force_balance =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
    basis->align =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  }
  return true;
}

GraspCorrectiveAction GraspActionLibrary::BuildNominalAction(
    const GraspActionBasis& basis) const {
  GraspCorrectiveAction action;
  const double min_force =
      basis.has_thumb && basis.has_index
          ? std::min(basis.thumb_force_n, basis.index_force_n)
          : (basis.has_thumb ? basis.thumb_force_n : basis.index_force_n);
  action.squeeze = std::clamp(
      config_.force_to_squeeze_gain *
          std::max(0.0, config_.target_min_normal_force_n - min_force),
      0.0, config_.max_squeeze);
  if (basis.has_align) {
    action.align_lateral = std::clamp(
        config_.contact_line_align_gain * basis.align_error_m,
        0.0, config_.max_align);
  }
  if (basis.has_thumb && basis.has_index) {
    action.force_balance = ClampSymmetric(
        config_.force_balance_gain *
            (basis.index_force_n - basis.thumb_force_n),
        config_.max_force_balance);
  }
  return action;
}

Eigen::VectorXd GraspActionLibrary::MapActionToQddot(
    const GraspCorrectiveAction& action,
    const GraspActionBasis& basis) const {
  Eigen::VectorXd qddot =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config_.action_dim));
  qddot += basis.squeeze * (action.squeeze - action.release);
  qddot += basis.align * action.align_lateral;
  qddot += basis.force_balance * action.force_balance;
  qddot += basis.thumb_bias * action.thumb_bias;
  qddot += basis.index_bias * action.index_bias;
  return ClampAction(qddot);
}

GraspCorrectiveAction GraspActionLibrary::SampleActionAround(
    const GraspCorrectiveAction& nominal) {
  const auto sample_coeff = [this](const double center, const double stddev,
                                   const double limit) {
      if (stddev <= 0.0 || limit <= 0.0) {
        return ClampSymmetric(center, limit);
      }
      std::normal_distribution<double> normal(center, stddev);
      return ClampSymmetric(normal(rng_), limit);
    };

  GraspCorrectiveAction action;
  const double squeeze_signed =
      sample_coeff(nominal.squeeze - nominal.release,
                   config_.squeeze_std, config_.max_squeeze);
  if (squeeze_signed >= 0.0) {
    action.squeeze = squeeze_signed;
  } else {
    action.release = -squeeze_signed;
  }
  action.align_lateral =
      sample_coeff(nominal.align_lateral, config_.align_std,
                   config_.max_align);
  action.force_balance =
      sample_coeff(nominal.force_balance, config_.force_balance_std,
                   config_.max_force_balance);
  action.thumb_bias =
      sample_coeff(nominal.thumb_bias, config_.thumb_bias_std,
                   config_.max_thumb_bias);
  action.index_bias =
      sample_coeff(nominal.index_bias, config_.index_bias_std,
                   config_.max_index_bias);
  return action;
}

void GraspActionLibrary::AddBasisProbeActions(
    const GraspActionBasis& basis,
    std::vector<ActionSequence>* candidates) const {
  if (candidates == nullptr) {
    return;
  }
  const auto add = [&](const GraspCorrectiveAction& action) {
      candidates->push_back(BuildDecayedSequence(MapActionToQddot(action, basis)));
    };

  GraspCorrectiveAction action;
  action.squeeze = config_.squeeze_light_accel_scale;
  add(action);
  action.squeeze = config_.squeeze_medium_accel_scale;
  add(action);
  action.squeeze = config_.squeeze_strong_accel_scale;
  add(action);
  action = GraspCorrectiveAction{};
  action.release = config_.release_accel_scale;
  add(action);

  action = GraspCorrectiveAction{};
  action.thumb_bias = config_.squeeze_light_accel_scale;
  add(action);
  action = GraspCorrectiveAction{};
  action.index_bias = config_.squeeze_light_accel_scale;
  add(action);

  if (basis.has_align) {
    action = GraspCorrectiveAction{};
    action.align_lateral = config_.align_accel_scale;
    add(action);
    action.align_lateral = -config_.align_accel_scale;
    add(action);
    action = GraspCorrectiveAction{};
    action.squeeze = config_.squeeze_light_accel_scale;
    action.align_lateral = config_.align_accel_scale;
    add(action);
    action.align_lateral = -config_.align_accel_scale;
    add(action);
  }
}

}  // namespace mppi_core
