// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/object/object_belief_initializer.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/object/object_geometry_query.hpp"

namespace mppi_core {
namespace {

constexpr double kTiny = 1.0e-12;

bool IsFinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

double PositiveOrDefault(double value, double fallback) {
  return IsFinitePositive(value) ? value : fallback;
}

Eigen::Vector3d PositiveStdOrFallback(const Eigen::Vector3d& value,
                                      const Eigen::Vector3d& fallback) {
  Eigen::Vector3d out = fallback;
  for (Eigen::Index i = 0; i < 3; ++i) {
    if (IsFinitePositive(value[i])) {
      out[i] = value[i];
    }
  }
  return out;
}

double SafeSquared(double value) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::infinity();
  }
  return value * value;
}

Eigen::Matrix3d RpyToRotation(const Eigen::Vector3d& rpy_rad) {
  const Eigen::AngleAxisd roll(rpy_rad.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy_rad.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy_rad.z(), Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

double RotationDistanceRad(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::Matrix3d relative = a.transpose() * b;
  return Eigen::AngleAxisd(relative).angle();
}

Eigen::Isometry3d SamplePoseAroundPrior(
    const ObjectPrior& prior, const Eigen::Vector3d& position_sample_m,
    const Eigen::Vector3d& rpy_sample_rad) {
  Eigen::Isometry3d pose = prior.initial_pose_world;
  pose.translation() += position_sample_m;
  pose.linear() = prior.initial_pose_world.linear() *
                  RpyToRotation(rpy_sample_rad);
  return pose;
}

double ContactWeight(const ObjectContactObservation& contact,
                     const ObjectBeliefInitializationConfig& config) {
  const double confidence =
      std::isfinite(contact.confidence)
          ? std::clamp(contact.confidence, 0.0, 1.0)
          : 0.0;
  const double force_scale =
      PositiveOrDefault(config.contact_force_weight_scale_n, 1.0);
  const double force_n =
      std::isfinite(contact.normal_force_n)
          ? std::max(0.0, contact.normal_force_n)
          : 0.0;
  return confidence * (1.0 + force_n / force_scale);
}

struct SensorContactAggregate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t sensor_index{0};
  Eigen::Vector3d weighted_point_world_m{Eigen::Vector3d::Zero()};
  double weight_sum{0.0};

  Eigen::Vector3d pointWorld() const {
    if (weight_sum > kTiny) {
      return weighted_point_world_m / weight_sum;
    }
    return Eigen::Vector3d::Zero();
  }
};

struct ContactWidthObservation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  Eigen::Vector3d point_a_world_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d point_b_world_m{Eigen::Vector3d::Zero()};
  double measured_width_m{0.0};
};

ContactWidthObservation MakeContactWidthObservation(
    const std::vector<ObjectContactObservation,
                      Eigen::aligned_allocator<ObjectContactObservation>>&
        contacts,
    const ObjectBeliefInitializationConfig& config) {
  ContactWidthObservation width;
  if (!config.use_thumb_index_contact_width ||
      std::max(0.0, config.w_contact_width) <= 0.0) {
    return width;
  }

  std::vector<SensorContactAggregate,
              Eigen::aligned_allocator<SensorContactAggregate>>
      sensor_contacts;
  for (const auto& contact : contacts) {
    if (!contact.point_world_m.allFinite()) {
      continue;
    }
    const double contact_weight = ContactWeight(contact, config);
    if (contact_weight <= kTiny) {
      continue;
    }

    auto found = std::find_if(
        sensor_contacts.begin(), sensor_contacts.end(),
        [&contact](const SensorContactAggregate& aggregate) {
          return aggregate.sensor_index == contact.sensor_index;
        });
    if (found == sensor_contacts.end()) {
      SensorContactAggregate aggregate;
      aggregate.sensor_index = contact.sensor_index;
      sensor_contacts.push_back(aggregate);
      found = sensor_contacts.end() - 1;
    }
    found->weighted_point_world_m += contact_weight * contact.point_world_m;
    found->weight_sum += contact_weight;
  }

  sensor_contacts.erase(
      std::remove_if(
          sensor_contacts.begin(), sensor_contacts.end(),
          [](const SensorContactAggregate& aggregate) {
            return aggregate.weight_sum <= kTiny;
          }),
      sensor_contacts.end());
  if (sensor_contacts.size() < 2) {
    return width;
  }

  std::sort(
      sensor_contacts.begin(), sensor_contacts.end(),
      [](const SensorContactAggregate& lhs,
         const SensorContactAggregate& rhs) {
        return lhs.weight_sum > rhs.weight_sum;
      });

  width.point_a_world_m = sensor_contacts[0].pointWorld();
  width.point_b_world_m = sensor_contacts[1].pointWorld();
  width.measured_width_m =
      (width.point_b_world_m - width.point_a_world_m).norm();
  width.valid =
      width.point_a_world_m.allFinite() &&
      width.point_b_world_m.allFinite() &&
      std::isfinite(width.measured_width_m) &&
      width.measured_width_m > kTiny;
  return width;
}

void ScoreContactWidth(
    const ObjectPrior& prior,
    const Eigen::Isometry3d& particle_pose_world,
    const ContactWidthObservation& width,
    const ObjectBeliefInitializationConfig& config,
    ObjectParticleScore* score) {
  if (score == nullptr || !width.valid) {
    return;
  }

  const ObjectSurfaceQueryResult surface_a = QueryObjectSurface(
      prior.geometry, particle_pose_world, width.point_a_world_m);
  const ObjectSurfaceQueryResult surface_b = QueryObjectSurface(
      prior.geometry, particle_pose_world, width.point_b_world_m);
  if (!surface_a.valid || !surface_b.valid ||
      !surface_a.closest_point_world.allFinite() ||
      !surface_b.closest_point_world.allFinite()) {
    return;
  }

  const double particle_width_m =
      (surface_b.closest_point_world - surface_a.closest_point_world).norm();
  if (!std::isfinite(particle_width_m)) {
    return;
  }

  const double width_sigma =
      PositiveOrDefault(config.contact_width_sigma_m, 0.005);
  score->contact_width_error_m =
      particle_width_m - width.measured_width_m;
  score->contact_width_cost =
      SafeSquared(score->contact_width_error_m / width_sigma);
}

ObjectParticleScore ScoreParticle(
    const ObjectPrior& prior, const Eigen::Isometry3d& particle_pose_world,
    const std::vector<ObjectContactObservation,
                      Eigen::aligned_allocator<ObjectContactObservation>>&
        contacts,
    const ContactWidthObservation& contact_width,
    const ObjectBeliefInitializationConfig& config) {
  ObjectParticleScore score;
  if (!particle_pose_world.matrix().allFinite() || contacts.empty()) {
    return score;
  }

  const double surface_sigma =
      PositiveOrDefault(config.surface_distance_sigma_m, 0.005);
  const double normal_sigma =
      PositiveOrDefault(config.normal_alignment_sigma, 0.25);
  const double prior_position_sigma =
      PositiveOrDefault(config.prior_position_sigma_m, 0.02);
  const double prior_rotation_sigma =
      PositiveOrDefault(config.prior_rotation_sigma_rad, 0.35);

  double surface_cost_sum = 0.0;
  double normal_cost_sum = 0.0;
  double surface_distance_sum = 0.0;
  double normal_error_sum = 0.0;
  double weight_sum = 0.0;
  for (const auto& contact : contacts) {
    if (!contact.point_world_m.allFinite() ||
        !contact.normal_world.allFinite() ||
        contact.normal_world.norm() <= kTiny) {
      continue;
    }
    const double contact_weight = ContactWeight(contact, config);
    if (contact_weight <= kTiny) {
      continue;
    }
    const ObjectSurfaceQueryResult surface = QueryObjectSurface(
        prior.geometry, particle_pose_world, contact.point_world_m);
    if (!surface.valid || !std::isfinite(surface.signed_distance_m) ||
        surface.normal_world.norm() <= kTiny) {
      continue;
    }

    const Eigen::Vector3d tactile_normal_world =
        contact.normal_world.normalized();
    const Eigen::Vector3d surface_normal_world =
        surface.normal_world.normalized();

    const double normal_dot =
        std::clamp(surface_normal_world.dot(tactile_normal_world), -1.0, 1.0);
    const double opposing_normal_error = 1.0 + normal_dot;
    const double surface_distance_m = std::abs(surface.signed_distance_m);

    surface_cost_sum +=
        contact_weight * SafeSquared(surface_distance_m / surface_sigma);
    normal_cost_sum +=
        contact_weight * SafeSquared(opposing_normal_error / normal_sigma);
    surface_distance_sum += contact_weight * surface_distance_m;
    normal_error_sum += contact_weight * opposing_normal_error;
    weight_sum += contact_weight;
  }

  if (weight_sum <= kTiny) {
    return score;
  }

  score.surface_cost = surface_cost_sum / weight_sum;
  score.normal_cost = normal_cost_sum / weight_sum;
  score.mean_surface_distance_m = surface_distance_sum / weight_sum;
  score.mean_normal_alignment_error = normal_error_sum / weight_sum;

  const double position_error_m =
      (particle_pose_world.translation() -
       prior.initial_pose_world.translation()).norm();
  const double rotation_error_rad = RotationDistanceRad(
      prior.initial_pose_world.linear(), particle_pose_world.linear());
  score.prior_cost =
      SafeSquared(position_error_m / prior_position_sigma) +
      SafeSquared(rotation_error_rad / prior_rotation_sigma);
  ScoreContactWidth(
      prior, particle_pose_world, contact_width, config, &score);

  score.total_cost =
      std::max(0.0, config.w_surface) * score.surface_cost +
      std::max(0.0, config.w_normal) * score.normal_cost +
      std::max(0.0, config.w_prior) * score.prior_cost +
      std::max(0.0, config.w_contact_width) * score.contact_width_cost;
  return score;
}

bool HasFiniteCost(const ObjectParticleScore& score) {
  return std::isfinite(score.total_cost);
}

void NormalizeParticleWeights(
    const std::vector<ObjectParticleScore>& scores,
    VirtualObjectBelief* belief) {
  if (belief == nullptr || belief->particles.size() != scores.size()) {
    return;
  }

  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto& score : scores) {
    if (HasFiniteCost(score)) {
      best_cost = std::min(best_cost, score.total_cost);
    }
  }
  if (!std::isfinite(best_cost)) {
    return;
  }

  double weight_sum = 0.0;
  for (std::size_t i = 0; i < belief->particles.size(); ++i) {
    auto& particle = belief->particles[i];
    if (!HasFiniteCost(scores[i])) {
      particle.weight = 0.0;
      continue;
    }
    particle.weight = std::exp(-0.5 * (scores[i].total_cost - best_cost));
    weight_sum += particle.weight;
  }
  if (weight_sum <= kTiny || !std::isfinite(weight_sum)) {
    return;
  }
  for (auto& particle : belief->particles) {
    particle.weight /= weight_sum;
  }
}

std::size_t BestParticleIndex(const std::vector<ObjectParticleScore>& scores) {
  std::size_t best_index = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scores.size(); ++i) {
    if (scores[i].total_cost < best_cost) {
      best_cost = scores[i].total_cost;
      best_index = i;
    }
  }
  return best_index;
}

bool RepresentativeObjectBeliefPose(const VirtualObjectBelief& belief,
                                    Eigen::Isometry3d* pose_world) {
  if (pose_world == nullptr || !HasVirtualObjectBelief(belief) ||
      !IsValidVirtualObjectBelief(belief) || belief.particles.empty()) {
    return false;
  }

  double weight_sum = 0.0;
  Eigen::Vector3d weighted_translation = Eigen::Vector3d::Zero();
  const VirtualObjectState* orientation_source = nullptr;
  double orientation_weight = -1.0;
  for (const auto& particle : belief.particles) {
    if (!IsValidVirtualObjectState(particle) ||
        !particle.pose_world.matrix().allFinite()) {
      continue;
    }
    const double weight =
        std::isfinite(particle.weight) && particle.weight > 0.0
            ? particle.weight
            : 1.0;
    weighted_translation += weight * particle.pose_world.translation();
    weight_sum += weight;
    if (weight > orientation_weight) {
      orientation_weight = weight;
      orientation_source = &particle;
    }
  }
  if (orientation_source == nullptr || weight_sum <= kTiny) {
    return false;
  }

  *pose_world = orientation_source->pose_world;
  pose_world->translation() = weighted_translation / weight_sum;
  return pose_world->matrix().allFinite();
}

ObjectBeliefInitializationResult ReweightExistingBeliefFromContacts(
    const ObjectPrior& prior,
    const VirtualObjectBelief& previous_belief,
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts,
    const ObjectBeliefInitializationConfig& config) {
  ObjectBeliefInitializationResult result;
  if (!IsValidObjectPrior(prior) ||
      !HasVirtualObjectBelief(previous_belief) ||
      !IsValidVirtualObjectBelief(previous_belief) ||
      previous_belief.particles.empty()) {
    return result;
  }

  result.contacts =
      ExtractObjectContactObservations(q_meas, tactile_sensors, tactile_contexts);
  double total_contact_force_n = 0.0;
  for (const auto& contact : result.contacts) {
    if (std::isfinite(contact.normal_force_n)) {
      total_contact_force_n += std::max(0.0, contact.normal_force_n);
    }
  }
  const double force_threshold_n =
      std::max(0.0, config.contact_force_threshold_n);
  if (result.contacts.size() < config.min_contact_count ||
      total_contact_force_n < force_threshold_n) {
    return result;
  }

  ObjectPrior scoring_prior = prior;
  if (IsValidObjectGeometry(previous_belief.geometry)) {
    scoring_prior.geometry = previous_belief.geometry;
  }

  result.belief = previous_belief;
  result.belief.geometry = scoring_prior.geometry;
  result.particle_scores.reserve(result.belief.particles.size());
  const ContactWidthObservation contact_width =
      MakeContactWidthObservation(result.contacts, config);

  for (const auto& particle : result.belief.particles) {
    result.particle_scores.push_back(
        ScoreParticle(scoring_prior, particle.pose_world, result.contacts,
                      contact_width, config));
  }

  NormalizeParticleWeights(result.particle_scores, &result.belief);
  result.best_particle_index = BestParticleIndex(result.particle_scores);
  if (result.best_particle_index < result.particle_scores.size()) {
    const auto& best_score = result.particle_scores[result.best_particle_index];
    result.best_cost = best_score.total_cost;
    result.best_surface_distance_m = best_score.mean_surface_distance_m;
    result.best_normal_alignment_error =
        best_score.mean_normal_alignment_error;
  }

  result.belief.valid = true;
  result.valid = std::isfinite(result.best_cost) &&
                 IsValidVirtualObjectBelief(result.belief);
  if (!result.valid) {
    result.belief = VirtualObjectBelief{};
  }
  return result;
}

}  // namespace

std::vector<ObjectContactObservation,
            Eigen::aligned_allocator<ObjectContactObservation>>
ExtractObjectContactObservations(
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts) {
  std::vector<ObjectContactObservation,
              Eigen::aligned_allocator<ObjectContactObservation>>
      contacts;

  if (q_meas.size() == 0 || !q_meas.allFinite() ||
      tactile_sensors.size() != tactile_contexts.size()) {
    return contacts;
  }

  for (std::size_t sensor_i = 0; sensor_i < tactile_sensors.size(); ++sensor_i) {
    const auto& tactile = tactile_sensors[sensor_i];
    const auto& sensor_context = tactile_contexts[sensor_i];
    const auto* kinematics = sensor_context.kinematics;
    if (!tactile.valid || kinematics == nullptr ||
        !IsValidContactKinematicsContext(*kinematics) ||
        q_meas.size() != static_cast<Eigen::Index>(kinematics->model->nq)) {
      continue;
    }

    pinocchio::Data data(*kinematics->model);
    pinocchio::forwardKinematics(*kinematics->model, data, q_meas);
    pinocchio::updateFramePlacements(*kinematics->model, data);
    const auto& sensor_pose_world = data.oMf[kinematics->sensor_frame_id];
    const Eigen::Matrix3d rotation_world_sensor =
        sensor_pose_world.rotation();

    for (std::size_t hemi_i = 0; hemi_i < tactile.hemispheres.size(); ++hemi_i) {
      const auto& hemisphere = tactile.hemispheres[hemi_i];
      if (!hemisphere.contact || !hemisphere.cop_sensor_m.allFinite()) {
        continue;
      }

      Eigen::Vector3d point_sensor_m{hemisphere.cop_sensor_m.x(),
                                     hemisphere.cop_sensor_m.y(), 0.0};
      Eigen::Vector3d normal_sensor = Eigen::Vector3d::UnitZ();
      if (sensor_context.hemispheres.size() == tactile.hemispheres.size()) {
        const auto& geometry = sensor_context.hemispheres[hemi_i];
        if (geometry.hemisphere_index != hemisphere.hemisphere_index ||
            !geometry.normal_sensor.allFinite() ||
            geometry.normal_sensor.norm() <= kTiny) {
          return {};
        }
        point_sensor_m = HemisphereLocalPointSensorM(hemisphere, geometry);
        normal_sensor = geometry.normal_sensor.normalized();
      }
      if (!point_sensor_m.allFinite()) {
        continue;
      }

      ObjectContactObservation contact;
      contact.sensor_index = tactile.sensor_index >= 0
                                 ? static_cast<std::size_t>(tactile.sensor_index)
                                 : sensor_i;
      contact.hemisphere_index = hemisphere.hemisphere_index;
      contact.point_world_m = sensor_pose_world.act(point_sensor_m);
      contact.normal_world = rotation_world_sensor * normal_sensor;
      if (std::isfinite(kinematics->normal_axis_sign) &&
          kinematics->normal_axis_sign < 0.0) {
        contact.normal_world = -contact.normal_world;
      }
      contact.normal_force_n =
          std::isfinite(hemisphere.normal_force_n)
              ? std::max(0.0, hemisphere.normal_force_n)
              : 0.0;
      contact.confidence =
          std::isfinite(hemisphere.confidence)
              ? std::clamp(hemisphere.confidence, 0.0, 1.0)
              : 0.0;
      if (contact.point_world_m.allFinite() && contact.normal_world.allFinite() &&
          contact.normal_world.norm() > kTiny) {
        contacts.push_back(std::move(contact));
      }
    }
  }

  return contacts;
}

ObjectBeliefInitializationResult InitializeObjectBeliefFromContacts(
    const ObjectPrior& prior,
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts,
    const ObjectBeliefInitializationConfig& config) {
  ObjectBeliefInitializationResult result;
  if (!IsValidObjectPrior(prior) || config.particle_count == 0) {
    return result;
  }

  result.contacts =
      ExtractObjectContactObservations(q_meas, tactile_sensors, tactile_contexts);
  double total_contact_force_n = 0.0;
  for (const auto& contact : result.contacts) {
    if (std::isfinite(contact.normal_force_n)) {
      total_contact_force_n += std::max(0.0, contact.normal_force_n);
    }
  }
  const double force_threshold_n =
      std::max(0.0, config.contact_force_threshold_n);
  if (result.contacts.size() < config.min_contact_count ||
      total_contact_force_n < force_threshold_n) {
    return result;
  }

  const Eigen::Vector3d position_std_m =
      PositiveStdOrFallback(prior.position_std_m,
                            config.fallback_position_sample_std_m);
  const Eigen::Vector3d rpy_std_rad =
      PositiveStdOrFallback(prior.rpy_std_rad,
                            config.fallback_rpy_sample_std_rad);

  std::mt19937 rng(config.random_seed);
  std::normal_distribution<double> unit_normal(0.0, 1.0);
  const ContactWidthObservation contact_width =
      MakeContactWidthObservation(result.contacts, config);

  result.belief.geometry = prior.geometry;
  result.belief.particles.reserve(config.particle_count);
  result.particle_scores.reserve(config.particle_count);

  for (std::size_t i = 0; i < config.particle_count; ++i) {
    Eigen::Vector3d position_sample_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy_sample_rad = Eigen::Vector3d::Zero();
    if (i > 0) {
      for (Eigen::Index axis = 0; axis < 3; ++axis) {
        position_sample_m[axis] = position_std_m[axis] * unit_normal(rng);
        rpy_sample_rad[axis] = rpy_std_rad[axis] * unit_normal(rng);
      }
    }

    VirtualObjectState particle;
    particle.pose_world =
        SamplePoseAroundPrior(prior, position_sample_m, rpy_sample_rad);
    particle.velocity_world.setZero();
    particle.weight = 0.0;
    particle.valid = particle.pose_world.matrix().allFinite();

    result.particle_scores.push_back(
        ScoreParticle(
            prior, particle.pose_world, result.contacts, contact_width,
            config));
    result.belief.particles.push_back(std::move(particle));
  }

  NormalizeParticleWeights(result.particle_scores, &result.belief);
  result.best_particle_index = BestParticleIndex(result.particle_scores);
  if (result.best_particle_index < result.particle_scores.size()) {
    const auto& best_score = result.particle_scores[result.best_particle_index];
    result.best_cost = best_score.total_cost;
    result.best_surface_distance_m = best_score.mean_surface_distance_m;
    result.best_normal_alignment_error =
        best_score.mean_normal_alignment_error;
  }

  result.belief.valid = true;
  result.valid = std::isfinite(result.best_cost) &&
                 IsValidVirtualObjectBelief(result.belief);
  if (!result.valid) {
    result.belief = VirtualObjectBelief{};
  }
  return result;
}

ObjectBeliefInitializationResult UpdateObjectBeliefFromCurrentContacts(
    const ObjectPrior& prior,
    const VirtualObjectBelief& previous_belief,
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts,
    const ObjectBeliefInitializationConfig& config) {
  if (!IsValidObjectPrior(prior)) {
    return ObjectBeliefInitializationResult{};
  }

  ObjectPrior tracking_prior = prior;
  Eigen::Isometry3d previous_pose_world = Eigen::Isometry3d::Identity();
  if (RepresentativeObjectBeliefPose(previous_belief, &previous_pose_world)) {
    tracking_prior.initial_pose_world = previous_pose_world;
  }

  auto reweighted = ReweightExistingBeliefFromContacts(
      tracking_prior, previous_belief, q_meas, tactile_sensors,
      tactile_contexts, config);
  if (reweighted.valid) {
    return reweighted;
  }

  return InitializeObjectBeliefFromContacts(
      tracking_prior, q_meas, tactile_sensors, tactile_contexts, config);
}

ObjectBeliefInitializationResult InitializeObjectBeliefFromObservation(
    const GraspObservation& observation,
    const ObjectBeliefInitializationConfig& config) {
  return InitializeObjectBeliefFromContacts(
      observation.object_prior, observation.q_meas, observation.tactile_meas,
      observation.tactile_contexts, config);
}

VirtualObjectBelief ResolveObjectBeliefForObservation(
    const GraspObservation& observation,
    const Eigen::Ref<const Eigen::VectorXd>& q_rollout_root,
    const ObjectBeliefInitializationConfig& config) {
  if (!HasObjectPrior(observation.object_prior)) {
    return HasVirtualObjectBelief(observation.object_belief)
               ? observation.object_belief
               : VirtualObjectBelief{};
  }

  const auto result = UpdateObjectBeliefFromCurrentContacts(
      observation.object_prior, observation.object_belief, q_rollout_root,
      observation.tactile_meas, observation.tactile_contexts, config);
  if (result.valid) {
    return result.belief;
  }
  return HasVirtualObjectBelief(observation.object_belief)
             ? observation.object_belief
             : VirtualObjectBelief{};
}

}  // namespace mppi_core
