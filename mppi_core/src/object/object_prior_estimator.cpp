// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/object/object_prior_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mppi_core/object/virtual_object_state.hpp"

namespace mppi_core {
namespace {

constexpr double kTinyWeight = 1.0e-12;

bool IsUsableObjectPriorEstimatorConfig(
    const ObjectPrior& prior, const ObjectPriorEstimatorConfig& config) {
  return HasObjectPrior(prior) && IsValidObjectPrior(prior) &&
         config.belief.particle_count > 0;
}

bool IsUsableBelief(const VirtualObjectBelief& belief) {
  return HasVirtualObjectBelief(belief) &&
         IsValidVirtualObjectBelief(belief);
}

double PositivePrimitiveMinimumExtentM(const ObjectGeometryHandle& geometry) {
  if (!IsValidObjectGeometry(geometry) ||
      !geometry.primitive_size_m.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  if (geometry.type == ObjectGeometryType::kSphere) {
    return geometry.primitive_size_m.x() > 0.0
               ? 2.0 * geometry.primitive_size_m.x()
               : std::numeric_limits<double>::infinity();
  }
  if (geometry.type == ObjectGeometryType::kCylinder) {
    const double diameter = 2.0 * geometry.primitive_size_m.x();
    const double height = geometry.primitive_size_m.z();
    if (diameter > 0.0 && height > 0.0) {
      return std::min(diameter, height);
    }
    return std::numeric_limits<double>::infinity();
  }

  double min_extent_m = std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < 3; ++i) {
    const double extent_m = geometry.primitive_size_m[i];
    if (std::isfinite(extent_m) && extent_m > 0.0) {
      min_extent_m = std::min(min_extent_m, extent_m);
    }
  }
  return min_extent_m;
}

struct SensorContactAggregate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t sensor_index{0};
  Eigen::Vector3d weighted_point_world_m{Eigen::Vector3d::Zero()};
  double weight_sum{0.0};

  Eigen::Vector3d pointWorld() const {
    if (weight_sum > kTinyWeight) {
      return weighted_point_world_m / weight_sum;
    }
    return Eigen::Vector3d::Zero();
  }
};

double ContactAggregateWeight(const ObjectContactObservation& contact) {
  double weight = std::isfinite(contact.confidence)
                      ? std::clamp(contact.confidence, 0.0, 1.0)
                      : 0.0;
  if (std::isfinite(contact.normal_force_n) && contact.normal_force_n > 0.0) {
    weight *= std::max(1.0, contact.normal_force_n);
  }
  return weight;
}

bool ComputeTwoSensorContactGapM(
    const std::vector<ObjectContactObservation,
                      Eigen::aligned_allocator<ObjectContactObservation>>&
        contacts,
    double* gap_m) {
  if (gap_m == nullptr) {
    return false;
  }

  std::vector<SensorContactAggregate,
              Eigen::aligned_allocator<SensorContactAggregate>>
      aggregates;
  for (const auto& contact : contacts) {
    if (!contact.point_world_m.allFinite()) {
      continue;
    }
    const double weight = ContactAggregateWeight(contact);
    if (weight <= kTinyWeight) {
      continue;
    }

    auto found = std::find_if(
        aggregates.begin(), aggregates.end(),
        [&contact](const SensorContactAggregate& aggregate) {
          return aggregate.sensor_index == contact.sensor_index;
        });
    if (found == aggregates.end()) {
      SensorContactAggregate aggregate;
      aggregate.sensor_index = contact.sensor_index;
      aggregates.push_back(aggregate);
      found = aggregates.end() - 1;
    }
    found->weighted_point_world_m += weight * contact.point_world_m;
    found->weight_sum += weight;
  }

  aggregates.erase(
      std::remove_if(
          aggregates.begin(), aggregates.end(),
          [](const SensorContactAggregate& aggregate) {
            return aggregate.weight_sum <= kTinyWeight;
          }),
      aggregates.end());
  if (aggregates.size() < 2) {
    return false;
  }

  std::sort(
      aggregates.begin(), aggregates.end(),
      [](const SensorContactAggregate& lhs,
         const SensorContactAggregate& rhs) {
        return lhs.weight_sum > rhs.weight_sum;
      });

  *gap_m = (aggregates[1].pointWorld() - aggregates[0].pointWorld()).norm();
  return std::isfinite(*gap_m);
}

bool ShouldRejectCloseSensorGap(
    const ObjectPrior& prior, const ObjectPriorEstimatorConfig& config,
    const std::vector<ObjectContactObservation,
                      Eigen::aligned_allocator<ObjectContactObservation>>&
        contacts,
    double* gap_m, double* min_gap_m, double* object_min_extent_m) {
  if (!config.reject_close_sensor_gap_as_fingertip_touch) {
    return false;
  }
  const double object_extent_m =
      PositivePrimitiveMinimumExtentM(prior.geometry);
  if (object_min_extent_m != nullptr) {
    *object_min_extent_m = object_extent_m;
  }
  if (!std::isfinite(object_extent_m) || object_extent_m <= 0.0) {
    return false;
  }

  const double scale =
      std::isfinite(config.sensor_gap_min_object_extent_scale)
          ? std::max(0.0, config.sensor_gap_min_object_extent_scale)
          : 0.0;
  const double margin_m =
      std::isfinite(config.sensor_gap_min_margin_m)
          ? std::max(0.0, config.sensor_gap_min_margin_m)
          : 0.0;
  const double required_gap_m =
      std::max(0.0, scale * object_extent_m - margin_m);
  if (min_gap_m != nullptr) {
    *min_gap_m = required_gap_m;
  }
  if (required_gap_m <= 0.0) {
    return false;
  }

  double measured_gap_m = std::numeric_limits<double>::infinity();
  if (!ComputeTwoSensorContactGapM(contacts, &measured_gap_m)) {
    return false;
  }
  if (gap_m != nullptr) {
    *gap_m = measured_gap_m;
  }
  return measured_gap_m < required_gap_m;
}

}  // namespace

ObjectPriorEstimator::ObjectPriorEstimator(
    const ObjectPrior& prior,
    const ObjectBeliefInitializationConfig& config) {
  Configure(prior, config);
}

ObjectPriorEstimator::ObjectPriorEstimator(
    const ObjectPrior& prior, const ObjectPriorEstimatorConfig& config) {
  Configure(prior, config);
}

bool ObjectPriorEstimator::Configure(
    const ObjectPrior& prior,
    const ObjectBeliefInitializationConfig& config) {
  ObjectPriorEstimatorConfig estimator_config;
  estimator_config.belief = config;
  return Configure(prior, estimator_config);
}

bool ObjectPriorEstimator::Configure(
    const ObjectPrior& prior, const ObjectPriorEstimatorConfig& config) {
  prior_ = prior;
  config_ = config;
  belief_ = VirtualObjectBelief{};
  configured_ = IsUsableObjectPriorEstimatorConfig(prior_, config_);
  status_ = ObjectPriorEstimatorStatus{};
  status_.configured = configured_;
  return configured_;
}

void ObjectPriorEstimator::Reset() {
  belief_ = VirtualObjectBelief{};
  status_ = ObjectPriorEstimatorStatus{};
  status_.configured = configured_;
}

bool ObjectPriorEstimator::has_belief() const {
  return IsUsableBelief(belief_);
}

ObjectBeliefInitializationResult ObjectPriorEstimator::Update(
    const Eigen::Ref<const Eigen::VectorXd>& q_meas,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const std::vector<TactileSensorContext>& tactile_contexts) {
  const std::size_t previous_update_count = status_.update_count;
  status_ = ObjectPriorEstimatorStatus{};
  status_.configured = configured_;
  status_.update_count = previous_update_count;

  if (!configured_) {
    belief_ = VirtualObjectBelief{};
    return ObjectBeliefInitializationResult{};
  }

  ObjectBeliefInitializationResult precheck_result;
  precheck_result.contacts =
      ExtractObjectContactObservations(q_meas, tactile_sensors, tactile_contexts);
  status_.contact_count = precheck_result.contacts.size();
  status_.object_min_extent_m =
      PositivePrimitiveMinimumExtentM(prior_.geometry);
  double measured_gap_m = std::numeric_limits<double>::infinity();
  double min_gap_m = 0.0;
  double object_min_extent_m = status_.object_min_extent_m;
  if (ShouldRejectCloseSensorGap(prior_, config_, precheck_result.contacts,
                                 &measured_gap_m, &min_gap_m,
                                 &object_min_extent_m)) {
    status_.close_sensor_gap_rejected = true;
    status_.sensor_gap_m = measured_gap_m;
    status_.min_sensor_gap_m = min_gap_m;
    status_.object_min_extent_m = object_min_extent_m;
    belief_ = VirtualObjectBelief{};
    status_.belief_valid = false;
    status_.particle_count = 0;
    return precheck_result;
  }
  status_.sensor_gap_m = measured_gap_m;
  status_.min_sensor_gap_m = min_gap_m;
  status_.object_min_extent_m = object_min_extent_m;

  const auto result = UpdateObjectBeliefFromCurrentContacts(
      prior_, belief_, q_meas, tactile_sensors, tactile_contexts,
      config_.belief);

  status_.contact_count = result.contacts.size();
  status_.best_particle_index = result.best_particle_index;
  status_.best_cost = result.best_cost;
  status_.best_surface_distance_m = result.best_surface_distance_m;
  status_.best_normal_alignment_error =
      result.best_normal_alignment_error;

  if (result.valid) {
    belief_ = result.belief;
    status_.updated = true;
    ++status_.update_count;
  } else if (!has_belief()) {
    belief_ = VirtualObjectBelief{};
  }

  status_.belief_valid = has_belief();
  status_.particle_count =
      status_.belief_valid ? belief_.particleCount() : std::size_t{0};
  return result;
}

bool ObjectPriorEstimator::representativePose(
    Eigen::Isometry3d* pose_world) const {
  return RepresentativePose(belief_, pose_world);
}

bool ObjectPriorEstimator::RepresentativePose(
    const VirtualObjectBelief& belief,
    Eigen::Isometry3d* pose_world) {
  if (pose_world == nullptr || !IsUsableBelief(belief) ||
      belief.particles.empty()) {
    return false;
  }

  double weight_sum = 0.0;
  Eigen::Vector3d weighted_translation_m = Eigen::Vector3d::Zero();
  const VirtualObjectState* orientation_source = nullptr;
  double best_weight = -1.0;

  for (const auto& particle : belief.particles) {
    if (!IsValidVirtualObjectState(particle) ||
        !particle.pose_world.matrix().allFinite()) {
      continue;
    }
    const double weight =
        std::isfinite(particle.weight) && particle.weight > 0.0
            ? particle.weight
            : 1.0;
    weighted_translation_m.noalias() +=
        weight * particle.pose_world.translation();
    weight_sum += weight;
    if (weight > best_weight) {
      best_weight = weight;
      orientation_source = &particle;
    }
  }

  if (orientation_source == nullptr || weight_sum <= kTinyWeight) {
    return false;
  }

  *pose_world = orientation_source->pose_world;
  pose_world->translation() = weighted_translation_m / weight_sum;
  return pose_world->matrix().allFinite();
}

}  // namespace mppi_core
