// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/object/object_contact_support_evaluator.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <Eigen/StdVector>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/object/object_geometry_query.hpp"

namespace mppi_core {
namespace {

constexpr double kTiny = 1.0e-12;

struct CandidateHemispherePoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t sensor_vector_index{0};
  std::size_t sensor_index{0};
  std::size_t hemisphere_index{0};

  bool measured_contact{false};
  double measured_normal_force_n{0.0};

  Eigen::Vector2d point_sensor_xy_m{Eigen::Vector2d::Zero()};
  Eigen::Vector3d point_world_m{Eigen::Vector3d::Zero()};
  double radius_m{0.0};
};

struct SensorGridBounds {
  bool valid{false};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
};

struct SensorContactAccumulator {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t count{0};
  Eigen::Vector2d centroid_sum{Eigen::Vector2d::Zero()};
  bool bounds_valid{false};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
};

struct PinocchioPlacementCache {
  const pinocchio::Model* model{nullptr};
  std::unique_ptr<pinocchio::Data> data;
};

bool IsNonnegativeFinite(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

double Square(const double value) { return value * value; }

double HingePositive(const double value) {
  return std::isfinite(value) ? std::max(0.0, value) : 0.0;
}

double SafeWeight(const VirtualObjectState& particle,
                  const ObjectContactSupportEvaluatorConfig& config) {
  if (!config.use_particle_weights) {
    return 1.0;
  }
  if (!std::isfinite(particle.weight) || particle.weight <= 0.0) {
    return 0.0;
  }
  return particle.weight;
}

bool HasValidConfig(const ObjectContactSupportEvaluatorConfig& config) {
  return config.max_object_samples > 0 &&
         config.min_active_tactile_sensors > 0 &&
         config.min_active_hemisphere_total > 0 &&
         IsNonnegativeFinite(config.contact_birth_margin_m) &&
         IsNonnegativeFinite(config.contact_loss_margin_m) &&
         config.contact_loss_margin_m >= config.contact_birth_margin_m &&
         IsNonnegativeFinite(config.contact_stiffness_n_per_m) &&
         IsNonnegativeFinite(config.hemisphere_radius_m) &&
         IsNonnegativeFinite(config.support_distance_scale_m) &&
         config.support_distance_scale_m > 0.0 &&
         IsNonnegativeFinite(config.max_allowed_penetration_m) &&
         IsNonnegativeFinite(config.target_predicted_normal_force_n) &&
         IsNonnegativeFinite(config.max_predicted_normal_force_n) &&
         IsNonnegativeFinite(config.min_predicted_contact_force_n) &&
         IsNonnegativeFinite(config.max_predicted_force_per_sensor_n) &&
         IsNonnegativeFinite(config.contact_loss_weight) &&
         IsNonnegativeFinite(config.support_weight) &&
         IsNonnegativeFinite(config.edge_weight) &&
         IsNonnegativeFinite(config.penetration_weight) &&
         IsNonnegativeFinite(config.predicted_force_low_weight) &&
         IsNonnegativeFinite(config.predicted_force_high_weight) &&
         IsNonnegativeFinite(config.target_edge_margin_m);
}

const HemisphereState* FindHemisphereState(
    const TactileState& tactile, const std::size_t hemisphere_index) {
  for (const auto& hemisphere : tactile.hemispheres) {
    if (hemisphere.hemisphere_index == hemisphere_index) {
      return &hemisphere;
    }
  }
  return nullptr;
}

pinocchio::Data* GetFramePlacementData(
    const pinocchio::Model& model,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    std::vector<PinocchioPlacementCache>* caches) {
  if (caches == nullptr ||
      q.size() != static_cast<Eigen::Index>(model.nq) || !q.allFinite()) {
    return nullptr;
  }
  for (auto& cache : *caches) {
    if (cache.model == &model) {
      return cache.data.get();
    }
  }

  PinocchioPlacementCache cache;
  cache.model = &model;
  cache.data = std::make_unique<pinocchio::Data>(model);
  pinocchio::forwardKinematics(model, *cache.data, q);
  pinocchio::updateFramePlacements(model, *cache.data);
  caches->push_back(std::move(cache));
  return caches->back().data.get();
}

bool AddPointFromGeometry(
    const GraspState& state,
    const TactileSensorContext& sensor_context,
    const std::size_t sensor_vector_index,
    const HemisphereGeometry& geometry,
    const HemisphereState* measured,
    std::vector<PinocchioPlacementCache>* caches,
    std::vector<CandidateHemispherePoint,
                Eigen::aligned_allocator<CandidateHemispherePoint>>* points) {
  if (points == nullptr || caches == nullptr ||
      sensor_context.kinematics == nullptr ||
      !IsValidContactKinematicsContext(*sensor_context.kinematics) ||
      !geometry.center_sensor_m.allFinite()) {
    return false;
  }

  const auto& kinematics = *sensor_context.kinematics;
  const auto& model = *kinematics.model;
  pinocchio::Data* data =
      GetFramePlacementData(model, state.robot.q, caches);
  if (data == nullptr || kinematics.sensor_frame_id >= model.frames.size()) {
    return false;
  }

  const Eigen::Vector3d point_sensor_m =
      measured != nullptr ? HemisphereLocalPointSensorM(*measured, geometry)
                          : geometry.center_sensor_m;
  if (!point_sensor_m.allFinite()) {
    return false;
  }

  CandidateHemispherePoint point;
  point.sensor_vector_index = sensor_vector_index;
  point.sensor_index = sensor_context.sensor_index >= 0
                           ? static_cast<std::size_t>(
                                 sensor_context.sensor_index)
                           : sensor_vector_index;
  point.hemisphere_index = geometry.hemisphere_index;
  point.measured_contact = measured != nullptr && measured->contact;
  point.measured_normal_force_n =
      measured != nullptr && std::isfinite(measured->normal_force_n)
          ? std::max(0.0, measured->normal_force_n)
          : 0.0;
  point.point_sensor_xy_m = point_sensor_m.head<2>();
  point.point_world_m =
      data->oMf[kinematics.sensor_frame_id].act(point_sensor_m);
  point.radius_m = std::isfinite(geometry.radius_m) && geometry.radius_m > 0.0
                       ? geometry.radius_m
                       : 0.0;
  if (!point.point_world_m.allFinite()) {
    return false;
  }
  points->push_back(std::move(point));
  return true;
}

std::vector<CandidateHemispherePoint,
            Eigen::aligned_allocator<CandidateHemispherePoint>>
ComputeCandidateHemispherePoints(const GraspState& state,
                                 const RolloutContext& context) {
  std::vector<CandidateHemispherePoint,
              Eigen::aligned_allocator<CandidateHemispherePoint>>
      points;
  if (!state.valid || !IsValid(state.robot) ||
      state.tactile_sensors.size() != context.tactile_contexts.size()) {
    return points;
  }

  std::vector<PinocchioPlacementCache> caches;
  for (std::size_t sensor_i = 0; sensor_i < state.tactile_sensors.size();
       ++sensor_i) {
    const auto& tactile = state.tactile_sensors[sensor_i];
    const auto& sensor_context = context.tactile_contexts[sensor_i];
    if (!tactile.valid || sensor_context.kinematics == nullptr) {
      continue;
    }

    if (!sensor_context.hemispheres.empty()) {
      for (const auto& geometry : sensor_context.hemispheres) {
        AddPointFromGeometry(
            state, sensor_context, sensor_i, geometry,
            FindHemisphereState(tactile, geometry.hemisphere_index),
            &caches, &points);
      }
      continue;
    }

    for (const auto& hemisphere : tactile.hemispheres) {
      HemisphereGeometry geometry;
      geometry.hemisphere_index = hemisphere.hemisphere_index;
      geometry.center_sensor_m = Eigen::Vector3d{
          hemisphere.cop_sensor_m.x(), hemisphere.cop_sensor_m.y(), 0.0};
      geometry.normal_sensor = Eigen::Vector3d::UnitZ();
      AddPointFromGeometry(
          state, sensor_context, sensor_i, geometry, &hemisphere,
          &caches, &points);
    }
  }
  return points;
}

std::vector<SensorGridBounds> ComputeSensorGridBounds(
    const std::vector<CandidateHemispherePoint,
                      Eigen::aligned_allocator<CandidateHemispherePoint>>&
        points,
    const std::size_t sensor_count) {
  std::vector<SensorGridBounds> bounds(sensor_count);
  for (const auto& point : points) {
    if (point.sensor_vector_index >= bounds.size() ||
        !point.point_sensor_xy_m.allFinite()) {
      continue;
    }
    auto& sensor_bounds = bounds[point.sensor_vector_index];
    if (!sensor_bounds.valid) {
      sensor_bounds.min_x = sensor_bounds.max_x = point.point_sensor_xy_m.x();
      sensor_bounds.min_y = sensor_bounds.max_y = point.point_sensor_xy_m.y();
      sensor_bounds.valid = true;
      continue;
    }
    sensor_bounds.min_x = std::min(sensor_bounds.min_x,
                                   point.point_sensor_xy_m.x());
    sensor_bounds.max_x = std::max(sensor_bounds.max_x,
                                   point.point_sensor_xy_m.x());
    sensor_bounds.min_y = std::min(sensor_bounds.min_y,
                                   point.point_sensor_xy_m.y());
    sensor_bounds.max_y = std::max(sensor_bounds.max_y,
                                   point.point_sensor_xy_m.y());
  }
  for (auto& sensor_bounds : bounds) {
    if (sensor_bounds.valid &&
        (sensor_bounds.max_x - sensor_bounds.min_x <= kTiny ||
         sensor_bounds.max_y - sensor_bounds.min_y <= kTiny)) {
      sensor_bounds.valid = false;
    }
  }
  return bounds;
}

double ComputePredictedForce(
    const double gap_m,
    const ObjectContactSupportEvaluatorConfig& config) {
  if (!std::isfinite(gap_m) || config.contact_stiffness_n_per_m <= 0.0) {
    return 0.0;
  }
  const double compression_m =
      config.contact_birth_margin_m - gap_m;
  if (compression_m <= 0.0) {
    return 0.0;
  }
  return std::min(config.max_predicted_normal_force_n,
                  config.contact_stiffness_n_per_m * compression_m);
}

double SensorEdgeMargin(
    const SensorGridBounds& bounds,
    const Eigen::Vector2d& centroid_sensor_m) {
  if (!bounds.valid || !centroid_sensor_m.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::min({
      centroid_sensor_m.x() - bounds.min_x,
      bounds.max_x - centroid_sensor_m.x(),
      centroid_sensor_m.y() - bounds.min_y,
      bounds.max_y - centroid_sensor_m.y()});
}

void AddSensorPoint(SensorContactAccumulator* accumulator,
                    const Eigen::Vector2d& point_sensor_m) {
  if (accumulator == nullptr || !point_sensor_m.allFinite()) {
    return;
  }
  accumulator->centroid_sum += point_sensor_m;
  if (!accumulator->bounds_valid) {
    accumulator->min_x = accumulator->max_x = point_sensor_m.x();
    accumulator->min_y = accumulator->max_y = point_sensor_m.y();
    accumulator->bounds_valid = true;
  } else {
    accumulator->min_x = std::min(accumulator->min_x, point_sensor_m.x());
    accumulator->max_x = std::max(accumulator->max_x, point_sensor_m.x());
    accumulator->min_y = std::min(accumulator->min_y, point_sensor_m.y());
    accumulator->max_y = std::max(accumulator->max_y, point_sensor_m.y());
  }
  ++accumulator->count;
}

double SensorSupportArea(const SensorContactAccumulator& accumulator) {
  if (!accumulator.bounds_valid || accumulator.count < 2U) {
    return 0.0;
  }
  return std::max(0.0, accumulator.max_x - accumulator.min_x) *
         std::max(0.0, accumulator.max_y - accumulator.min_y);
}

Eigen::Vector2d CentroidOrNan(
    const std::vector<SensorContactAccumulator>& accumulators) {
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  std::size_t count = 0;
  for (const auto& accumulator : accumulators) {
    if (accumulator.count == 0U) {
      continue;
    }
    sum += accumulator.centroid_sum;
    count += accumulator.count;
  }
  if (count == 0U) {
    return Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
  }
  return sum / static_cast<double>(count);
}

bool FiniteVector(const Eigen::Vector2d& value) {
  return value.allFinite();
}

void AccumulateWeightedVector(Eigen::Vector2d* total,
                              const Eigen::Vector2d& sample,
                              const double weight) {
  if (total == nullptr || !FiniteVector(sample) || weight <= 0.0 ||
      !std::isfinite(weight)) {
    return;
  }
  if (!total->allFinite()) {
    *total = Eigen::Vector2d::Zero();
  }
  *total += weight * sample;
}

void FinalizeSummaryFromEvaluation(ObjectContactSupportEvaluation* evaluation) {
  if (evaluation == nullptr) {
    return;
  }
  evaluation->support_summary.predicted_active_count =
      static_cast<int>(std::lround(
          evaluation->predicted_active_hemisphere_total));
  evaluation->support_summary.measured_active_count =
      static_cast<int>(std::lround(
          evaluation->measured_active_hemisphere_total));
  evaluation->support_summary.predicted_edge_margin_m =
      evaluation->min_edge_margin_m;
  evaluation->support_summary.predicted_total_force_n =
      evaluation->predicted_normal_force_total_n;
}

ObjectContactSupportEvaluation EvaluateOneObjectSample(
    const GraspState& state,
    const ObjectGeometryHandle& geometry,
    const VirtualObjectState& object,
    const std::vector<CandidateHemispherePoint,
                      Eigen::aligned_allocator<CandidateHemispherePoint>>&
        points,
    const std::vector<SensorGridBounds>& sensor_bounds,
    const ObjectContactSupportEvaluatorConfig& config) {
  ObjectContactSupportEvaluation out;
  if (!IsValidVirtualObjectState(object) || !IsValidObjectGeometry(geometry) ||
      points.empty()) {
    return out;
  }

  out.valid = true;
  out.object_sample_count = 1;
  out.min_signed_distance_m = std::numeric_limits<double>::infinity();
  out.min_edge_margin_m = std::numeric_limits<double>::infinity();
  out.measured_active_hemisphere_total =
      static_cast<double>(state.activeHemisphereCountTotal());
  out.measured_active_tactile_sensors =
      static_cast<double>(state.activeTactileSensorCount());

  std::vector<double> predicted_force_by_sensor(state.tactile_sensors.size(),
                                                0.0);
  std::vector<std::size_t> predicted_count_by_sensor(
      state.tactile_sensors.size(), 0);
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      predicted_centroid_sum(
      state.tactile_sensors.size(), Eigen::Vector2d::Zero());
  std::vector<SensorContactAccumulator> predicted_accumulators(
      state.tactile_sensors.size());
  std::vector<SensorContactAccumulator> measured_accumulators(
      state.tactile_sensors.size());

  for (const auto& point : points) {
    const auto surface = QueryObjectSurface(
        geometry, object.pose_world, point.point_world_m);
    if (!surface.valid) {
      out.valid = false;
      return {};
    }
    ++out.geometry_query_count;
    const double radius_m =
        point.radius_m > 0.0 ? point.radius_m : config.hemisphere_radius_m;
    const double gap_m = surface.signed_distance_m - radius_m;
    out.min_signed_distance_m =
        std::min(out.min_signed_distance_m, gap_m);

    const double predicted_force_n =
        ComputePredictedForce(gap_m, config);
    const bool predicted_contact =
        gap_m <= config.contact_birth_margin_m &&
        predicted_force_n >= config.min_predicted_contact_force_n;
    const bool measured_contact_lost =
        point.measured_contact &&
        gap_m > config.contact_loss_margin_m;
    const double penetration_violation_m = HingePositive(
        -gap_m - config.max_allowed_penetration_m);
    out.penetration_cost +=
        config.penetration_weight * Square(penetration_violation_m);

    if (measured_contact_lost) {
      out.lost_measured_contact_count += 1.0;
    }
    if (point.measured_contact &&
        point.sensor_vector_index < measured_accumulators.size()) {
      AddSensorPoint(&measured_accumulators[point.sensor_vector_index],
                     point.point_sensor_xy_m);
    }
    if (!predicted_contact ||
        point.sensor_vector_index >= state.tactile_sensors.size()) {
      continue;
    }

    const double support_score = std::exp(
        -std::max(0.0, gap_m) / config.support_distance_scale_m);
    out.predicted_active_hemisphere_total += support_score;
    out.predicted_normal_force_total_n += predicted_force_n;
    predicted_force_by_sensor[point.sensor_vector_index] += predicted_force_n;
    predicted_count_by_sensor[point.sensor_vector_index] += 1U;
    predicted_centroid_sum[point.sensor_vector_index] +=
        point.point_sensor_xy_m;
    AddSensorPoint(&predicted_accumulators[point.sensor_vector_index],
                   point.point_sensor_xy_m);
  }

  for (std::size_t sensor_i = 0; sensor_i < predicted_count_by_sensor.size();
       ++sensor_i) {
    if (predicted_count_by_sensor[sensor_i] == 0U) {
      continue;
    }
    out.predicted_active_tactile_sensors += 1.0;
    const Eigen::Vector2d centroid =
        predicted_centroid_sum[sensor_i] /
        static_cast<double>(predicted_count_by_sensor[sensor_i]);
    out.min_edge_margin_m = std::min(
        out.min_edge_margin_m,
        SensorEdgeMargin(sensor_bounds[sensor_i], centroid));
    const double force_excess_n = HingePositive(
        predicted_force_by_sensor[sensor_i] -
        config.max_predicted_force_per_sensor_n);
    out.predicted_force_high_cost +=
        config.predicted_force_high_weight * Square(force_excess_n);
  }

  if (!std::isfinite(out.min_signed_distance_m)) {
    out.min_signed_distance_m = 0.0;
  }
  if (!std::isfinite(out.min_edge_margin_m)) {
    out.min_edge_margin_m = config.target_edge_margin_m;
  }

  out.contact_loss_cost =
      config.contact_loss_weight * Square(out.lost_measured_contact_count);
  const double sensor_deficit = HingePositive(
      static_cast<double>(config.min_active_tactile_sensors) -
      out.predicted_active_tactile_sensors);
  const double hemisphere_deficit = HingePositive(
      static_cast<double>(config.min_active_hemisphere_total) -
      out.predicted_active_hemisphere_total);
  out.support_cost = config.support_weight *
                     (Square(sensor_deficit) + Square(hemisphere_deficit));
  const double edge_deficit =
      HingePositive(config.target_edge_margin_m - out.min_edge_margin_m);
  out.edge_cost = config.edge_weight * Square(edge_deficit);
  const double predicted_force_deficit_n = HingePositive(
      config.target_predicted_normal_force_n -
      out.predicted_normal_force_total_n);
  out.predicted_force_low_cost =
      config.predicted_force_low_weight * Square(predicted_force_deficit_n);
  out.support_summary.predicted_centroid_sensor_m =
      CentroidOrNan(predicted_accumulators);
  out.support_summary.measured_centroid_sensor_m =
      CentroidOrNan(measured_accumulators);
  out.support_summary.predicted_support_area = 0.0;
  for (const auto& accumulator : predicted_accumulators) {
    out.support_summary.predicted_support_area +=
        SensorSupportArea(accumulator);
  }
  FinalizeSummaryFromEvaluation(&out);
  return out;
}

void AccumulateWeighted(ObjectContactSupportEvaluation* total,
                        const ObjectContactSupportEvaluation& sample,
                        const double weight) {
  if (total == nullptr || !sample.valid || weight <= 0.0 ||
      !std::isfinite(weight)) {
    return;
  }
  total->valid = true;
  total->object_sample_count += sample.object_sample_count;
  total->geometry_query_count += sample.geometry_query_count;
  total->measured_active_hemisphere_total +=
      weight * sample.measured_active_hemisphere_total;
  total->measured_active_tactile_sensors +=
      weight * sample.measured_active_tactile_sensors;
  total->predicted_active_hemisphere_total +=
      weight * sample.predicted_active_hemisphere_total;
  total->predicted_active_tactile_sensors +=
      weight * sample.predicted_active_tactile_sensors;
  total->lost_measured_contact_count +=
      weight * sample.lost_measured_contact_count;
  total->predicted_normal_force_total_n +=
      weight * sample.predicted_normal_force_total_n;
  total->contact_loss_cost += weight * sample.contact_loss_cost;
  total->support_cost += weight * sample.support_cost;
  total->edge_cost += weight * sample.edge_cost;
  total->penetration_cost += weight * sample.penetration_cost;
  total->predicted_force_low_cost +=
      weight * sample.predicted_force_low_cost;
  total->predicted_force_high_cost +=
      weight * sample.predicted_force_high_cost;
  total->support_summary.predicted_support_area +=
      weight * sample.support_summary.predicted_support_area;
  AccumulateWeightedVector(
      &total->support_summary.predicted_centroid_sensor_m,
      sample.support_summary.predicted_centroid_sensor_m, weight);
  AccumulateWeightedVector(
      &total->support_summary.measured_centroid_sensor_m,
      sample.support_summary.measured_centroid_sensor_m, weight);
  total->min_signed_distance_m =
      total->object_sample_count == sample.object_sample_count
          ? sample.min_signed_distance_m
          : std::min(total->min_signed_distance_m,
                     sample.min_signed_distance_m);
  total->min_edge_margin_m =
      total->object_sample_count == sample.object_sample_count
          ? sample.min_edge_margin_m
          : std::min(total->min_edge_margin_m, sample.min_edge_margin_m);
}

void DivideAverages(ObjectContactSupportEvaluation* total,
                    const double weight_sum) {
  if (total == nullptr || !total->valid || weight_sum <= 0.0 ||
      !std::isfinite(weight_sum)) {
    return;
  }
  total->measured_active_hemisphere_total /= weight_sum;
  total->measured_active_tactile_sensors /= weight_sum;
  total->predicted_active_hemisphere_total /= weight_sum;
  total->predicted_active_tactile_sensors /= weight_sum;
  total->lost_measured_contact_count /= weight_sum;
  total->predicted_normal_force_total_n /= weight_sum;
  total->contact_loss_cost /= weight_sum;
  total->support_cost /= weight_sum;
  total->edge_cost /= weight_sum;
  total->penetration_cost /= weight_sum;
  total->predicted_force_low_cost /= weight_sum;
  total->predicted_force_high_cost /= weight_sum;
  total->support_summary.predicted_support_area /= weight_sum;
  if (total->support_summary.predicted_centroid_sensor_m.allFinite()) {
    total->support_summary.predicted_centroid_sensor_m /= weight_sum;
  }
  if (total->support_summary.measured_centroid_sensor_m.allFinite()) {
    total->support_summary.measured_centroid_sensor_m /= weight_sum;
  }
  FinalizeSummaryFromEvaluation(total);
}

}  // namespace

ObjectContactSupportEvaluation EvaluateObjectContactSupport(
    const GraspState& state,
    const RolloutContext& context,
    const ObjectContactSupportEvaluatorConfig& config) {
  ObjectContactSupportEvaluation out;
  if (!config.enabled || !HasValidConfig(config) || !state.valid ||
      !IsValidVirtualObjectBelief(state.object_belief) ||
      !state.object_belief.hasParticles() ||
      state.tactile_sensors.size() != context.tactile_contexts.size()) {
    return out;
  }

  const auto points = ComputeCandidateHemispherePoints(state, context);
  if (points.empty()) {
    return out;
  }
  const auto sensor_bounds =
      ComputeSensorGridBounds(points, state.tactile_sensors.size());

  double weight_sum = 0.0;
  std::size_t used_samples = 0;
  out.min_signed_distance_m = std::numeric_limits<double>::infinity();
  out.min_edge_margin_m = std::numeric_limits<double>::infinity();
  for (const auto& particle : state.object_belief.particles) {
    if (used_samples >= config.max_object_samples) {
      break;
    }
    const double weight = SafeWeight(particle, config);
    if (weight <= 0.0) {
      continue;
    }
    const auto sample = EvaluateOneObjectSample(
        state, state.object_belief.geometry, particle, points, sensor_bounds,
        config);
    if (!sample.valid) {
      continue;
    }
    AccumulateWeighted(&out, sample, weight);
    weight_sum += weight;
    ++used_samples;
  }

  DivideAverages(&out, weight_sum);
  if (!out.valid || used_samples == 0) {
    return {};
  }
  out.object_sample_count = used_samples;
  if (!std::isfinite(out.min_signed_distance_m)) {
    out.min_signed_distance_m = 0.0;
  }
  if (!std::isfinite(out.min_edge_margin_m)) {
    out.min_edge_margin_m = config.target_edge_margin_m;
  }
  return out;
}

}  // namespace mppi_core
