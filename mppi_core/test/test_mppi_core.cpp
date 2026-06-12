// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/joint-prismatic.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unbounded.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mppi_core/config/mppi_config.hpp"
#include "mppi_core/config/robust_grasp_policy_config.hpp"
#include "mppi_core/config/rollout_config.hpp"
#include "mppi_core/contact/contact_force_correction.hpp"
#include "mppi_core/contact/contact_force_projection.hpp"
#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/core/mppi_optimizer.hpp"
#include "mppi_core/costs/grasp_stability_cost.hpp"
#include "mppi_core/logging/mppi_rollout_logger.hpp"
#include "mppi_core/logging/vector_csv.hpp"
#include "mppi_core/object/object_belief_initializer.hpp"
#include "mppi_core/object/object_contact_prediction.hpp"
#include "mppi_core/object/object_contact_support_evaluator.hpp"
#include "mppi_core/object/object_geometry_query.hpp"
#include "mppi_core/object/object_prior_estimator.hpp"
#include "mppi_core/policy/continuous_qddot_mppi.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/rollout/contact_force_rollout.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
#include "mppi_core/rollout/object_prior_grasp_rollout.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/task/task_config.hpp"
#include "mppi_core/tactile/nari_touch_adapter.hpp"
#include "mppi_core/tactile/tactile_transition.hpp"

namespace {

constexpr double kTolerance = 1.0e-9;

mppi_core::GraspState MakeState(std::size_t dim,
                                const mppi_core::TactileState& tactile) {
  const Eigen::VectorXd zero =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  return mppi_core::MakeGraspState(zero, zero, zero, tactile, tactile);
}

mppi_core::TactileState ToTestTactileState(mppi_core::NariTouchState nari) {
  nari.valid = true;
  nari.stamp_sec = 1.0;
  nari.sensor_index = 2;
  nari.frame_name = "test_tactile";
  return mppi_core::ConvertNariTouchToTactileState(nari);
}

struct TestPinocchioSensorModel {
  pinocchio::Model model;
  pinocchio::FrameIndex sensor_frame_id{0};
};

TestPinocchioSensorModel MakeSingleRevoluteZSensorModel() {
  TestPinocchioSensorModel out;
  const auto joint_id = out.model.addJoint(
      0, pinocchio::JointModelRZ(), pinocchio::SE3::Identity(), "finger_rz");
  out.sensor_frame_id = out.model.addFrame(
      pinocchio::Frame("tactile_sensor", joint_id, 0,
                       pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  return out;
}

TestPinocchioSensorModel MakeSinglePrismaticZSensorModel() {
  TestPinocchioSensorModel out;
  const auto joint_id = out.model.addJoint(
      0, pinocchio::JointModelPZ(), pinocchio::SE3::Identity(), "finger_pz");
  out.sensor_frame_id = out.model.addFrame(
      pinocchio::Frame("tactile_sensor", joint_id, 0,
                       pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  return out;
}

TestPinocchioSensorModel MakeSinglePrismaticZSensorModelWithInertia() {
  TestPinocchioSensorModel out = MakeSinglePrismaticZSensorModel();
  const pinocchio::JointIndex joint_id = out.model.getJointId("finger_pz");
  out.model.appendBodyToJoint(
      joint_id,
      pinocchio::Inertia(
          1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()),
      pinocchio::SE3::Identity());
  return out;
}

TestPinocchioSensorModel MakePrismaticAndRevoluteZSensorModel() {
  TestPinocchioSensorModel out;
  const auto prismatic_joint_id = out.model.addJoint(
      0, pinocchio::JointModelPZ(), pinocchio::SE3::Identity(), "finger_pz");
  const auto revolute_joint_id =
      out.model.addJoint(prismatic_joint_id, pinocchio::JointModelRZ(),
                         pinocchio::SE3::Identity(), "finger_rz");
  out.sensor_frame_id = out.model.addFrame(
      pinocchio::Frame("tactile_sensor", revolute_joint_id, 0,
                       pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  return out;
}

TestPinocchioSensorModel MakeSingleUnboundedRevoluteZSensorModel() {
  TestPinocchioSensorModel out;
  const auto joint_id =
      out.model.addJoint(0, pinocchio::JointModelRUBZ(),
                         pinocchio::SE3::Identity(), "finger_rubz");
  out.sensor_frame_id = out.model.addFrame(
      pinocchio::Frame("tactile_sensor", joint_id, 0,
                       pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  return out;
}

mppi_core::GraspState MakeContactKinematicsState(
    const pinocchio::Model& model, const mppi_core::TactileState& tactile) {
  return mppi_core::MakeGraspState(
      pinocchio::neutral(model), Eigen::VectorXd::Zero(model.nv),
      Eigen::VectorXd::Zero(model.nv), tactile, tactile);
}

Eigen::Vector3d WorldPointPosition(const pinocchio::Model& model,
                                   pinocchio::Data* data,
                                   pinocchio::FrameIndex sensor_frame_id,
                                   const Eigen::VectorXd& q,
                                   const Eigen::Vector3d& point_sensor_m) {
  pinocchio::forwardKinematics(model, *data, q);
  pinocchio::updateFramePlacements(model, *data);
  return data->oMf[sensor_frame_id].act(point_sensor_m);
}

mppi_core::HemisphereState MakeHemisphere(std::size_t index,
                                          const Eigen::Vector2d& cop_sensor_m,
                                          double normal_force_n = 1.0,
                                          bool contact = true) {
  mppi_core::HemisphereState hemisphere;
  hemisphere.hemisphere_index = index;
  hemisphere.contact = contact;
  hemisphere.cop_sensor_m = cop_sensor_m;
  hemisphere.normal_force_n = contact ? normal_force_n : 0.0;
  hemisphere.confidence = 1.0;
  return hemisphere;
}

Eigen::Vector3d HemispherePointSensorM(
    const mppi_core::HemisphereState& hemisphere) {
  return Eigen::Vector3d{hemisphere.cop_sensor_m.x(),
                         hemisphere.cop_sensor_m.y(), 0.0};
}

void SetActiveHemispheresAroundCentroid(mppi_core::TactileState* tactile,
                                        std::size_t active_count,
                                        const Eigen::Vector2d& centroid_m,
                                        double total_normal_force_n) {
  tactile->hemispheres.clear();
  tactile->total_force_n =
      Eigen::Vector3d{0.0, 0.0, std::max(0.0, total_normal_force_n)};
  tactile->contact_state = active_count > 0
                               ? mppi_core::TactileState::kEnoughContacts
                               : mppi_core::TactileState::kNoContact;

  if (active_count == 0) {
    return;
  }

  const double per_hemisphere_force =
      std::max(0.0, total_normal_force_n) / static_cast<double>(active_count);
  const double spacing_m = 1.0e-4;
  const double center = 0.5 * static_cast<double>(active_count - 1);
  for (std::size_t i = 0; i < active_count; ++i) {
    const double offset_x = (static_cast<double>(i) - center) * spacing_m;
    tactile->hemispheres.push_back(MakeHemisphere(
        i, centroid_m + Eigen::Vector2d{offset_x, 0.0}, per_hemisphere_force));
  }
}

mppi_core::TactileState MakeInactiveTactileState(
    mppi_core::TactileState tactile) {
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.total_force_n.setZero();
  tactile.shear_displacement_m.setZero();
  tactile.rotational_shear_rad = 0.0;
  tactile.shear_velocity_mps.setZero();
  tactile.rotational_shear_velocity_radps = 0.0;
  tactile.slip_score = 0.0;
  tactile.slip_velocity_score = 0.0;
  tactile.incipient_slip_score = 0.0;
  tactile.confidence = 0.0;
  for (auto& hemisphere : tactile.hemispheres) {
    hemisphere.contact = false;
    hemisphere.normal_force_n = 0.0;
    hemisphere.confidence = 0.0;
  }
  return tactile;
}

bool ComputeTestTactileCentroidM(const mppi_core::TactileState& tactile,
                                 Eigen::Vector2d* centroid_m) {
  if (centroid_m == nullptr) {
    return false;
  }

  Eigen::Vector2d weighted_sum = Eigen::Vector2d::Zero();
  double weight_sum = 0.0;
  for (const auto& hemisphere : tactile.hemispheres) {
    if (!hemisphere.contact || !hemisphere.cop_sensor_m.allFinite()) {
      continue;
    }
    const double weight = std::isfinite(hemisphere.normal_force_n) &&
                                  hemisphere.normal_force_n > 0.0
                              ? hemisphere.normal_force_n
                              : 1.0;
    weighted_sum += weight * hemisphere.cop_sensor_m;
    weight_sum += weight;
  }

  if (weight_sum <= 0.0) {
    return false;
  }
  *centroid_m = weighted_sum / weight_sum;
  return centroid_m->allFinite();
}

Eigen::Vector2d TestTactileCentroidM(const mppi_core::TactileState& tactile) {
  Eigen::Vector2d centroid_m = Eigen::Vector2d::Zero();
  (void)ComputeTestTactileCentroidM(tactile, &centroid_m);
  return centroid_m;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::ostringstream contents;
  contents << in.rdbuf();
  return contents.str();
}

std::filesystem::path MppiCorePackageRoot() {
  return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::filesystem::path UniqueTempDirectory(const std::string& prefix) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         (prefix + "_" + std::to_string(now));
}

std::filesystem::path PlatoNariTouchUrdfPath() {
  return std::filesystem::path(MPPI_CORE_PLATO_NARITOUCH_URDF_PATH);
}

std::vector<mppi_core::TactileState,
            Eigen::aligned_allocator<mppi_core::TactileState>>
MakeTactileSensors(const mppi_core::TactileState& first_tactile,
                   const mppi_core::TactileState& second_tactile) {
  std::vector<mppi_core::TactileState,
              Eigen::aligned_allocator<mppi_core::TactileState>>
      tactile_sensors;
  tactile_sensors.reserve(2);
  tactile_sensors.push_back(first_tactile);
  tactile_sensors.push_back(second_tactile);
  return tactile_sensors;
}

mppi_core::VirtualObjectBelief MakeTestObjectBelief() {
  mppi_core::ObjectGeometryHandle geometry;
  geometry.name = "test_box";
  geometry.type = mppi_core::ObjectGeometryType::kBox;
  geometry.primitive_size_m = Eigen::Vector3d{0.04, 0.03, 0.02};
  geometry.valid = true;

  mppi_core::VirtualObjectState particle;
  particle.pose_world.translation() = Eigen::Vector3d{0.01, -0.02, 0.03};
  particle.velocity_world.setZero();
  particle.weight = 1.0;
  particle.valid = true;

  mppi_core::VirtualObjectBelief belief;
  belief.geometry = geometry;
  belief.particles.push_back(particle);
  belief.valid = true;
  return belief;
}

mppi_core::ObjectPrior MakeTestBoxObjectPrior(
    const Eigen::Vector3d& center_world_m,
    const Eigen::Vector3d& size_m = Eigen::Vector3d{0.1, 0.1, 0.1}) {
  mppi_core::ObjectPrior prior;
  prior.name = "test_box_prior";
  prior.geometry.name = "test_box";
  prior.geometry.type = mppi_core::ObjectGeometryType::kBox;
  prior.geometry.primitive_size_m = size_m;
  prior.geometry.valid = true;
  prior.initial_pose_world.translation() = center_world_m;
  prior.position_std_m = Eigen::Vector3d{0.01, 0.01, 0.01};
  prior.rpy_std_rad = Eigen::Vector3d{0.05, 0.05, 0.05};
  prior.valid = true;
  return prior;
}

std::vector<mppi_core::TactileSensorContext> MakeTactileContexts(
    const mppi_core::PinocchioContactKinematicsContext* first_kinematics,
    const mppi_core::PinocchioContactKinematicsContext* second_kinematics) {
  std::vector<mppi_core::TactileSensorContext> contexts(2);
  contexts[0].kinematics = first_kinematics;
  contexts[1].kinematics = second_kinematics;
  return contexts;
}

std::vector<mppi_core::HemisphereGeometry> MakeHemisphereGeometry(
    const mppi_core::TactileState& tactile) {
  std::vector<mppi_core::HemisphereGeometry> geometry;
  geometry.reserve(tactile.hemispheres.size());
  for (std::size_t i = 0; i < tactile.hemispheres.size(); ++i) {
    const auto& hemisphere = tactile.hemispheres[i];
    mppi_core::HemisphereGeometry hemi_geometry;
    hemi_geometry.hemisphere_index = hemisphere.hemisphere_index;
    hemi_geometry.center_sensor_m = HemispherePointSensorM(hemisphere);
    hemi_geometry.normal_sensor = Eigen::Vector3d::UnitZ();
    if (i > 0) {
      hemi_geometry.neighbors.push_back(
          tactile.hemispheres[i - 1].hemisphere_index);
    }
    if (i + 1 < tactile.hemispheres.size()) {
      hemi_geometry.neighbors.push_back(
          tactile.hemispheres[i + 1].hemisphere_index);
    }
    geometry.push_back(std::move(hemi_geometry));
  }
  return geometry;
}

std::vector<mppi_core::TactileSensorContext> MakeTactileContextsForStates(
    const mppi_core::PinocchioContactKinematicsContext* first_kinematics,
    const mppi_core::TactileState& first_tactile,
    const mppi_core::PinocchioContactKinematicsContext* second_kinematics,
    const mppi_core::TactileState& second_tactile) {
  auto contexts = MakeTactileContexts(first_kinematics, second_kinematics);
  contexts[0].hemispheres = MakeHemisphereGeometry(first_tactile);
  contexts[1].hemispheres = MakeHemisphereGeometry(second_tactile);
  return contexts;
}

mppi_core::TactileSensorContext MakeTactileContextForState(
    const mppi_core::TactileState& tactile) {
  mppi_core::TactileSensorContext context;
  context.hemispheres = MakeHemisphereGeometry(tactile);
  return context;
}

mppi_core::RobotState MakeTestRobotState() {
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);
  return mppi_core::MakeRobotState(zero, zero, zero);
}

std::vector<mppi_core::HemisphereMotion> MakeMotionsForTactileState(
    const mppi_core::TactileState& tactile) {
  std::vector<mppi_core::HemisphereMotion> motions;
  motions.reserve(tactile.hemispheres.size());
  for (const auto& hemi : tactile.hemispheres) {
    mppi_core::HemisphereMotion motion;
    motion.hemisphere_index = hemi.hemisphere_index;
    motion.position_sensor_m = HemispherePointSensorM(hemi);
    motion.point_world_m = motion.position_sensor_m;
    motion.normal_world = Eigen::Vector3d::UnitZ();
    motions.push_back(std::move(motion));
  }
  return motions;
}

}  // namespace

TEST(TactileStateTest, HemispheresDefaultEmptyAndCountActiveOnly) {
  mppi_core::TactileState tactile;
  EXPECT_TRUE(tactile.hemispheres.empty());
  EXPECT_EQ(tactile.activeHemisphereCount(), 0U);
  EXPECT_FALSE(tactile.hasContact());

  mppi_core::HemisphereState inactive;
  inactive.contact = false;
  inactive.normal_force_n = 5.0;
  mppi_core::HemisphereState active;
  active.contact = true;
  active.normal_force_n = 2.0;
  tactile.hemispheres.push_back(inactive);
  tactile.hemispheres.push_back(active);
  tactile.hemispheres.push_back(inactive);

  EXPECT_EQ(tactile.activeHemisphereCount(), 1U);
  EXPECT_EQ(tactile.hemisphereCount(), 3U);
  EXPECT_NEAR(tactile.activeHemisphereNormalForceN(), 2.0, kTolerance);

  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  EXPECT_TRUE(tactile.hasContact());
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.valid = true;
  EXPECT_TRUE(tactile.readyForMppiStart());
}

TEST(RobotCommandTest, ResizeAndValidationSupportNqNvSplit) {
  mppi_core::RobotCommand command;
  command.Resize(7, 6);
  command.valid = true;

  EXPECT_TRUE(command.HasValidDimensions());
  EXPECT_TRUE(command.AllFinite());
  EXPECT_TRUE(command.IsUsable());
  EXPECT_EQ(command.q_cmd.size(), 7);
  EXPECT_EQ(command.qdot_cmd.size(), 6);
  EXPECT_EQ(command.tau_cmd.size(), 6);
  EXPECT_EQ(command.kp.size(), 6);
  EXPECT_EQ(command.kd.size(), 6);

  command.qdot_cmd = Eigen::VectorXd::Zero(5);
  EXPECT_FALSE(command.HasValidDimensions());
  EXPECT_FALSE(command.IsUsable());
}

TEST(RobotCommandTest, ZeroHoldAndInvalidHelpersSetUsability) {
  const Eigen::VectorXd q_current = Eigen::VectorXd::Constant(2, 0.3);
  const Eigen::VectorXd qdot_current = Eigen::VectorXd::Constant(2, 0.4);

  const auto hold = mppi_core::MakeZeroHoldRobotCommand(q_current, qdot_current);

  EXPECT_TRUE(hold.valid);
  EXPECT_TRUE(hold.IsUsable());
  EXPECT_NEAR(hold.q_cmd[0], 0.3, kTolerance);
  EXPECT_NEAR(hold.qdot_cmd.norm(), 0.0, kTolerance);
  EXPECT_NEAR(hold.tau_cmd.norm(), 0.0, kTolerance);
  EXPECT_NEAR(hold.kp.norm(), 0.0, kTolerance);
  EXPECT_NEAR(hold.kd.norm(), 0.0, kTolerance);

  const auto invalid = mppi_core::MakeInvalidRobotCommand(3, 2);
  EXPECT_FALSE(invalid.valid);
  EXPECT_TRUE(invalid.HasValidDimensions());
  EXPECT_TRUE(invalid.AllFinite());
  EXPECT_FALSE(invalid.IsUsable());
}

TEST(VectorCsvTest, SerializesVectorsAndEscapesCells) {
  EXPECT_EQ(mppi_core::logging::VectorToCsvCell(Eigen::VectorXd{}), "");

  Eigen::VectorXd normal(3);
  normal << 0.1, 0.2, -3.0;
  EXPECT_EQ(mppi_core::logging::VectorToCsvCell(normal), "\"0.1;0.2;-3\"");

  Eigen::VectorXd nonfinite(3);
  nonfinite << std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity();
  EXPECT_EQ(
      mppi_core::logging::VectorToCsvCell(nonfinite),
      "\"nan;inf;-inf\"");

  EXPECT_EQ(mppi_core::logging::BoolToString(true), "true");
  EXPECT_EQ(mppi_core::logging::BoolToString(false), "false");
  EXPECT_EQ(
      mppi_core::logging::EscapeCsvCell("a,\"b\""),
      "\"a,\"\"b\"\"\"");
}

TEST(MppiRolloutLoggerTest, DisabledLoggerMethodsDoNotThrow) {
  mppi_core::logging::MppiRolloutLogger logger;
  mppi_core::logging::MppiRolloutLoggerConfig config;
  config.enabled = false;

  EXPECT_TRUE(logger.Configure(config));
  EXPECT_FALSE(logger.enabled());
  EXPECT_NO_THROW(logger.LogTick(mppi_core::logging::MppiTickLogRecord{}));
  EXPECT_NO_THROW(logger.LogRollout(0, 0.0, mppi_core::RolloutTrace{}));
  EXPECT_NO_THROW(logger.LogEvent(0, 0.0, "event", "detail"));
  EXPECT_NO_THROW(logger.Flush());
}

TEST(MppiRolloutLoggerTest, EnabledLoggerWritesTickRolloutAndEventFiles) {
  const std::filesystem::path output_dir =
      UniqueTempDirectory("mppi_rollout_logger_test");

  mppi_core::logging::MppiRolloutLoggerConfig config;
  config.enabled = true;
  config.output_directory = output_dir.string();
  config.file_prefix = "unit";
  config.flush_every_n_ticks = 1;

  mppi_core::logging::MppiRolloutLogger logger;
  ASSERT_TRUE(logger.Configure(config));
  ASSERT_TRUE(logger.enabled());

  mppi_core::logging::MppiTickLogRecord record;
  record.tick_index = 7;
  record.time_s = 1.25;
  record.controller_state = "mppi_grasp";
  record.phase = "hold";
  record.q_meas = Eigen::VectorXd::Constant(1, 0.1);
  record.qdot_meas = Eigen::VectorXd::Constant(1, 0.2);
  record.tau_meas = Eigen::VectorXd::Constant(1, 0.3);
  record.q_cmd = Eigen::VectorXd::Constant(1, 0.4);
  record.qdot_cmd = Eigen::VectorXd::Constant(1, 0.5);
  record.tau_cmd = Eigen::VectorXd::Constant(1, 0.6);
  record.selected_action_qddot = Eigen::VectorXd::Constant(1, 0.7);
  record.command_valid = true;
  logger.LogTick(record);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  SetActiveHemispheresAroundCentroid(&tactile, 1, Eigen::Vector2d::Zero(), 2.0);

  mppi_core::RolloutTrace trace;
  trace.states.push_back(mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), tactile, tactile));
  trace.states.push_back(mppi_core::MakeGraspState(
      Eigen::VectorXd::Constant(1, 0.11), Eigen::VectorXd::Constant(1, 0.1),
      Eigen::VectorXd::Constant(1, 0.2), tactile, tactile));
  trace.actions.push_back(Eigen::VectorXd::Constant(1, 1.0));
  trace.step_costs.push_back(0.5);
  trace.total_cost = 0.5;
  logger.LogRollout(7, 1.25, trace);
  logger.LogEvent(7, 1.25, "event", "detail");
  logger.Flush();

  const auto ticks_path = output_dir / "unit_ticks.csv";
  const auto rollouts_path = output_dir / "unit_rollouts.csv";
  const auto events_path = output_dir / "unit_events.csv";
  ASSERT_TRUE(std::filesystem::exists(ticks_path));
  ASSERT_TRUE(std::filesystem::exists(rollouts_path));
  ASSERT_TRUE(std::filesystem::exists(events_path));

  const std::string ticks = ReadTextFile(ticks_path);
  const std::string rollouts = ReadTextFile(rollouts_path);
  const std::string events = ReadTextFile(events_path);
  EXPECT_NE(ticks.find("tick_index,time_s,controller_state"), std::string::npos);
  EXPECT_NE(ticks.find("7,1.25,mppi_grasp"), std::string::npos);
  EXPECT_NE(rollouts.find("horizon_index,pred_time_s"), std::string::npos);
  EXPECT_NE(rollouts.find("7,1.25,1"), std::string::npos);
  EXPECT_NE(events.find("tick_index,time_s,event,detail"), std::string::npos);
  EXPECT_NE(events.find("7,1.25,event,detail"), std::string::npos);

  std::filesystem::remove_all(output_dir);
}

TEST(RobotSystemTest, StoresAndReturnsRobotStateForPinocchioModel) {
  const auto model = MakeSingleRevoluteZSensorModel();
  mppi_core::RobotSystem robot_system(model.model);

  EXPECT_TRUE(robot_system.hasModel());
  EXPECT_FALSE(robot_system.hasState());
  EXPECT_EQ(robot_system.nq(), model.model.nq);
  EXPECT_EQ(robot_system.nv(), model.model.nv);

  const Eigen::VectorXd q = pinocchio::neutral(model.model);
  const Eigen::VectorXd qdot = Eigen::VectorXd::Constant(model.model.nv, 0.2);
  const Eigen::VectorXd tau = Eigen::VectorXd::Constant(model.model.nv, 0.4);

  robot_system.UpdateState(q, qdot, tau, 1.25);

  ASSERT_TRUE(robot_system.hasState());
  const auto& state = robot_system.state();
  EXPECT_TRUE(state.q.isApprox(q));
  EXPECT_TRUE(state.qdot.isApprox(qdot));
  EXPECT_TRUE(state.tau.isApprox(tau));
  EXPECT_NEAR(state.time_s, 1.25, kTolerance);
}

TEST(RobotSystemTest, RejectsRobotStateDimensionMismatch) {
  const auto model = MakeSingleRevoluteZSensorModel();
  mppi_core::RobotSystem robot_system(model.model);

  const Eigen::VectorXd wrong_q = Eigen::VectorXd::Zero(model.model.nq + 1);
  const Eigen::VectorXd qdot = Eigen::VectorXd::Zero(model.model.nv);

  EXPECT_THROW(robot_system.UpdateState(wrong_q, qdot), std::invalid_argument);
}

TEST(GraspStateTest, MakeGraspStateValidatesDimensionsAndTactileValidity) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;

  const Eigen::VectorXd q = Eigen::VectorXd::Constant(2, 0.25);
  const Eigen::VectorXd dq = Eigen::VectorXd::Constant(2, -0.5);
  const Eigen::VectorXd tau = Eigen::VectorXd::Constant(2, 1.25);

  const auto valid_state =
      mppi_core::MakeGraspState(q, dq, tau, tactile, tactile);
  EXPECT_TRUE(valid_state.valid);
  EXPECT_EQ(valid_state.robot.q.size(), 2);
  EXPECT_EQ(valid_state.robot.qdot.size(), 2);
  EXPECT_EQ(valid_state.robot.tau.size(), 2);
  EXPECT_NEAR(valid_state.robot.q[0], 0.25, kTolerance);
  EXPECT_NEAR(valid_state.robot.qdot[1], -0.5, kTolerance);
  EXPECT_NEAR(valid_state.robot.tau[0], 1.25, kTolerance);
  EXPECT_TRUE(valid_state.tactile_sensors[0].valid);
  EXPECT_NEAR(valid_state.tactile_sensors[0].total_force_n.z(), 1.0,
              kTolerance);

  tactile.valid = false;
  const auto invalid_tactile_state =
      mppi_core::MakeGraspState(q, dq, tau, tactile, tactile);
  EXPECT_FALSE(invalid_tactile_state.valid);

  tactile.valid = true;
  const Eigen::VectorXd mismatched_dq = Eigen::VectorXd::Zero(3);
  const auto mismatched_state =
      mppi_core::MakeGraspState(q, mismatched_dq, tau, tactile, tactile);
  EXPECT_FALSE(mismatched_state.valid);
  EXPECT_EQ(mismatched_state.robot.q.size(), 2);
  EXPECT_EQ(mismatched_state.robot.qdot.size(), 3);

  const Eigen::VectorXd wrong_tau = Eigen::VectorXd::Zero(3);
  const auto bad_torque_state =
      mppi_core::MakeGraspState(q, dq, wrong_tau, tactile, tactile);
  EXPECT_FALSE(bad_torque_state.valid);

  const Eigen::VectorXd q_nq_not_nv = Eigen::VectorXd::Constant(3, 0.2);
  const auto pinocchio_shaped_state =
      mppi_core::MakeGraspState(q_nq_not_nv, dq, tau, tactile, tactile);
  EXPECT_TRUE(pinocchio_shaped_state.valid);
  EXPECT_EQ(pinocchio_shaped_state.robot.q.size(), 3);
  EXPECT_EQ(pinocchio_shaped_state.robot.qdot.size(), 2);
  EXPECT_EQ(pinocchio_shaped_state.robot.tau.size(), 2);
}

TEST(GraspStateTest, MakeGraspStateRejectsNonFiniteJointVectors) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(2);

  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mppi_core::MakeGraspState(q, dq, tau, tactile, tactile).valid);

  q[0] = 0.0;
  dq[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(mppi_core::MakeGraspState(q, dq, tau, tactile, tactile).valid);

  dq[1] = 0.0;
  tau[0] = std::numeric_limits<double>::quiet_NaN();
  const auto bad_torque_state =
      mppi_core::MakeGraspState(q, dq, tau, tactile, tactile);
  EXPECT_FALSE(bad_torque_state.valid);
}

TEST(GraspStateTest, ExplicitTwoSensorStateRequiresBothTactileStates) {
  const mppi_core::RobotState robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Zero(2));

  mppi_core::TactileState tactile0;
  tactile0.valid = true;
  tactile0.sensor_index = 0;
  mppi_core::TactileState tactile1;
  tactile1.valid = true;
  tactile1.sensor_index = 1;

  const auto valid_state = mppi_core::MakeGraspState(robot, tactile0, tactile1);
  EXPECT_TRUE(valid_state.valid);
  EXPECT_EQ(valid_state.tactile_sensors[0].sensor_index, 0);
  EXPECT_EQ(valid_state.tactile_sensors[1].sensor_index, 1);

  tactile1.valid = false;
  const auto invalid_state =
      mppi_core::MakeGraspState(robot, tactile0, tactile1);
  EXPECT_FALSE(invalid_state.valid);
}

TEST(GraspStateTest, ObjectBeliefIsOptionalAndCarriedWhenProvided) {
  const mppi_core::RobotState robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Zero(2));

  mppi_core::TactileState tactile0;
  tactile0.valid = true;
  mppi_core::TactileState tactile1;
  tactile1.valid = true;

  const auto tactile_only_state =
      mppi_core::MakeGraspState(robot, tactile0, tactile1);
  EXPECT_TRUE(tactile_only_state.valid);
  EXPECT_FALSE(tactile_only_state.hasObjectBelief());

  const auto object_belief = MakeTestObjectBelief();
  const auto object_state =
      mppi_core::MakeGraspState(robot, tactile0, tactile1, object_belief);
  EXPECT_TRUE(object_state.valid);
  EXPECT_TRUE(object_state.hasObjectBelief());
  ASSERT_EQ(object_state.object_belief.particleCount(), 1U);
  EXPECT_NEAR(object_state.object_belief.particles[0].pose_world.translation().x(),
              0.01, kTolerance);
}

TEST(ObjectGeometryQueryTest, BoxQueryUsesSignedDistanceConvention) {
  mppi_core::ObjectGeometryHandle geometry;
  geometry.name = "box";
  geometry.type = mppi_core::ObjectGeometryType::kBox;
  geometry.primitive_size_m = Eigen::Vector3d{0.1, 0.1, 0.1};
  geometry.valid = true;

  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const mppi_core::ObjectGeometryQuery query(geometry);

  const auto outside = query.Query(pose, Eigen::Vector3d{0.0, 0.0, 0.06});
  EXPECT_TRUE(outside.valid);
  EXPECT_NEAR(outside.signed_distance_m, 0.01, kTolerance);
  EXPECT_NEAR(outside.closest_point_world.z(), 0.05, kTolerance);
  EXPECT_NEAR(outside.normal_world.z(), 1.0, kTolerance);

  const auto surface = query.Query(pose, Eigen::Vector3d{0.0, 0.0, 0.05});
  EXPECT_TRUE(surface.valid);
  EXPECT_NEAR(surface.signed_distance_m, 0.0, kTolerance);
  EXPECT_NEAR(surface.normal_world.z(), 1.0, kTolerance);

  const auto inside = query.Query(pose, Eigen::Vector3d::Zero());
  EXPECT_TRUE(inside.valid);
  EXPECT_NEAR(inside.signed_distance_m, -0.05, kTolerance);
}

TEST(ObjectBeliefInitializerTest,
     InitializesParticleWeightsFromMeasuredContactAndPrior) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(MakeHemisphere(3, Eigen::Vector2d::Zero()));

  mppi_core::ObjectBeliefInitializationConfig config;
  config.particle_count = 1;
  config.random_seed = 4;

  const Eigen::VectorXd q_meas = Eigen::VectorXd::Constant(1, 0.05);
  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);
  const auto result = mppi_core::InitializeObjectBeliefFromContacts(
      prior, q_meas, MakeTactileSensors(tactile, inactive_tactile),
      MakeTactileContextsForStates(&kinematics, tactile, nullptr,
                                   inactive_tactile),
      config);

  EXPECT_TRUE(result.valid);
  ASSERT_EQ(result.contacts.size(), 1U);
  ASSERT_EQ(result.belief.particleCount(), 1U);
  EXPECT_TRUE(result.belief.valid);
  EXPECT_NEAR(result.belief.particles[0].weight, 1.0, kTolerance);
  EXPECT_NEAR(result.contacts[0].point_world_m.z(), 0.05, kTolerance);
  EXPECT_NEAR(result.contacts[0].normal_world.z(), -1.0, kTolerance);
  EXPECT_NEAR(result.best_surface_distance_m, 0.0, kTolerance);
  EXPECT_NEAR(result.best_normal_alignment_error, 0.0, kTolerance);

  config.min_contact_count = 2;
  const auto not_enough_contacts = mppi_core::InitializeObjectBeliefFromContacts(
      prior, q_meas, MakeTactileSensors(tactile, inactive_tactile),
      MakeTactileContextsForStates(&kinematics, tactile, nullptr,
                                   inactive_tactile),
      config);
  EXPECT_FALSE(not_enough_contacts.valid);
}

TEST(ObjectBeliefInitializerTest, ObservationInitializationUsesMeasuredQ) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(MakeHemisphere(0, Eigen::Vector2d::Zero()));
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::HemisphereGeometry geometry;
  geometry.hemisphere_index = 0;
  geometry.center_sensor_m = Eigen::Vector3d::Zero();
  geometry.normal_sensor = Eigen::Vector3d::UnitZ();

  mppi_core::TactileSensorContext tactile_context;
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;
  tactile_context.hemispheres.push_back(geometry);

  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = Eigen::VectorXd::Constant(1, 0.05);
  observation.tactile_meas = MakeTactileSensors(tactile, inactive_tactile);
  observation.tactile_contexts =
      {tactile_context, mppi_core::TactileSensorContext{}};
  observation.object_prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());

  mppi_core::ObjectBeliefInitializationConfig config;
  config.particle_count = 1;

  const auto result =
      mppi_core::InitializeObjectBeliefFromObservation(observation, config);

  EXPECT_TRUE(result.valid);
  ASSERT_EQ(result.contacts.size(), 1U);
  EXPECT_NEAR(result.contacts[0].point_world_m.z(), 0.05, kTolerance);
  EXPECT_NEAR(result.best_surface_distance_m, 0.0, kTolerance);

  const auto resolved = mppi_core::ResolveObjectBeliefForObservation(
      observation, observation.q_meas, config);
  EXPECT_TRUE(mppi_core::HasVirtualObjectBelief(resolved));
  EXPECT_TRUE(mppi_core::IsValidVirtualObjectBelief(resolved));
  ASSERT_EQ(resolved.particleCount(), 1U);
  EXPECT_NEAR(resolved.particles[0].weight, 1.0, kTolerance);
}

TEST(ObjectBeliefInitializerTest,
     ExistingBeliefIsReweightedFromCurrentTactileContacts) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(MakeHemisphere(0, Eigen::Vector2d::Zero()));

  auto tactile_context = MakeTactileContextForState(tactile);
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief previous;
  previous.geometry = prior.geometry;
  for (const auto z_m : {0.0, 0.2}) {
    mppi_core::VirtualObjectState particle;
    particle.pose_world.setIdentity();
    particle.pose_world.translation().z() = z_m;
    particle.velocity_world.setZero();
    particle.weight = z_m > 0.0 ? 0.95 : 0.05;
    particle.valid = true;
    previous.particles.push_back(std::move(particle));
  }
  previous.valid = true;

  mppi_core::ObjectBeliefInitializationConfig config;
  config.min_contact_count = 1;
  config.w_prior = 0.0;

  const Eigen::VectorXd q_meas = Eigen::VectorXd::Constant(1, 0.05);
  const auto result = mppi_core::UpdateObjectBeliefFromCurrentContacts(
      prior, previous, q_meas,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      std::vector<mppi_core::TactileSensorContext>{tactile_context}, config);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.contacts.size(), 1U);
  ASSERT_EQ(result.belief.particleCount(), 2U);
  EXPECT_EQ(result.best_particle_index, 0U);
  EXPECT_GT(result.belief.particles[0].weight,
            result.belief.particles[1].weight);
  EXPECT_NEAR(result.best_surface_distance_m, 0.0, kTolerance);

  mppi_core::GraspObservation observation;
  observation.q_meas = q_meas;
  observation.tactile_meas =
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile};
  observation.tactile_contexts = {tactile_context};
  observation.object_prior = prior;
  observation.object_belief = previous;

  const auto resolved = mppi_core::ResolveObjectBeliefForObservation(
      observation, q_meas, config);
  ASSERT_TRUE(mppi_core::HasVirtualObjectBelief(resolved));
  ASSERT_EQ(resolved.particleCount(), 2U);
  EXPECT_GT(resolved.particles[0].weight, resolved.particles[1].weight);
}

TEST(ObjectPriorEstimatorTest, UpdatesAndKeepsBeliefAcrossMissingContacts) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(MakeHemisphere(0, Eigen::Vector2d::Zero()));

  auto tactile_context = MakeTactileContextForState(tactile);
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;

  mppi_core::ObjectBeliefInitializationConfig config;
  config.particle_count = 1;
  config.min_contact_count = 1;

  mppi_core::ObjectPriorEstimator estimator;
  ASSERT_TRUE(estimator.Configure(
      MakeTestBoxObjectPrior(Eigen::Vector3d::Zero()), config));

  const Eigen::VectorXd q_meas = Eigen::VectorXd::Constant(1, 0.05);
  const auto first_result = estimator.Update(
      q_meas,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      std::vector<mppi_core::TactileSensorContext>{tactile_context});

  ASSERT_TRUE(first_result.valid);
  EXPECT_TRUE(estimator.has_belief());
  EXPECT_TRUE(estimator.status().updated);
  EXPECT_TRUE(estimator.status().belief_valid);
  EXPECT_EQ(estimator.status().update_count, 1U);
  EXPECT_EQ(estimator.status().contact_count, 1U);
  EXPECT_EQ(estimator.status().particle_count, 1U);

  Eigen::Isometry3d representative_pose = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(estimator.representativePose(&representative_pose));
  EXPECT_TRUE(representative_pose.matrix().allFinite());

  const auto inactive_tactile = MakeInactiveTactileState(tactile);
  const auto missing_contact_result = estimator.Update(
      q_meas,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{
          inactive_tactile},
      std::vector<mppi_core::TactileSensorContext>{tactile_context});

  EXPECT_FALSE(missing_contact_result.valid);
  EXPECT_TRUE(estimator.has_belief());
  EXPECT_FALSE(estimator.status().updated);
  EXPECT_TRUE(estimator.status().belief_valid);
  EXPECT_EQ(estimator.status().update_count, 1U);
  EXPECT_EQ(estimator.status().contact_count, 0U);
  EXPECT_EQ(estimator.status().particle_count, 1U);
}

TEST(ObjectPriorEstimatorTest, RejectsCloseTwoSensorGapAsFingertipTouch) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState thumb;
  thumb.valid = true;
  thumb.sensor_index = 0;
  thumb.contact_state = mppi_core::TactileState::kEnoughContacts;
  thumb.hemispheres.push_back(MakeHemisphere(0, Eigen::Vector2d::Zero()));

  mppi_core::TactileState index = thumb;
  index.sensor_index = 1;

  auto thumb_context = MakeTactileContextForState(thumb);
  thumb_context.sensor_index = 0;
  thumb_context.kinematics = &kinematics;
  auto index_context = MakeTactileContextForState(index);
  index_context.sensor_index = 1;
  index_context.kinematics = &kinematics;

  mppi_core::ObjectPriorEstimatorConfig config;
  config.belief.particle_count = 1;
  config.belief.min_contact_count = 1;
  config.reject_close_sensor_gap_as_fingertip_touch = true;
  config.sensor_gap_min_object_extent_scale = 0.8;
  config.sensor_gap_min_margin_m = 0.0;

  mppi_core::ObjectPriorEstimator estimator;
  ASSERT_TRUE(estimator.Configure(
      MakeTestBoxObjectPrior(Eigen::Vector3d::Zero(),
                             Eigen::Vector3d{0.10, 0.08, 0.06}),
      config));

  const Eigen::VectorXd q_meas = Eigen::VectorXd::Constant(1, 0.05);
  const auto result = estimator.Update(
      q_meas, MakeTactileSensors(thumb, index),
      std::vector<mppi_core::TactileSensorContext>{
          thumb_context, index_context});

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(estimator.has_belief());
  EXPECT_FALSE(estimator.status().updated);
  EXPECT_TRUE(estimator.status().close_sensor_gap_rejected);
  EXPECT_EQ(estimator.status().update_count, 0U);
  EXPECT_EQ(estimator.status().contact_count, 2U);
  EXPECT_NEAR(estimator.status().sensor_gap_m, 0.0, kTolerance);
  EXPECT_NEAR(estimator.status().object_min_extent_m, 0.06, kTolerance);
  EXPECT_NEAR(estimator.status().min_sensor_gap_m, 0.048, kTolerance);
}

TEST(ObjectContactPredictionTest,
     PredictsInactiveHemisphereBirthFromObjectDistance) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.hemispheres.push_back(MakeHemisphere(
      0, Eigen::Vector2d::Zero(), 0.0, false));

  mppi_core::HemisphereGeometry geometry;
  geometry.hemisphere_index = 0;
  geometry.center_sensor_m = Eigen::Vector3d::Zero();
  geometry.normal_sensor = Eigen::Vector3d::UnitZ();

  mppi_core::TactileSensorContext tactile_context;
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;
  tactile_context.hemispheres.push_back(geometry);

  mppi_core::VirtualObjectState object;
  object.pose_world.setIdentity();
  object.velocity_world.setZero();
  object.weight = 1.0;
  object.valid = true;
  const auto object_prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());

  mppi_core::ObjectContactPredictionConfig config;
  config.contact_distance_threshold_m = 0.002;
  config.contact_stiffness_n_per_m = 500.0;
  config.min_predicted_contact_force_n = 0.001;

  const Eigen::VectorXd q = Eigen::VectorXd::Constant(1, 0.05);
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);
  const auto near_predictions = mppi_core::PredictObjectHemisphereContacts(
      q, MakeTactileSensors(tactile, inactive_tactile),
      {tactile_context, mppi_core::TactileSensorContext{}}, object,
      object_prior.geometry, config);

  ASSERT_EQ(near_predictions.size(), 1U);
  EXPECT_FALSE(near_predictions[0].measured_contact);
  EXPECT_TRUE(near_predictions[0].predicted_contact);
  EXPECT_NEAR(near_predictions[0].signed_distance_m, 0.0, kTolerance);
  EXPECT_GT(near_predictions[0].predicted_normal_force_n, 0.0);

  object.pose_world.translation().z() = -0.02;
  const auto far_predictions = mppi_core::PredictObjectHemisphereContacts(
      q, MakeTactileSensors(tactile, inactive_tactile),
      {tactile_context, mppi_core::TactileSensorContext{}}, object,
      object_prior.geometry, config);

  ASSERT_EQ(far_predictions.size(), 1U);
  EXPECT_FALSE(far_predictions[0].predicted_contact);
  EXPECT_GT(far_predictions[0].signed_distance_m,
            config.contact_distance_threshold_m);
}

TEST(ObjectContactSupportEvaluatorTest,
     StableSurfaceContactHasLowScenarioCost) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(MakeHemisphere(
      0, Eigen::Vector2d{0.0, 0.0}, 1.0, true));
  tactile.hemispheres.push_back(MakeHemisphere(
      1, Eigen::Vector2d{0.002, 0.0}, 0.0, false));
  tactile.hemispheres.push_back(MakeHemisphere(
      2, Eigen::Vector2d{0.0, 0.002}, 0.0, false));
  tactile.hemispheres.push_back(MakeHemisphere(
      3, Eigen::Vector2d{0.002, 0.002}, 0.0, false));

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectState object;
  object.pose_world.setIdentity();
  object.velocity_world.setZero();
  object.weight = 1.0;
  object.valid = true;
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(object);
  belief.valid = true;

  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.05), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  const auto state = mppi_core::MakeGraspState(
      robot,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts =
      std::vector<mppi_core::TactileSensorContext>{
          MakeTactileContextForState(tactile)};
  context.tactile_contexts[0].sensor_index = 0;
  context.tactile_contexts[0].kinematics = &kinematics;

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.002;
  config.contact_loss_margin_m = 0.004;
  config.contact_stiffness_n_per_m = 0.0;
  config.min_predicted_contact_force_n = 1.0e6;
  config.target_predicted_normal_force_n = 1.0e6;
  config.predicted_force_low_weight = 1.0e6;
  config.predicted_force_high_weight = 1.0e6;
  config.target_edge_margin_m = 0.0;

  const auto evaluation =
      mppi_core::EvaluateObjectContactSupport(state, context, config);

  EXPECT_TRUE(evaluation.valid);
  EXPECT_EQ(evaluation.object_sample_count, 1U);
  EXPECT_EQ(evaluation.geometry_query_count, 4U);
  EXPECT_NEAR(evaluation.measured_active_hemisphere_total, 1.0, kTolerance);
  EXPECT_NEAR(evaluation.predicted_active_hemisphere_total, 4.0, kTolerance);
  EXPECT_NEAR(evaluation.predicted_normal_force_total_n, 0.0, kTolerance);
  EXPECT_NEAR(evaluation.predicted_force_low_cost, 0.0, kTolerance);
  EXPECT_NEAR(evaluation.predicted_force_high_cost, 0.0, kTolerance);
  EXPECT_NEAR(evaluation.lost_measured_contact_count, 0.0, kTolerance);
  EXPECT_NEAR(evaluation.totalCost(), 0.0, kTolerance);
}

TEST(ObjectContactSupportEvaluatorTest,
     PositiveGapUsesGoodContactBandInsteadOfPredictedForce) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  mppi_core::RolloutContext context;
  context.tactile_contexts =
      std::vector<mppi_core::TactileSensorContext>{
          MakeTactileContextForState(tactile)};
  context.tactile_contexts[0].sensor_index = 0;
  context.tactile_contexts[0].kinematics = &kinematics;

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.003;
  config.contact_loss_margin_m = 0.004;
  config.support_distance_scale_m = 0.001;
  config.contact_stiffness_n_per_m = 0.0;
  config.min_predicted_contact_force_n = 1.0e6;
  config.target_edge_margin_m = 0.0;

  const auto near_state = mppi_core::MakeGraspState(
      mppi_core::MakeRobotState(
          Eigen::VectorXd::Constant(1, 0.051),
          Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)),
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);
  const auto farther_state = mppi_core::MakeGraspState(
      mppi_core::MakeRobotState(
          Eigen::VectorXd::Constant(1, 0.052),
          Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)),
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  const auto near =
      mppi_core::EvaluateObjectContactSupport(near_state, context, config);
  const auto farther =
      mppi_core::EvaluateObjectContactSupport(farther_state, context, config);

  ASSERT_TRUE(near.valid);
  ASSERT_TRUE(farther.valid);
  EXPECT_NEAR(near.min_signed_distance_m, 0.001, kTolerance);
  EXPECT_NEAR(farther.min_signed_distance_m, 0.002, kTolerance);
  EXPECT_NEAR(near.predicted_active_hemisphere_total, 1.0, kTolerance);
  EXPECT_NEAR(
      farther.predicted_active_hemisphere_total, std::exp(-1.0), 1.0e-9);
  EXPECT_GT(
      near.predicted_active_hemisphere_total,
      farther.predicted_active_hemisphere_total);
  EXPECT_NEAR(near.predicted_normal_force_total_n, 0.0, kTolerance);
  EXPECT_NEAR(farther.predicted_normal_force_total_n, 0.0, kTolerance);
}

TEST(ObjectContactSupportEvaluatorTest,
     ContactStiffnessDoesNotChangeGeometricSupportCost) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  const auto state = mppi_core::MakeGraspState(
      mppi_core::MakeRobotState(
          Eigen::VectorXd::Constant(1, 0.051),
          Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)),
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts =
      std::vector<mppi_core::TactileSensorContext>{
          MakeTactileContextForState(tactile)};
  context.tactile_contexts[0].sensor_index = 0;
  context.tactile_contexts[0].kinematics = &kinematics;

  mppi_core::ObjectContactSupportEvaluatorConfig low_stiffness;
  low_stiffness.min_active_tactile_sensors = 1;
  low_stiffness.min_active_hemisphere_total = 1;
  low_stiffness.contact_birth_margin_m = 0.003;
  low_stiffness.contact_loss_margin_m = 0.004;
  low_stiffness.support_distance_scale_m = 0.001;
  low_stiffness.contact_stiffness_n_per_m = 0.0;
  low_stiffness.target_predicted_normal_force_n = 100.0;
  low_stiffness.predicted_force_low_weight = 100.0;
  low_stiffness.predicted_force_high_weight = 100.0;
  low_stiffness.target_edge_margin_m = 0.0;

  auto high_stiffness = low_stiffness;
  high_stiffness.contact_stiffness_n_per_m = 1.0e9;
  high_stiffness.max_predicted_normal_force_n = 1.0e9;
  high_stiffness.max_predicted_force_per_sensor_n = 0.0;

  const auto low =
      mppi_core::EvaluateObjectContactSupport(state, context, low_stiffness);
  const auto high =
      mppi_core::EvaluateObjectContactSupport(state, context, high_stiffness);

  ASSERT_TRUE(low.valid);
  ASSERT_TRUE(high.valid);
  EXPECT_NEAR(low.totalCost(), high.totalCost(), kTolerance);
  EXPECT_NEAR(
      low.predicted_active_hemisphere_total,
      high.predicted_active_hemisphere_total,
      kTolerance);
  EXPECT_NEAR(low.predicted_normal_force_total_n, 0.0, kTolerance);
  EXPECT_NEAR(high.predicted_normal_force_total_n, 0.0, kTolerance);
}

TEST(ObjectContactSupportEvaluatorTest,
     LostSurfaceContactAddsContactLossAndSupportCost) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(MakeHemisphere(
      0, Eigen::Vector2d::Zero(), 1.0, true));

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectState object;
  object.pose_world.setIdentity();
  object.velocity_world.setZero();
  object.weight = 1.0;
  object.valid = true;
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(object);
  belief.valid = true;

  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.08), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  const auto state = mppi_core::MakeGraspState(
      robot,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts =
      std::vector<mppi_core::TactileSensorContext>{
          MakeTactileContextForState(tactile)};
  context.tactile_contexts[0].sensor_index = 0;
  context.tactile_contexts[0].kinematics = &kinematics;

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.002;
  config.contact_loss_margin_m = 0.004;
  config.min_predicted_contact_force_n = 0.001;

  const auto evaluation =
      mppi_core::EvaluateObjectContactSupport(state, context, config);

  EXPECT_TRUE(evaluation.valid);
  EXPECT_NEAR(evaluation.predicted_active_hemisphere_total, 0.0, kTolerance);
  EXPECT_NEAR(evaluation.lost_measured_contact_count, 1.0, kTolerance);
  EXPECT_GT(evaluation.contact_loss_cost, 0.0);
  EXPECT_GT(evaluation.support_cost, 0.0);
  EXPECT_GT(evaluation.totalCost(), 0.0);
}

TEST(ObjectContactSupportEvaluatorTest,
     InactiveHemisphereUsesFixedGeometryCenterInsteadOfStaleCop) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d{0.50, 0.50}, 0.0, false));

  mppi_core::HemisphereGeometry geometry;
  geometry.hemisphere_index = 0;
  geometry.center_sensor_m = Eigen::Vector3d::Zero();
  geometry.normal_sensor = Eigen::Vector3d::UnitZ();

  mppi_core::TactileSensorContext tactile_context;
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;
  tactile_context.hemispheres.push_back(geometry);

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.05), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  const auto state = mppi_core::MakeGraspState(
      robot,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts = {tactile_context};

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.002;
  config.contact_loss_margin_m = 0.004;
  config.min_predicted_contact_force_n = 0.001;
  config.target_predicted_normal_force_n = 0.0;
  config.target_edge_margin_m = 0.0;

  const auto evaluation =
      mppi_core::EvaluateObjectContactSupport(state, context, config);

  ASSERT_TRUE(evaluation.valid);
  EXPECT_NEAR(evaluation.predicted_active_hemisphere_total, 1.0, kTolerance);
  EXPECT_TRUE(evaluation.support_summary.predicted_centroid_sensor_m.allFinite());
  EXPECT_NEAR(evaluation.support_summary.predicted_centroid_sensor_m.x(), 0.0,
              kTolerance);
  EXPECT_NEAR(evaluation.support_summary.predicted_centroid_sensor_m.y(), 0.0,
              kTolerance);
}

TEST(ObjectContactSupportEvaluatorTest,
     HemisphereRadiusConvertsCenterDistanceToSurfaceGap) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));

  auto tactile_context = MakeTactileContextForState(tactile);
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;
  ASSERT_EQ(tactile_context.hemispheres.size(), 1U);
  tactile_context.hemispheres[0].radius_m = 0.001;

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.051), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  const auto state = mppi_core::MakeGraspState(
      robot,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts = {tactile_context};

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.002;
  config.contact_loss_margin_m = 0.004;
  config.min_predicted_contact_force_n = 0.0;
  config.target_predicted_normal_force_n = 0.0;
  config.target_edge_margin_m = 0.0;

  const auto evaluation =
      mppi_core::EvaluateObjectContactSupport(state, context, config);

  ASSERT_TRUE(evaluation.valid);
  EXPECT_NEAR(evaluation.min_signed_distance_m, 0.0, kTolerance);
  EXPECT_NEAR(evaluation.predicted_active_hemisphere_total, 1.0, kTolerance);
  EXPECT_NEAR(evaluation.penetration_cost, 0.0, kTolerance);
}

TEST(ObjectContactSupportEvaluatorTest,
     MeasuredContactOverlapDoesNotAddDeepContactCost) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));

  auto tactile_context = MakeTactileContextForState(tactile);
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;
  tactile_context.hemispheres[0].radius_m = 0.001;

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.048), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  const auto state = mppi_core::MakeGraspState(
      robot,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts = {tactile_context};

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.002;
  config.contact_loss_margin_m = 0.004;
  config.min_predicted_contact_force_n = 0.0;
  config.target_predicted_normal_force_n = 0.0;
  config.good_contact_gap_min_m = -0.001;
  config.deep_contact_weight = 5.0;
  config.target_edge_margin_m = 0.0;

  const auto evaluation =
      mppi_core::EvaluateObjectContactSupport(state, context, config);

  ASSERT_TRUE(evaluation.valid);
  EXPECT_LT(evaluation.min_signed_distance_m, config.good_contact_gap_min_m);
  EXPECT_NEAR(evaluation.predicted_active_hemisphere_total, 1.0, kTolerance);
  EXPECT_NEAR(evaluation.penetration_cost, 0.0, kTolerance);
}

TEST(ObjectContactSupportEvaluatorTest,
     PredictedDeepContactAddsMildGuardCost) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 0.0, false));

  auto tactile_context = MakeTactileContextForState(tactile);
  tactile_context.sensor_index = 0;
  tactile_context.kinematics = &kinematics;
  tactile_context.hemispheres[0].radius_m = 0.001;

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.048), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  const auto state = mppi_core::MakeGraspState(
      robot,
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      belief);

  mppi_core::RolloutContext context;
  context.tactile_contexts = {tactile_context};

  mppi_core::ObjectContactSupportEvaluatorConfig config;
  config.min_active_tactile_sensors = 1;
  config.min_active_hemisphere_total = 1;
  config.contact_birth_margin_m = 0.002;
  config.contact_loss_margin_m = 0.004;
  config.min_predicted_contact_force_n = 0.0;
  config.target_predicted_normal_force_n = 0.0;
  config.good_contact_gap_min_m = -0.001;
  config.deep_contact_scale_m = 0.002;
  config.deep_contact_weight = 5.0;
  config.target_edge_margin_m = 0.0;

  const auto evaluation =
      mppi_core::EvaluateObjectContactSupport(state, context, config);

  ASSERT_TRUE(evaluation.valid);
  EXPECT_LT(evaluation.min_signed_distance_m, config.good_contact_gap_min_m);
  EXPECT_GT(evaluation.penetration_cost, 0.0);
  EXPECT_LT(evaluation.penetration_cost, 1.0e-3);
}

TEST(RobustGraspStateCostTest,
     MeasuredTactileForceStillProvidesPreloadCost) {
  mppi_core::RobustGraspStateCostConfig config;
  config.min_active_tactile_sensors = 0;
  config.min_active_hemisphere_total = 0;
  config.contact_loss_weight = 0.0;
  config.support_weight = 0.0;
  config.target_normal_force_n = 1.0;
  config.min_normal_force_per_sensor_n = 0.1;
  config.max_normal_force_per_sensor_n = 5.0;
  config.force_low_weight = 10.0;
  config.force_high_weight = 0.0;
  config.force_balance_weight = 0.0;
  config.shear_weight = 0.0;
  config.rotation_weight = 0.0;
  config.slip_score_weight = 0.0;
  config.enable_contact_line_alignment = false;
  config.qddot_weight = 0.0;
  config.tau_weight = 0.0;
  config.object_support.enabled = false;

  mppi_core::TactileState weak_tactile;
  weak_tactile.valid = true;
  weak_tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  weak_tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 0.05, true));
  weak_tactile.total_force_n.z() =
      weak_tactile.activeHemisphereNormalForceN();

  mppi_core::TactileState target_tactile = weak_tactile;
  target_tactile.hemispheres[0].normal_force_n = 1.0;
  target_tactile.total_force_n.z() =
      target_tactile.activeHemisphereNormalForceN();

  const auto weak_state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), weak_tactile, target_tactile);
  const auto target_state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), target_tactile, target_tactile);

  mppi_core::RobustGraspStateCost cost(config);
  mppi_core::RolloutContext context;
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  mppi_core::RobustGraspStateCostBreakdown weak_breakdown;
  mppi_core::RobustGraspStateCostBreakdown target_breakdown;

  const double weak_cost =
      cost.Evaluate(weak_state, action, context, &weak_breakdown);
  const double target_cost =
      cost.Evaluate(target_state, action, context, &target_breakdown);

  EXPECT_GT(weak_breakdown.preload_cost, 0.0);
  EXPECT_GT(weak_breakdown.force_cost, target_breakdown.force_cost);
  EXPECT_GT(weak_cost, target_cost);
  EXPECT_NEAR(target_breakdown.force_cost, 0.0, kTolerance);
  EXPECT_FALSE(weak_breakdown.object_support.valid);
}

TEST(GraspDisturbanceSamplerTest,
     ZeroObjectDisturbanceConfigProducesZeroSamples) {
  mppi_core::GraspDisturbanceSamplerConfig config;
  config.num_disturbance_rollouts = 3;
  config.horizon_steps = 2;
  config.tactile_sensor_count = 1;
  config.tangent_velocity_std_mps = 0.0;
  config.tangent_velocity_max_mps = 0.0;
  config.rotational_velocity_std_radps = 0.0;
  config.rotational_velocity_max_radps = 0.0;
  config.normal_force_rate_std_nps = 0.0;
  config.normal_force_rate_max_nps = 0.0;
  config.cop_drift_velocity_std_mps = 0.0;
  config.cop_drift_velocity_max_mps = 0.0;
  config.sensor_local_noise_scale = 0.0;
  config.friction_scale_std = 0.0;
  config.friction_scale_min = 1.0;
  config.friction_scale_max = 1.0;

  mppi_core::GraspDisturbanceSampler sampler(config);
  const auto batch = sampler.SampleBatch();

  ASSERT_EQ(batch.size(), 3U);
  for (const auto& sequence : batch) {
    ASSERT_EQ(sequence.steps.size(), 2U);
    for (const auto& step : sequence.steps) {
      EXPECT_TRUE(step.object_disturbance.linear_velocity_world_mps.isZero(0.0));
      EXPECT_TRUE(step.object_disturbance.angular_velocity_world_radps.isZero(0.0));
      EXPECT_TRUE(step.object_disturbance.position_offset_world_m.isZero(0.0));
      EXPECT_TRUE(step.object_disturbance.rpy_offset_world_rad.isZero(0.0));
    }
  }
}

TEST(GraspDisturbanceSamplerTest,
     ObjectDisturbanceSamplesAreSeededAndNonzeroWhenEnabled) {
  mppi_core::GraspDisturbanceSamplerConfig config;
  config.num_disturbance_rollouts = 4;
  config.horizon_steps = 1;
  config.tactile_sensor_count = 1;
  config.random_seed = 42;
  config.object.linear_velocity_std_mps = 0.01;
  config.object.linear_velocity_max_mps = 0.02;
  config.object.angular_velocity_std_radps = 0.1;
  config.object.angular_velocity_max_radps = 0.2;

  mppi_core::GraspDisturbanceSampler first(config);
  mppi_core::GraspDisturbanceSampler second(config);
  const auto first_batch = first.SampleBatch();
  const auto second_batch = second.SampleBatch();

  ASSERT_EQ(first_batch.size(), second_batch.size());
  bool saw_nonzero = false;
  bool saw_difference = false;
  for (std::size_t i = 0; i < first_batch.size(); ++i) {
    const auto& a = first_batch[i].steps[0].object_disturbance;
    const auto& b = second_batch[i].steps[0].object_disturbance;
    EXPECT_TRUE(a.linear_velocity_world_mps.isApprox(
        b.linear_velocity_world_mps, kTolerance));
    EXPECT_TRUE(a.angular_velocity_world_radps.isApprox(
        b.angular_velocity_world_radps, kTolerance));
    saw_nonzero = saw_nonzero ||
                  a.linear_velocity_world_mps.norm() > 0.0 ||
                  a.angular_velocity_world_radps.norm() > 0.0;
    if (i > 0) {
      const auto& prev = first_batch[i - 1].steps[0].object_disturbance;
      saw_difference = saw_difference ||
                       !a.linear_velocity_world_mps.isApprox(
                           prev.linear_velocity_world_mps, kTolerance) ||
                       !a.angular_velocity_world_radps.isApprox(
                           prev.angular_velocity_world_radps, kTolerance);
    }
  }
  EXPECT_TRUE(saw_nonzero);
  EXPECT_TRUE(saw_difference);
}

TEST(ContinuousQddotMppiTest, SoftWeightsPreferLowerCostAndSumToOne) {
  const std::vector<double> weights =
      mppi_core::ComputeSoftMppiWeights({10.0, 0.0, 2.0}, 1.0);

  ASSERT_EQ(weights.size(), 3U);
  const double sum = weights[0] + weights[1] + weights[2];
  EXPECT_NEAR(sum, 1.0, kTolerance);
  EXPECT_GT(weights[1], weights[2]);
  EXPECT_GT(weights[2], weights[0]);

  const std::vector<double> sharp =
      mppi_core::ComputeSoftMppiWeights({0.0, 1.0}, 0.1);
  const std::vector<double> smooth =
      mppi_core::ComputeSoftMppiWeights({0.0, 1.0}, 10.0);
  EXPECT_GT(sharp[0] - sharp[1], smooth[0] - smooth[1]);
}

TEST(ContinuousQddotMppiTest, StepRobotStateEnforcesQdotLimit) {
  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, 0.9),
      Eigen::VectorXd::Zero(1));
  mppi_core::QddotRolloutLimits limits;
  limits.qdot_lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  limits.qdot_upper_bound = Eigen::VectorXd::Constant(1, 1.0);

  const auto next = mppi_core::StepRobotStateWithQddotLimits(
      robot, Eigen::VectorXd::Constant(1, 10.0), nullptr, 0.1, limits);

  ASSERT_TRUE(mppi_core::IsValid(next));
  EXPECT_NEAR(next.qdot[0], 1.0, kTolerance);
  EXPECT_NEAR(next.q[0], 0.1, kTolerance);
}

TEST(ContinuousQddotMppiTest, StepRobotStateEnforcesPositionLimit) {
  auto sensor_model = MakeSinglePrismaticZSensorModel();
  sensor_model.model.lowerPositionLimit[0] = -0.05;
  sensor_model.model.upperPositionLimit[0] = 0.05;
  mppi_core::RobotSystem robot_system(sensor_model.model);
  const auto robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Constant(1, 0.04), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1));
  mppi_core::QddotRolloutLimits limits;
  limits.qdot_lower_bound = Eigen::VectorXd::Constant(1, -10.0);
  limits.qdot_upper_bound = Eigen::VectorXd::Constant(1, 10.0);

  const auto next = mppi_core::StepRobotStateWithQddotLimits(
      robot, Eigen::VectorXd::Constant(1, 10.0), &robot_system, 0.1, limits);

  ASSERT_TRUE(mppi_core::IsValid(next));
  EXPECT_NEAR(next.qdot[0], 1.0, kTolerance);
  EXPECT_NEAR(next.q[0], 0.05, kTolerance);
}

TEST(ContinuousQddotMppiTest, RneaFeedforwardDisabledReturnsZeroTorque) {
  const auto sensor_model = MakeSinglePrismaticZSensorModelWithInertia();
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::GraspObservation observation;
  observation.q_meas = Eigen::VectorXd::Zero(sensor_model.model.nq);
  observation.qdot_meas = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.q_ref_current = observation.q_meas;
  observation.qdot_ref_current = observation.qdot_meas;
  observation.robot_system = &robot_system;
  const auto state = mppi_core::MakeGraspState(
      observation.q_meas, observation.qdot_meas,
      Eigen::VectorXd::Zero(sensor_model.model.nv), tactile, tactile);
  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  const Eigen::VectorXd qddot =
      Eigen::VectorXd::Constant(sensor_model.model.nv, 0.25);
  const Eigen::VectorXd previous;

  mppi_core::RneaFeedforwardConfig config;
  config.enabled = false;
  const auto result = mppi_core::ComputeRneaFeedforwardCommand(
      config, observation, state, context, qddot, previous, false, 0.01);

  EXPECT_TRUE(result.raw.isZero(kTolerance));
  EXPECT_TRUE(result.scaled.isZero(kTolerance));
  EXPECT_TRUE(result.command.isZero(kTolerance));
}

TEST(ContinuousQddotMppiTest, RneaFeedforwardScalesAndClampsTorque) {
  const auto sensor_model = MakeSinglePrismaticZSensorModelWithInertia();
  pinocchio::Data expected_data(sensor_model.model);
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::GraspObservation observation;
  observation.q_meas = Eigen::VectorXd::Zero(sensor_model.model.nq);
  observation.qdot_meas = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.q_ref_current = observation.q_meas;
  observation.qdot_ref_current = observation.qdot_meas;
  observation.robot_system = &robot_system;
  const auto state = mppi_core::MakeGraspState(
      observation.q_meas, observation.qdot_meas,
      Eigen::VectorXd::Zero(sensor_model.model.nv), tactile, tactile);
  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  const Eigen::VectorXd qddot =
      Eigen::VectorXd::Constant(sensor_model.model.nv, 0.25);
  const Eigen::VectorXd previous;

  mppi_core::RneaFeedforwardConfig config;
  config.enabled = true;
  config.tau_ff_scale = 0.5;
  config.max_tau_ff_nm = 100.0;
  config.max_tau_ff_rate_nm_s = 1000.0;
  auto result = mppi_core::ComputeRneaFeedforwardCommand(
      config, observation, state, context, qddot, previous, false, 0.01);
  const Eigen::VectorXd expected_raw =
      pinocchio::rnea(sensor_model.model, expected_data,
                      observation.q_meas, observation.qdot_meas, qddot);

  ASSERT_EQ(result.raw.size(), expected_raw.size());
  EXPECT_TRUE(result.raw.isApprox(expected_raw, kTolerance));
  EXPECT_TRUE(result.scaled.isApprox(0.5 * expected_raw, kTolerance));
  EXPECT_TRUE(result.command.isApprox(result.scaled, kTolerance));
  EXPECT_FALSE(result.clamped);

  config.max_tau_ff_nm = 0.05;
  result = mppi_core::ComputeRneaFeedforwardCommand(
      config, observation, state, context, qddot, previous, false, 0.01);
  EXPECT_TRUE(result.clamped);
  EXPECT_LE(result.command.cwiseAbs().maxCoeff(), 0.05 + kTolerance);
}

TEST(ContinuousQddotMppiTest, RneaFeedforwardRateLimitsTorque) {
  const auto sensor_model = MakeSinglePrismaticZSensorModelWithInertia();
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::GraspObservation observation;
  observation.q_meas = Eigen::VectorXd::Zero(sensor_model.model.nq);
  observation.qdot_meas = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.q_ref_current = observation.q_meas;
  observation.qdot_ref_current = observation.qdot_meas;
  observation.robot_system = &robot_system;
  const auto state = mppi_core::MakeGraspState(
      observation.q_meas, observation.qdot_meas,
      Eigen::VectorXd::Zero(sensor_model.model.nv), tactile, tactile);
  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;

  mppi_core::RneaFeedforwardConfig config;
  config.enabled = true;
  config.tau_ff_scale = 1.0;
  config.max_tau_ff_nm = 100.0;
  config.max_tau_ff_rate_nm_s = 1.0;
  const Eigen::VectorXd qddot =
      Eigen::VectorXd::Constant(sensor_model.model.nv, 1.0);
  const Eigen::VectorXd previous =
      Eigen::VectorXd::Zero(sensor_model.model.nv);

  const auto result = mppi_core::ComputeRneaFeedforwardCommand(
      config, observation, state, context, qddot, previous, true, 0.01);

  EXPECT_TRUE(result.rate_limited);
  EXPECT_LE(result.command.cwiseAbs().maxCoeff(), 0.01 + kTolerance);
}

TEST(ContinuousQddotMppiTest, RneaFeedforwardZerosForSafetyConditions) {
  const auto sensor_model = MakeSinglePrismaticZSensorModelWithInertia();
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::TactileState no_contact = tactile;
  for (auto& hemisphere : no_contact.hemispheres) {
    hemisphere.contact = false;
    hemisphere.normal_force_n = 0.0;
  }
  no_contact.contact_state = mppi_core::TactileState::kNoContact;
  no_contact.total_force_n.setZero();

  mppi_core::GraspObservation observation;
  observation.q_meas = Eigen::VectorXd::Zero(sensor_model.model.nq);
  observation.qdot_meas = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.q_ref_current = observation.q_meas;
  observation.qdot_ref_current = observation.qdot_meas;
  observation.robot_system = &robot_system;
  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  mppi_core::RneaFeedforwardConfig config;
  config.enabled = true;

  auto not_ready = mppi_core::MakeGraspState(
      observation.q_meas, observation.qdot_meas,
      Eigen::VectorXd::Zero(sensor_model.model.nv), tactile, tactile);
  not_ready.valid = false;
  const Eigen::VectorXd qddot =
      Eigen::VectorXd::Constant(sensor_model.model.nv, 1.0);
  const Eigen::VectorXd previous;
  auto result = mppi_core::ComputeRneaFeedforwardCommand(
      config, observation, not_ready, context, qddot, previous, false, 0.01);
  EXPECT_TRUE(result.zeroed_not_ready);
  EXPECT_TRUE(result.command.isZero(kTolerance));

  const auto contact_lost = mppi_core::MakeGraspState(
      observation.q_meas, observation.qdot_meas,
      Eigen::VectorXd::Zero(sensor_model.model.nv), no_contact, no_contact);
  result = mppi_core::ComputeRneaFeedforwardCommand(
      config, observation, contact_lost, context, qddot, previous, false,
      0.01);
  EXPECT_TRUE(result.zeroed_contact_loss);
  EXPECT_TRUE(result.command.isZero(kTolerance));
}

TEST(ObjectPriorGraspRolloutTest,
     StepVirtualObjectBeliefAppliesLinearAndAngularDisturbance) {
  const auto belief = MakeTestObjectBelief();

  mppi_core::VirtualObjectDisturbance disturbance;
  disturbance.linear_velocity_world_mps = Eigen::Vector3d{0.1, -0.2, 0.3};
  disturbance.angular_velocity_world_radps = Eigen::Vector3d{0.0, 0.0, 1.0};

  const auto next =
      mppi_core::StepVirtualObjectBelief(belief, disturbance, 0.5);

  ASSERT_TRUE(mppi_core::IsValidVirtualObjectBelief(next));
  ASSERT_EQ(next.particles.size(), belief.particles.size());
  EXPECT_TRUE(next.particles[0].pose_world.translation().isApprox(
      belief.particles[0].pose_world.translation() +
          Eigen::Vector3d{0.05, -0.1, 0.15},
      kTolerance));
  EXPECT_GT((next.particles[0].pose_world.linear() -
             belief.particles[0].pose_world.linear()).norm(), 1.0e-6);
  EXPECT_NEAR(next.particles[0].weight, belief.particles[0].weight,
              kTolerance);

  const auto unchanged = mppi_core::StepVirtualObjectBelief(
      belief, mppi_core::VirtualObjectDisturbance{}, 0.5);
  ASSERT_TRUE(mppi_core::IsValidVirtualObjectBelief(unchanged));
  EXPECT_TRUE(unchanged.particles[0].pose_world.matrix().isApprox(
      belief.particles[0].pose_world.matrix(), kTolerance));
}

TEST(ObjectPriorGraspRolloutTest,
     StepVirtualObjectBeliefIgnoresDeprecatedPoseOffsets) {
  const auto belief = MakeTestObjectBelief();

  mppi_core::VirtualObjectDisturbance disturbance;
  disturbance.position_offset_world_m = Eigen::Vector3d{1.0, 2.0, 3.0};
  disturbance.rpy_offset_world_rad = Eigen::Vector3d{0.5, 0.4, 0.3};

  const auto next =
      mppi_core::StepVirtualObjectBelief(belief, disturbance, 0.5);

  ASSERT_TRUE(mppi_core::IsValidVirtualObjectBelief(next));
  ASSERT_EQ(next.particles.size(), belief.particles.size());
  EXPECT_TRUE(next.particles[0].pose_world.matrix().isApprox(
      belief.particles[0].pose_world.matrix(), kTolerance));
}

TEST(ObjectPriorGraspRolloutTest,
     MainObjectPriorRolloutUpdatesRobotAndObjectWithoutTactileOnlyTransition) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));

  const auto state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1),
      std::vector<mppi_core::TactileState,
                  Eigen::aligned_allocator<mppi_core::TactileState>>{tactile},
      MakeTestObjectBelief());

  mppi_core::GraspDisturbanceStep disturbance;
  disturbance.object_disturbance.linear_velocity_world_mps =
      Eigen::Vector3d{0.2, 0.0, 0.0};
  disturbance.tactile_sensor_disturbances.resize(1);

  mppi_core::RolloutContext context;
  const auto result = mppi_core::StepObjectPriorGraspState(
      state, Eigen::VectorXd::Constant(1, 0.4), disturbance, context, 0.1);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.next_state.robot.qdot[0], 0.04, kTolerance);
  EXPECT_NEAR(result.next_state.robot.q[0], 0.004, kTolerance);
  EXPECT_TRUE(result.next_state.object_belief.particles[0]
                  .pose_world.translation()
                  .isApprox(state.object_belief.particles[0]
                                .pose_world.translation() +
                            Eigen::Vector3d{0.02, 0.0, 0.0},
                            kTolerance));
  EXPECT_EQ(result.next_state.tactile_sensors[0].activeHemisphereCount(), 1U);
}

TEST(RobustGraspPolicyTest, StableCenteredObjectContactKeepsContinuousMppiNearZero) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data first_data(sensor_model.model);
  pinocchio::Data second_data(sensor_model.model);

  mppi_core::PinocchioContactKinematicsContext first_kinematics;
  first_kinematics.model = &sensor_model.model;
  first_kinematics.data = &first_data;
  first_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  first_kinematics.normal_axis_sign = -1.0;

  mppi_core::PinocchioContactKinematicsContext second_kinematics;
  second_kinematics.model = &sensor_model.model;
  second_kinematics.data = &second_data;
  second_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  second_kinematics.normal_axis_sign = -1.0;

  mppi_core::TactileState thumb;
  thumb.valid = true;
  thumb.sensor_index = 0;
  thumb.contact_state = mppi_core::TactileState::kEnoughContacts;
  thumb.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  thumb.total_force_n.z() = thumb.activeHemisphereNormalForceN();

  mppi_core::TactileState index = thumb;
  index.sensor_index = 1;

  const auto prior = MakeTestBoxObjectPrior(Eigen::Vector3d::Zero());
  mppi_core::VirtualObjectBelief belief;
  belief.geometry = prior.geometry;
  belief.particles.push_back(mppi_core::VirtualObjectState{});
  belief.particles[0].pose_world.setIdentity();
  belief.particles[0].weight = 1.0;
  belief.particles[0].valid = true;
  belief.valid = true;

  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Constant(1, 0.05);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(1);
  observation.tactile_meas = MakeTactileSensors(thumb, index);
  observation.object_prior = prior;
  observation.object_belief = belief;
  observation.robot_system = &robot_system;
  observation.tactile_contexts = MakeTactileContextsForStates(
      &first_kinematics, thumb, &second_kinematics, index);

  mppi_core::RobustGraspPolicyConfig config;
  config.control_mode = mppi_core::RobustGraspControlMode::kContinuousQddotMppi;
  config.rollout.horizon_steps = 2;
  config.rollout.num_rollouts = 4;
  config.rollout.dt = 0.01;
  config.rollout.action_dim = 1;
  config.rollout.temperature = 1.0;
  config.rollout.action_lower_bound = Eigen::VectorXd::Constant(1, -10.0);
  config.rollout.action_upper_bound = Eigen::VectorXd::Constant(1, 10.0);
  config.rollout.action_noise_std = Eigen::VectorXd::Zero(1);
  config.rollout.action_noise_clip = Eigen::VectorXd::Zero(1);
  config.rollout.qdot_lower_bound = Eigen::VectorXd::Constant(1, -10.0);
  config.rollout.qdot_upper_bound = Eigen::VectorXd::Constant(1, 10.0);
  config.disturbance_sampler.num_disturbance_rollouts = 1;
  config.disturbance_sampler.horizon_steps = 2;
  config.disturbance_sampler.tactile_sensor_count = 2;
  config.disturbance_sampler.tangent_velocity_std_mps = 0.0;
  config.disturbance_sampler.tangent_velocity_max_mps = 0.0;
  config.disturbance_sampler.rotational_velocity_std_radps = 0.0;
  config.disturbance_sampler.rotational_velocity_max_radps = 0.0;
  config.disturbance_sampler.normal_force_rate_std_nps = 0.0;
  config.disturbance_sampler.normal_force_rate_max_nps = 0.0;
  config.disturbance_sampler.cop_drift_velocity_std_mps = 0.0;
  config.disturbance_sampler.cop_drift_velocity_max_mps = 0.0;
  config.disturbance_sampler.sensor_local_noise_scale = 0.0;
  config.disturbance_sampler.friction_scale_std = 0.0;
  config.disturbance_sampler.friction_scale_min = 1.0;
  config.disturbance_sampler.friction_scale_max = 1.0;
  config.start.min_enough_contact_sensors = 2;
  config.start.min_active_hemispheres_total = 2;
  config.cost.min_active_tactile_sensors = 2;
  config.cost.min_active_hemisphere_total = 2;
  config.cost.target_normal_force_n = 1.0;
  config.cost.min_normal_force_per_sensor_n = 0.1;
  config.cost.max_normal_force_per_sensor_n = 5.0;
  config.cost.qddot_weight = 1.0;
  config.cost.object_support.max_object_samples = 1;
  config.cost.object_support.min_active_tactile_sensors = 2;
  config.cost.object_support.min_active_hemisphere_total = 2;
  config.cost.object_support.contact_birth_margin_m = 0.002;
  config.cost.object_support.contact_loss_margin_m = 0.004;
  config.cost.object_support.target_predicted_normal_force_n = 1.0;
  config.cost.object_support.min_predicted_contact_force_n = 0.001;
  config.cost.object_support.target_edge_margin_m = 0.0;
  config.action_library.num_action_samples = 1;
  config.action_library.include_basis_probe_actions = true;
  config.action_library.target_min_normal_force_n = 1.0;
  config.action_library.max_safe_normal_force_n = 5.0;
  config.safe_hold_score_threshold = 0.0;
  config.min_required_score_improvement = 0.0;
  config.action_rate_weight = 0.0;
  config.continuous_control_rate_cost_weight = 0.0;
  config.continuous_smoothing_alpha = 1.0;

  mppi_core::RobustGraspPolicy policy;
  policy.Initialize(config);
  const auto command = policy.Update(observation);
  const auto& status = policy.status();

  EXPECT_TRUE(command.IsUsable());
  EXPECT_TRUE(status.used_continuous_qddot_mppi);
  EXPECT_EQ(status.best_action_name, "continuous_qddot_mppi_weighted");
  EXPECT_EQ(status.candidate_count, 4U);
  EXPECT_EQ(status.horizon_steps, 2U);
  EXPECT_NEAR(status.effective_sample_size, 4.0, kTolerance);
  EXPECT_EQ(status.selected_qddot.size(), 1);
  EXPECT_NEAR(status.selected_qddot[0], 0.0, kTolerance);
  EXPECT_EQ(status.qddot_cmd.size(), 1);
  EXPECT_NEAR(status.qddot_cmd[0], 0.0, kTolerance);
  EXPECT_EQ(status.qddot_best_first.size(), 1);
  EXPECT_NEAR(status.qddot_best_first[0], 0.0, kTolerance);
  EXPECT_GT(status.selected_object_geometry_query_count, 0U);
  EXPECT_GT(status.selected_object_sample_count, 0U);
  EXPECT_TRUE(status.selected_predicted_centroid_sensor_m.allFinite());
  EXPECT_TRUE(status.selected_measured_centroid_sensor_m.allFinite());
  EXPECT_GE(status.solve_time_ms, 0.0);
}

TEST(PlatoNariTouchUrdfTest, RobotSystemLoadsTactileFrames) {
  const auto urdf_path = PlatoNariTouchUrdfPath();
  ASSERT_TRUE(std::filesystem::exists(urdf_path)) << urdf_path;

  const mppi_core::RobotSystem robot_system(urdf_path.string());
  ASSERT_TRUE(robot_system.hasModel());
  EXPECT_EQ(robot_system.nq(), 8);
  EXPECT_EQ(robot_system.nv(), 8);
  EXPECT_EQ(robot_system.state().q.size(), robot_system.nq());
  EXPECT_EQ(robot_system.state().qdot.size(), robot_system.nv());
  EXPECT_EQ(robot_system.state().tau.size(), robot_system.nv());

  const auto& model = robot_system.model();
  for (const char* frame_name :
       {"thumb_distal_tactile", "index_distal_tactile",
        "middle_distal_tactile"}) {
    EXPECT_LT(model.getFrameId(frame_name), model.frames.size())
        << frame_name;
  }
  for (const char* joint_name : {"joint1", "joint6", "joint8"}) {
    EXPECT_LT(model.getJointId(joint_name), model.joints.size()) << joint_name;
  }
}

TEST(PlatoNariTouchUrdfTest,
     IndexTactilePointJacobianMatchesFiniteDifference) {
  const auto urdf_path = PlatoNariTouchUrdfPath();
  ASSERT_TRUE(std::filesystem::exists(urdf_path)) << urdf_path;

  const mppi_core::RobotSystem robot_system(urdf_path.string());
  const auto& model = robot_system.model();
  const pinocchio::FrameIndex sensor_frame_id =
      model.getFrameId("index_distal_tactile");
  ASSERT_LT(sensor_frame_id, model.frames.size());
  const pinocchio::JointIndex joint_id = model.getJointId("joint6");
  ASSERT_LT(joint_id, model.joints.size());

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.sensor_index = 0;
  tactile.frame_name = "index_distal_tactile";
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d{1.0e-3, -0.5e-3}, 1.0, true));
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  const auto robot = mppi_core::MakeRobotState(
      pinocchio::neutral(model), Eigen::VectorXd::Zero(model.nv),
      Eigen::VectorXd::Zero(model.nv));
  Eigen::VectorXd tangent_step = Eigen::VectorXd::Zero(model.nv);
  tangent_step[model.joints[joint_id].idx_v()] = 0.2;

  pinocchio::Data data(model);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &model;
  context.data = &data;
  context.sensor_frame_id = sensor_frame_id;

  const auto motions = mppi_core::ComputeHemisphereMotions(
      robot, tactile, tangent_step, context);

  ASSERT_EQ(motions.size(), 1U);
  EXPECT_EQ(motions[0].J_contact_world.rows(), 3);
  EXPECT_EQ(motions[0].J_contact_world.cols(), model.nv);
  EXPECT_EQ(motions[0].J_normal.size(), model.nv);
  EXPECT_TRUE(motions[0].J_contact_world.allFinite());
  EXPECT_TRUE(motions[0].J_normal.allFinite());
  EXPECT_GT(motions[0].delta_position_sensor_m.norm(), 1.0e-6);

  pinocchio::Data finite_difference_data(model);
  pinocchio::forwardKinematics(model, finite_difference_data, robot.q);
  pinocchio::updateFramePlacements(model, finite_difference_data);
  const Eigen::Matrix3d current_sensor_rotation =
      finite_difference_data.oMf[sensor_frame_id].rotation();
  const Eigen::Vector3d point_world_before = WorldPointPosition(
      model, &finite_difference_data, sensor_frame_id, robot.q,
      HemispherePointSensorM(tactile.hemispheres[0]));

  const double eps = 1.0e-6;
  const Eigen::VectorXd q_next =
      pinocchio::integrate(model, robot.q, eps * tangent_step);
  const Eigen::Vector3d point_world_after = WorldPointPosition(
      model, &finite_difference_data, sensor_frame_id, q_next,
      HemispherePointSensorM(tactile.hemispheres[0]));
  const Eigen::Vector3d finite_difference_delta_sensor =
      current_sensor_rotation.transpose() *
      (point_world_after - point_world_before);

  EXPECT_NEAR((eps * motions[0].delta_position_sensor_m -
               finite_difference_delta_sensor)
                  .norm(),
              0.0, 1.0e-8);
}

TEST(GraspContactKinematicsTest, EmptyOrInactiveHemispheresReturnEmpty) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd tangent_step = Eigen::VectorXd::Constant(1, 0.1);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  EXPECT_TRUE(mppi_core::ComputeHemisphereMotions(
                  state.robot, state.tactile_sensors[0], tangent_step, context)
                  .empty());

  mppi_core::HemisphereState inactive;
  inactive.contact = false;
  inactive.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(inactive);
  const auto inactive_state =
      MakeContactKinematicsState(sensor_model.model, tactile);
  EXPECT_TRUE(mppi_core::ComputeHemisphereMotions(
                  inactive_state.robot, inactive_state.tactile_sensors[0],
                  tangent_step, context)
                  .empty());
}

TEST(GraspContactKinematicsTest, ActiveHemisphereProducesSensorFrameMotion) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = 7;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd tangent_step = Eigen::VectorXd::Constant(1, 0.25);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  const auto motions = mppi_core::ComputeHemisphereMotions(
      state.robot, state.tactile_sensors[0], tangent_step, context);

  ASSERT_EQ(motions.size(), 1U);
  EXPECT_EQ(motions[0].hemisphere_index, 7U);
  EXPECT_NEAR(motions[0].position_sensor_m.x(), 1.0, kTolerance);
  EXPECT_NEAR(motions[0].position_sensor_m.y(), 0.0, kTolerance);
  EXPECT_NEAR(motions[0].position_sensor_m.z(), 0.0, kTolerance);
  EXPECT_NEAR(motions[0].delta_position_sensor_m.x(), 0.0, kTolerance);
  EXPECT_NEAR(motions[0].delta_position_sensor_m.y(), 0.25, kTolerance);
  EXPECT_NEAR(motions[0].delta_position_sensor_m.z(), 0.0, kTolerance);
}

TEST(GraspContactKinematicsTest, PointJacobianMatchesFiniteDifference) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);
  pinocchio::Data finite_difference_data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{0.7, -0.2};
  tactile.hemispheres.push_back(point);

  auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  state.robot.q[0] = 0.35;
  const double eps = 1.0e-6;
  const Eigen::VectorXd tangent_step = Eigen::VectorXd::Constant(1, 0.7);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  const auto motions = mppi_core::ComputeHemisphereMotions(
      state.robot, state.tactile_sensors[0], tangent_step, context);

  ASSERT_EQ(motions.size(), 1U);
  pinocchio::forwardKinematics(sensor_model.model, finite_difference_data,
                               state.robot.q);
  pinocchio::updateFramePlacements(sensor_model.model, finite_difference_data);
  const Eigen::Matrix3d current_sensor_rotation =
      finite_difference_data.oMf[sensor_model.sensor_frame_id].rotation();
  const Eigen::Vector3d point_world_before = WorldPointPosition(
      sensor_model.model, &finite_difference_data, sensor_model.sensor_frame_id,
      state.robot.q, HemispherePointSensorM(point));
  const Eigen::VectorXd q_next = pinocchio::integrate(
      sensor_model.model, state.robot.q, eps * tangent_step);
  const Eigen::Vector3d point_world_after = WorldPointPosition(
      sensor_model.model, &finite_difference_data, sensor_model.sensor_frame_id,
      q_next, HemispherePointSensorM(point));
  const Eigen::Vector3d finite_difference_delta_sensor =
      current_sensor_rotation.transpose() *
      (point_world_after - point_world_before);

  EXPECT_NEAR((eps * motions[0].delta_position_sensor_m -
               finite_difference_delta_sensor)
                  .norm(),
              0.0, 1.0e-9);
}

TEST(GraspContactKinematicsTest,
     NormalAxisSignKeepsPositiveZAsClosingConvention) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd tangent_step = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  const auto closing_positive = mppi_core::ComputeHemisphereMotions(
      state.robot, state.tactile_sensors[0], tangent_step, context);
  ASSERT_EQ(closing_positive.size(), 1U);
  EXPECT_GT(closing_positive[0].delta_position_sensor_m.z(), 0.0);

  const Eigen::VectorXd opening_tangent_step =
      Eigen::VectorXd::Constant(1, -0.01);
  const auto opening_negative = mppi_core::ComputeHemisphereMotions(
      state.robot, state.tactile_sensors[0], opening_tangent_step, context);
  ASSERT_EQ(opening_negative.size(), 1U);
  EXPECT_LT(opening_negative[0].delta_position_sensor_m.z(), 0.0);

  context.normal_axis_sign = -1.0;
  const auto closing_flipped = mppi_core::ComputeHemisphereMotions(
      state.robot, state.tactile_sensors[0], tangent_step, context);
  ASSERT_EQ(closing_flipped.size(), 1U);
  EXPECT_LT(closing_flipped[0].delta_position_sensor_m.z(), 0.0);
}

TEST(GraspContactKinematicsTest,
     NormalAxisSignControlsNormalVelocityConvention) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  auto sensor_context = MakeTactileContextForState(tactile);
  sensor_context.kinematics = &kinematics;

  const auto robot = MakeContactKinematicsState(sensor_model.model, tactile)
                         .robot;
  const auto next_robot = mppi_core::MakeRobotState(
      pinocchio::integrate(sensor_model.model, robot.q,
                           Eigen::VectorXd::Constant(sensor_model.model.nv,
                                                     0.01)),
      Eigen::VectorXd::Zero(sensor_model.model.nv),
      Eigen::VectorXd::Zero(sensor_model.model.nv));

  const auto closing_positive = mppi_core::ComputeHemisphereMotions(
      robot, next_robot, tactile, sensor_context, 0.1);
  ASSERT_EQ(closing_positive.size(), 1U);
  EXPECT_GT(closing_positive[0].normal_velocity_mps, 0.0);

  kinematics.normal_axis_sign = -1.0;
  const auto closing_flipped = mppi_core::ComputeHemisphereMotions(
      robot, next_robot, tactile, sensor_context, 0.1);
  ASSERT_EQ(closing_flipped.size(), 1U);
  EXPECT_LT(closing_flipped[0].normal_velocity_mps, 0.0);
}

TEST(GraspContactKinematicsTest, ZeroTangentStepProducesZeroMotion) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd tangent_step = Eigen::VectorXd::Zero(1);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  const auto motions = mppi_core::ComputeHemisphereMotions(
      state.robot, state.tactile_sensors[0], tangent_step, context);

  ASSERT_EQ(motions.size(), 1U);
  EXPECT_NEAR(motions[0].delta_position_sensor_m.norm(), 0.0, kTolerance);
}

TEST(GraspContactKinematicsTest, InvalidDimensionsReturnEmpty) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  const auto state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Zero(2), tactile, tactile);
  const Eigen::VectorXd tangent_step = Eigen::VectorXd::Zero(1);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  EXPECT_TRUE(mppi_core::ComputeHemisphereMotions(
                  state.robot, state.tactile_sensors[0], tangent_step, context)
                  .empty());

  const auto valid_state =
      MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd wrong_tangent_step = Eigen::VectorXd::Zero(2);
  EXPECT_TRUE(mppi_core::ComputeHemisphereMotions(
                  valid_state.robot, valid_state.tactile_sensors[0],
                  wrong_tangent_step, context)
                  .empty());
}

TEST(ContactForceProjectionTest, EmptyOrInvalidInputsReturnInvalidResult) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);

  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  const Eigen::VectorXd tau_residual = Eigen::VectorXd::Constant(1, 1.0);
  const auto no_contacts = mppi_core::ProjectContactForcesFromTorqueResidual(
      state.robot, state.tactile_sensors[0], tau_residual, context);
  EXPECT_FALSE(no_contacts.valid);
  EXPECT_TRUE(no_contacts.hemisphere_forces.empty());

  const Eigen::VectorXd wrong_tau = Eigen::VectorXd::Zero(2);
  const auto wrong_dimension =
      mppi_core::ProjectContactForcesFromTorqueResidual(
          state.robot, state.tactile_sensors[0], wrong_tau, context);
  EXPECT_FALSE(wrong_dimension.valid);
}

TEST(ContactForceProjectionTest, PrismaticNormalResidualProjectsToNormalForce) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = 4;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig config;
  config.regularization = 1.0e-9;
  config.tactile_prior_weight = 0.0;

  const Eigen::VectorXd tau_residual = Eigen::VectorXd::Constant(1, 2.0);
  const auto projection = mppi_core::ProjectContactForcesFromTorqueResidual(
      state.robot, state.tactile_sensors[0], tau_residual, context, config);

  ASSERT_TRUE(projection.valid);
  ASSERT_EQ(projection.hemisphere_forces.size(), 1U);
  EXPECT_EQ(projection.hemisphere_forces[0].hemisphere_index, 4U);
  EXPECT_NEAR(projection.hemisphere_forces[0].normal_force_n, 2.0, 1.0e-6);
  EXPECT_NEAR(projection.total_normal_force_n, 2.0, 1.0e-6);
  EXPECT_NEAR(projection.residual_norm, 0.0, 1.0e-6);
}

TEST(ContactForceProjectionTest, TactilePriorRegularizesNormalForce) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  point.normal_force_n = 1.5;
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig config;
  config.regularization = 0.0;
  config.tactile_prior_weight = 1000.0;

  const Eigen::VectorXd tau_residual = Eigen::VectorXd::Zero(1);
  const auto projection = mppi_core::ProjectContactForcesFromTorqueResidual(
      state.robot, state.tactile_sensors[0], tau_residual, context, config);

  ASSERT_TRUE(projection.valid);
  ASSERT_EQ(projection.hemisphere_forces.size(), 1U);
  EXPECT_NEAR(projection.hemisphere_forces[0].normal_force_n,
              1000.0 * 1.5 / 1001.0, 1.0e-6);
}

TEST(ContactForceProjectionTest, NegativeNormalForceCanBeClamped) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig config;
  config.regularization = 1.0e-9;
  config.tactile_prior_weight = 0.0;
  config.clamp_negative_normal_force = true;

  const Eigen::VectorXd tau_residual = Eigen::VectorXd::Constant(1, -1.0);
  const auto projection = mppi_core::ProjectContactForcesFromTorqueResidual(
      state.robot, state.tactile_sensors[0], tau_residual, context, config);

  ASSERT_TRUE(projection.valid);
  ASSERT_EQ(projection.hemisphere_forces.size(), 1U);
  EXPECT_NEAR(projection.hemisphere_forces[0].normal_force_n, 0.0, kTolerance);
  EXPECT_NEAR(projection.total_normal_force_n, 0.0, kTolerance);
}

TEST(ContactForceProjectionTest,
     RevoluteTangentialResidualWithoutNormalForceIsClampedToZero) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig config;
  config.regularization = 1.0e-9;
  config.tactile_prior_weight = 0.0;

  const Eigen::VectorXd tau_residual = Eigen::VectorXd::Constant(1, 2.0);
  const auto projection = mppi_core::ProjectContactForcesFromTorqueResidual(
      state.robot, state.tactile_sensors[0], tau_residual, context, config);

  ASSERT_TRUE(projection.valid);
  ASSERT_EQ(projection.hemisphere_forces.size(), 1U);
  EXPECT_NEAR(projection.hemisphere_forces[0].tangential_force_n.x(), 0.0,
              kTolerance);
  EXPECT_NEAR(projection.hemisphere_forces[0].tangential_force_n.y(), 0.0,
              kTolerance);
  EXPECT_NEAR(projection.net_tangential_force_n.y(), 0.0, kTolerance);
  EXPECT_NEAR(projection.net_torsional_moment_nm, 0.0, kTolerance);
  EXPECT_NEAR(projection.total_friction_violation, 0.0, kTolerance);
}

TEST(ContactForceProjectionTest, FrictionConeClampsTangentialForce) {
  const auto sensor_model = MakePrismaticAndRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  mppi_core::PinocchioContactKinematicsContext context;
  context.model = &sensor_model.model;
  context.data = &data;
  context.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig config;
  config.regularization = 1.0e-9;
  config.tactile_prior_weight = 0.0;
  config.friction_coefficient = 0.5;

  Eigen::VectorXd tau_residual = Eigen::VectorXd::Zero(2);
  tau_residual[0] = 1.0;
  tau_residual[1] = 2.0;
  const auto projection = mppi_core::ProjectContactForcesFromTorqueResidual(
      state.robot, state.tactile_sensors[0], tau_residual, context, config);

  ASSERT_TRUE(projection.valid);
  ASSERT_EQ(projection.hemisphere_forces.size(), 1U);
  EXPECT_NEAR(projection.hemisphere_forces[0].normal_force_n, 1.0, 1.0e-6);
  EXPECT_NEAR(projection.hemisphere_forces[0].tangential_force_n.x(), 0.0,
              kTolerance);
  EXPECT_NEAR(projection.hemisphere_forces[0].tangential_force_n.y(), 0.5,
              1.0e-6);
  EXPECT_NEAR(projection.net_tangential_force_n.y(), 0.5, 1.0e-6);
  EXPECT_NEAR(projection.net_torsional_moment_nm, 0.5, 1.0e-6);
  EXPECT_NEAR(projection.total_friction_violation, 0.0, kTolerance);
}

TEST(ContactForceRolloutTest, ProjectedForceUpdatesPredictedTactileState) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  tactile.shear_displacement_m = Eigen::Vector2d::Zero();
  tactile.rotational_shear_rad = 0.0;
  tactile.confidence = 1.0;
  SetActiveHemispheresAroundCentroid(&tactile, 2, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());

  mppi_core::ContactForceProjectionResult projection;
  projection.valid = true;
  projection.total_normal_force_n = 4.0;
  projection.net_tangential_force_n = Eigen::Vector2d{2.0, 0.0};
  projection.net_torsional_moment_nm = 0.5;

  mppi_core::HemisphereForce first_force;
  first_force.hemisphere_index = 0;
  first_force.normal_force_n = 2.0;
  mppi_core::HemisphereForce second_force;
  second_force.hemisphere_index = 1;
  second_force.normal_force_n = 2.0;
  projection.hemisphere_forces.push_back(first_force);
  projection.hemisphere_forces.push_back(second_force);

  mppi_core::ContactForceRolloutConfig config;
  config.force_lowpass_alpha = 1.0;
  config.shear_force_gain_m_per_n_s = 0.1;
  config.rotational_shear_gain_rad_per_nm_s = 1.0;
  config.min_active_hemisphere_count = 2;
  config.shear_ref_m = 1.0;
  config.rotational_shear_ref_rad = 1.0;

  mppi_core::StepTactileStateFromProjectedForce(projection, 0.1, config,
                                                &tactile);

  EXPECT_EQ(tactile.contact_state, mppi_core::TactileState::kEnoughContacts);
  EXPECT_NEAR(tactile.total_force_n.z(), 4.0, kTolerance);
  EXPECT_NEAR(tactile.shear_displacement_m.x(), 0.02, kTolerance);
  EXPECT_NEAR(tactile.rotational_shear_rad, 0.05, kTolerance);
  EXPECT_NEAR(tactile.incipient_slip_score, 0.07, kTolerance);
  EXPECT_EQ(tactile.activeHemisphereCount(), 2U);
  EXPECT_NEAR(tactile.hemispheres[0].normal_force_n, 2.0, kTolerance);
}

TEST(ContactForceRolloutTest,
     ZeroProjectedNormalForceLosesContactAndDeactivatesPoints) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  tactile.confidence = 1.0;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());

  mppi_core::ContactForceProjectionResult projection;
  projection.valid = true;
  projection.total_normal_force_n = 0.0;
  projection.total_friction_violation = 2.0;
  mppi_core::HemisphereForce force;
  force.hemisphere_index = 0;
  force.normal_force_n = 0.0;
  projection.hemisphere_forces.push_back(force);

  mppi_core::ContactForceRolloutConfig config;
  config.force_lowpass_alpha = 1.0;
  config.negative_normal_confidence_decay = 0.5;
  config.friction_violation_confidence_decay = 0.1;

  mppi_core::StepTactileStateFromProjectedForce(projection, 0.1, config,
                                                &tactile);

  EXPECT_NEAR(tactile.total_force_n.z(), 0.0, kTolerance);
  EXPECT_LT(tactile.confidence, 1.0);
  EXPECT_EQ(tactile.contact_state, mppi_core::TactileState::kNoContact);
  EXPECT_EQ(tactile.activeHemisphereCount(), 0U);
}

TEST(ContactForceCorrectionTest, UpdatesAndClampsNormalForceBias) {
  mppi_core::TactileState measured;
  measured.valid = true;
  measured.contact_state = mppi_core::TactileState::kEnoughContacts;

  mppi_core::ContactForceCorrectionState state;
  mppi_core::ContactForceCorrectionConfig config;
  config.bias_update_rate = 0.1;
  config.max_abs_bias_n = 0.5;

  mppi_core::UpdateContactForceCorrection(1.0, 2.0, measured, config, &state);
  EXPECT_NEAR(state.normal_force_bias_n, 0.1, kTolerance);

  mppi_core::UpdateContactForceCorrection(0.0, 10.0, measured, config, &state);
  EXPECT_NEAR(state.normal_force_bias_n, 0.5, kTolerance);
  EXPECT_NEAR(mppi_core::ApplyContactForceCorrection(1.0, state), 1.5,
              kTolerance);
}

TEST(ContactForceCorrectionTest, SkipsUpdateWithoutValidMeasuredContact) {
  mppi_core::TactileState measured;
  measured.valid = true;
  measured.contact_state = mppi_core::TactileState::kNoContact;

  mppi_core::ContactForceCorrectionState state;
  state.normal_force_bias_n = 0.25;
  mppi_core::ContactForceCorrectionConfig config;
  config.update_only_in_contact = true;

  mppi_core::UpdateContactForceCorrection(1.0, 3.0, measured, config, &state);
  EXPECT_NEAR(state.normal_force_bias_n, 0.25, kTolerance);

  config.enabled = false;
  measured.contact_state = mppi_core::TactileState::kEnoughContacts;
  mppi_core::UpdateContactForceCorrection(1.0, 3.0, measured, config, &state);
  EXPECT_NEAR(state.normal_force_bias_n, 0.25, kTolerance);
}

TEST(NariTouchStateTest, DefaultConstructorInitializesUnitPositions) {
  const mppi_core::NariTouchState tactile;
  const auto positions = mppi_core::NariTouchUnitPositionsM();

  ASSERT_EQ(tactile.units.size(), positions.size());
  for (std::size_t i = 0; i < tactile.units.size(); ++i) {
    EXPECT_NEAR(tactile.units[i].position_m.x(), positions[i].x(), kTolerance);
    EXPECT_NEAR(tactile.units[i].position_m.y(), positions[i].y(), kTolerance);
  }
}

TEST(NariTouchStateTest, ContactGateUsesContactStateAndUnits) {
  mppi_core::NariTouchState tactile;
  EXPECT_FALSE(tactile.hasContact());

  tactile.contact_state = mppi_core::NariTouchContactState::kFewContacts;
  EXPECT_TRUE(tactile.hasContact());

  tactile.contact_state = mppi_core::NariTouchContactState::kEnoughContacts;
  EXPECT_TRUE(tactile.hasContact());

  tactile.contact_state = mppi_core::NariTouchContactState::kNoContact;
  tactile.units[2].contact = true;
  tactile.units[4].contact = true;
  EXPECT_TRUE(tactile.hasContact());
  EXPECT_EQ(tactile.contactUnitCount(), 2U);
}

TEST(NariTouchStateTest, ContactCentroidIsForceWeightedAndUsesFiniteCop) {
  mppi_core::NariTouchState tactile;
  tactile.units[0].contact = true;
  tactile.units[0].position_m = Eigen::Vector2d{0.0, 0.0};
  tactile.units[0].cop = Eigen::Vector2d{0.001, 0.0};
  tactile.units[0].normal_force_n = 1.0;

  tactile.units[1].contact = true;
  tactile.units[1].position_m = Eigen::Vector2d{0.004, 0.0};
  tactile.units[1].cop =
      Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), 0.002};
  tactile.units[1].normal_force_n = 3.0;

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  ASSERT_TRUE(mppi_core::ComputeNariTouchContactCentroidM(tactile, &centroid));
  EXPECT_NEAR(centroid.x(), 0.00325, kTolerance);
  EXPECT_NEAR(centroid.y(), 0.0, kTolerance);
}

TEST(NariTouchAdapterTest, ContactStatesMapToTactileContactState) {
  EXPECT_EQ(mppi_core::ToTactileContactState(
                mppi_core::NariTouchContactState::kNoContact),
            mppi_core::TactileState::kNoContact);
  EXPECT_EQ(mppi_core::ToTactileContactState(
                mppi_core::NariTouchContactState::kFewContacts),
            mppi_core::TactileState::kFewContacts);
  EXPECT_EQ(mppi_core::ToTactileContactState(
                mppi_core::NariTouchContactState::kEnoughContacts),
            mppi_core::TactileState::kEnoughContacts);
}

TEST(NariTouchAdapterTest, ConvertsForcesCentroidShearAndHemisphereState) {
  mppi_core::NariTouchState nari;
  nari.contact_state = mppi_core::NariTouchContactState::kEnoughContacts;
  nari.force_n = Eigen::Vector3d{0.1, -0.2, 1.0};
  nari.shear_displacement_m = Eigen::Vector2d{0.01, -0.02};
  nari.rotational_shear_rad = 0.3;
  nari.shear_velocity_mps = Eigen::Vector2d{0.4, -0.5};
  nari.rotational_shear_velocity_radps = 0.6;
  nari.slip_score = 0.7;
  nari.slip_velocity_score = 0.8;
  nari.incipient_slip_score = 0.9;
  nari.units[0].contact = true;
  nari.units[0].position_m = Eigen::Vector2d{0.0, 0.0};
  nari.units[0].normal_force_n = 0.25;
  nari.units[1].contact = true;
  nari.units[1].position_m = Eigen::Vector2d{0.002, 0.0};
  nari.units[1].normal_force_n = 0.75;

  const auto tactile = ToTestTactileState(nari);

  EXPECT_TRUE(tactile.valid);
  EXPECT_EQ(tactile.sensor_index, 2);
  EXPECT_EQ(tactile.frame_name, "test_tactile");
  EXPECT_NEAR(tactile.stamp_sec, 1.0, kTolerance);
  EXPECT_EQ(tactile.contact_state, mppi_core::TactileState::kEnoughContacts);
  EXPECT_NEAR(tactile.total_force_n.x(), 0.1, kTolerance);
  EXPECT_NEAR(tactile.total_force_n.y(), -0.2, kTolerance);
  EXPECT_NEAR(tactile.total_force_n.z(), 1.0, kTolerance);
  EXPECT_NEAR(TestTactileCentroidM(tactile).x(), 0.0015, kTolerance);
  EXPECT_NEAR(tactile.shear_displacement_m.x(), 0.01, kTolerance);
  EXPECT_NEAR(tactile.shear_displacement_m.y(), -0.02, kTolerance);
  EXPECT_NEAR(tactile.rotational_shear_rad, 0.3, kTolerance);
  EXPECT_NEAR(tactile.shear_velocity_mps.x(), 0.4, kTolerance);
  EXPECT_NEAR(tactile.shear_velocity_mps.y(), -0.5, kTolerance);
  EXPECT_NEAR(tactile.rotational_shear_velocity_radps, 0.6, kTolerance);
  EXPECT_NEAR(tactile.slip_score, 0.7, kTolerance);
  EXPECT_NEAR(tactile.slip_velocity_score, 0.8, kTolerance);
  EXPECT_NEAR(tactile.incipient_slip_score, 0.9, kTolerance);
  EXPECT_EQ(tactile.hemispheres.size(), mppi_core::kNariTouchUnitCount);
  EXPECT_EQ(tactile.activeHemisphereCount(), 2U);
}

TEST(NariTouchAdapterTest, AllUnitsProduceDenseHemisphereState) {
  mppi_core::NariTouchState nari;
  nari.units[0].contact = true;
  nari.units[0].position_m = Eigen::Vector2d{0.001, 0.002};
  nari.units[0].cop = Eigen::Vector2d{0.0005, -0.00025};
  nari.units[0].normal_force_n = 0.4;

  nari.units[1].contact = false;
  nari.units[1].normal_force_n = 9.0;

  nari.units[3].contact = true;
  nari.units[3].position_m = Eigen::Vector2d{0.003, -0.004};
  nari.units[3].cop =
      Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), 0.001};
  nari.units[3].normal_force_n = -0.1;

  nari.units[7].contact = true;
  nari.units[7].position_m = Eigen::Vector2d{-0.002, 0.005};
  nari.units[7].cop = Eigen::Vector2d{0.0, 0.001};
  nari.units[7].normal_force_n = std::numeric_limits<double>::infinity();

  const auto tactile = ToTestTactileState(nari);

  ASSERT_EQ(tactile.hemispheres.size(), mppi_core::kNariTouchUnitCount);
  EXPECT_EQ(tactile.activeHemisphereCount(), 3U);

  const auto& first = tactile.hemispheres[0];
  EXPECT_TRUE(first.contact);
  EXPECT_EQ(first.hemisphere_index, 0U);
  EXPECT_NEAR(first.cop_sensor_m.x(), 0.0015, kTolerance);
  EXPECT_NEAR(first.cop_sensor_m.y(), 0.00175, kTolerance);
  EXPECT_NEAR(first.normal_force_n, 0.4, kTolerance);
  EXPECT_NEAR(first.confidence, 1.0, kTolerance);

  const auto& inactive = tactile.hemispheres[1];
  EXPECT_FALSE(inactive.contact);
  EXPECT_EQ(inactive.hemisphere_index, 1U);
  EXPECT_NEAR(inactive.normal_force_n, 0.0, kTolerance);
  EXPECT_NEAR(inactive.confidence, 0.0, kTolerance);

  const auto& second = tactile.hemispheres[3];
  EXPECT_TRUE(second.contact);
  EXPECT_EQ(second.hemisphere_index, 3U);
  EXPECT_NEAR(second.cop_sensor_m.x(), 0.003, kTolerance);
  EXPECT_NEAR(second.cop_sensor_m.y(), -0.004, kTolerance);
  EXPECT_NEAR(second.normal_force_n, 0.0, kTolerance);
  EXPECT_NEAR(second.confidence, 1.0, kTolerance);

  const auto& third = tactile.hemispheres[7];
  EXPECT_TRUE(third.contact);
  EXPECT_EQ(third.hemisphere_index, 7U);
  EXPECT_NEAR(third.cop_sensor_m.x(), -0.002, kTolerance);
  EXPECT_NEAR(third.cop_sensor_m.y(), 0.006, kTolerance);
  EXPECT_NEAR(third.normal_force_n, 0.0, kTolerance);
  EXPECT_NEAR(third.confidence, 1.0, kTolerance);
}

TEST(NariTouchAdapterTest, CopiesAggregateShearAndScoresDirectly) {
  mppi_core::NariTouchState nari;
  nari.shear_displacement_m = Eigen::Vector2d{0.01, -0.02};
  nari.rotational_shear_rad = 0.03;
  nari.shear_velocity_mps = Eigen::Vector2d{0.3, -0.4};
  nari.rotational_shear_velocity_radps = 0.5;
  nari.slip_score = 10.0;
  nari.slip_velocity_score = 11.0;
  nari.incipient_slip_score = 12.0;

  const auto tactile = ToTestTactileState(nari);
  EXPECT_NEAR(tactile.shear_displacement_m.x(), 0.01, kTolerance);
  EXPECT_NEAR(tactile.shear_displacement_m.y(), -0.02, kTolerance);
  EXPECT_NEAR(tactile.rotational_shear_rad, 0.03, kTolerance);
  EXPECT_NEAR(tactile.shear_velocity_mps.x(), 0.3, kTolerance);
  EXPECT_NEAR(tactile.shear_velocity_mps.y(), -0.4, kTolerance);
  EXPECT_NEAR(tactile.rotational_shear_velocity_radps, 0.5, kTolerance);
  EXPECT_NEAR(tactile.slip_score, 10.0, kTolerance);
  EXPECT_NEAR(tactile.slip_velocity_score, 11.0, kTolerance);
  EXPECT_NEAR(tactile.incipient_slip_score, 12.0, kTolerance);
}

TEST(NariTouchAdapterTest, ConfidenceReflectsContactQuality) {
  mppi_core::NariTouchState no_contact;
  const auto no_contact_tactile = ToTestTactileState(no_contact);

  mppi_core::NariTouchState stable;
  stable.contact_state = mppi_core::NariTouchContactState::kEnoughContacts;
  stable.units[3].contact = true;
  stable.units[3].position_m = Eigen::Vector2d{0.0, 0.0};
  stable.units[3].normal_force_n = 1.0;
  const auto stable_tactile = ToTestTactileState(stable);

  EXPECT_GT(stable_tactile.confidence, no_contact_tactile.confidence);
}

TEST(TactileTransitionTest,
     PositiveNormalVelocityCanBirthInactiveHemisphere) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.confidence = 1.0;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.hemispheres.push_back(
      MakeHemisphere(1, Eigen::Vector2d{1.0e-4, 0.0}, 0.0, false));
  tactile.hemispheres[1].confidence = 0.0;
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  const auto robot = MakeTestRobotState();
  const auto sensor_context = MakeTactileContextForState(tactile);
  auto motions = MakeMotionsForTactileState(tactile);
  motions[1].normal_velocity_mps = 0.2;

  mppi_core::TactileTransitionConfig config;
  config.birth_approach_velocity_mps = 0.1;
  config.born_normal_force_n = 0.2;

  const auto next = mppi_core::StepTactileState(
      tactile, robot, robot, motions, sensor_context, config, 0.1);

  ASSERT_TRUE(next.valid);
  ASSERT_EQ(next.hemispheres.size(), 2U);
  EXPECT_TRUE(next.hemispheres[1].contact);
  EXPECT_NEAR(next.hemispheres[1].normal_force_n, 0.2, kTolerance);
  EXPECT_NEAR(next.hemispheres[1].confidence, 0.5, kTolerance);
  EXPECT_EQ(next.activeHemisphereCount(), 2U);
  EXPECT_EQ(next.contact_state, mppi_core::TactileState::kEnoughContacts);
  EXPECT_NEAR(next.total_force_n.z(), 1.2, kTolerance);
}

TEST(TactileTransitionTest,
     NegativeNormalVelocityCanLoseActiveHemisphere) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.confidence = 1.0;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.hemispheres.push_back(
      MakeHemisphere(1, Eigen::Vector2d{1.0e-4, 0.0}, 0.0, false));
  tactile.hemispheres[1].confidence = 0.0;
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  const auto robot = MakeTestRobotState();
  const auto sensor_context = MakeTactileContextForState(tactile);
  auto motions = MakeMotionsForTactileState(tactile);
  motions[0].normal_velocity_mps = -0.2;

  mppi_core::TactileTransitionConfig config;
  config.loss_unloading_velocity_mps = 0.1;

  const auto next = mppi_core::StepTactileState(
      tactile, robot, robot, motions, sensor_context, config, 0.1);

  ASSERT_TRUE(next.valid);
  ASSERT_EQ(next.hemispheres.size(), 2U);
  EXPECT_FALSE(next.hemispheres[0].contact);
  EXPECT_NEAR(next.hemispheres[0].normal_force_n, 0.0, kTolerance);
  EXPECT_EQ(next.activeHemisphereCount(), 0U);
  EXPECT_EQ(next.contact_state, mppi_core::TactileState::kNoContact);
  EXPECT_NEAR(next.total_force_n.z(), 0.0, kTolerance);
}

TEST(TactileTransitionTest,
     InactiveNeighborBelowApproachThresholdRemainsInactive) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.hemispheres.push_back(
      MakeHemisphere(1, Eigen::Vector2d{1.0e-4, 0.0}, 0.0, false));
  tactile.hemispheres[1].confidence = 0.0;
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  const auto robot = MakeTestRobotState();
  const auto sensor_context = MakeTactileContextForState(tactile);
  auto motions = MakeMotionsForTactileState(tactile);
  motions[1].normal_velocity_mps = 0.001;

  mppi_core::TactileTransitionConfig config;
  config.birth_approach_velocity_mps = 0.002;

  const auto next = mppi_core::StepTactileState(
      tactile, robot, robot, motions, sensor_context, config, 0.1);

  ASSERT_TRUE(next.valid);
  EXPECT_FALSE(next.hemispheres[1].contact);
  EXPECT_EQ(next.activeHemisphereCount(), 1U);
}

TEST(TactileTransitionTest,
     InactiveNonNeighborAboveApproachThresholdRemainsInactive) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.hemispheres.push_back(
      MakeHemisphere(1, Eigen::Vector2d{1.0e-4, 0.0}, 0.0, false));
  tactile.hemispheres.push_back(
      MakeHemisphere(2, Eigen::Vector2d{2.0e-4, 0.0}, 0.0, false));

  const auto robot = MakeTestRobotState();
  const auto sensor_context = MakeTactileContextForState(tactile);
  auto motions = MakeMotionsForTactileState(tactile);
  motions[2].normal_velocity_mps = 0.2;

  mppi_core::TactileTransitionConfig config;
  config.birth_approach_velocity_mps = 0.1;

  const auto next = mppi_core::StepTactileState(
      tactile, robot, robot, motions, sensor_context, config, 0.1);

  ASSERT_TRUE(next.valid);
  EXPECT_FALSE(next.hemispheres[2].contact);
  EXPECT_EQ(next.activeHemisphereCount(), 1U);
}

TEST(TactileTransitionTest,
     LowNormalForceAloneDoesNotLoseActiveHemisphere) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 0.0, true));
  tactile.total_force_n.z() = 0.0;

  const auto robot = MakeTestRobotState();
  const auto sensor_context = MakeTactileContextForState(tactile);
  const auto motions = MakeMotionsForTactileState(tactile);

  mppi_core::TactileTransitionConfig config;
  config.loss_unloading_velocity_mps = 0.1;

  const auto next = mppi_core::StepTactileState(
      tactile, robot, robot, motions, sensor_context, config, 0.1);

  ASSERT_TRUE(next.valid);
  EXPECT_TRUE(next.hemispheres[0].contact);
  EXPECT_NEAR(next.hemispheres[0].normal_force_n, 0.0, kTolerance);
  EXPECT_EQ(next.activeHemisphereCount(), 1U);
}

TEST(TactileTransitionTest, HighShearCanLoseContactAndPreventBirth) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.shear_displacement_m = Eigen::Vector2d{0.01, 0.0};
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.hemispheres.push_back(
      MakeHemisphere(1, Eigen::Vector2d{1.0e-4, 0.0}, 0.0, false));

  const auto robot = MakeTestRobotState();
  const auto sensor_context = MakeTactileContextForState(tactile);
  auto motions = MakeMotionsForTactileState(tactile);
  motions[1].normal_velocity_mps = 0.2;

  mppi_core::TactileTransitionConfig config;
  config.birth_approach_velocity_mps = 0.1;
  config.max_shear_m = 0.003;

  const auto next = mppi_core::StepTactileState(
      tactile, robot, robot, motions, sensor_context, config, 0.1);

  ASSERT_TRUE(next.valid);
  EXPECT_FALSE(next.hemispheres[0].contact);
  EXPECT_FALSE(next.hemispheres[1].contact);
  EXPECT_EQ(next.activeHemisphereCount(), 0U);
}
TEST(GraspStateRolloutModelTest,
     ResidualCorrectionInputsAreIgnoredByGraspStateRollout) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::RobotSystem robot_system(sensor_model.model);

  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.regularization = 1.0e-9;
  projection_config.tactile_prior_weight = 0.0;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.force_lowpass_alpha = 1.0;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::ContactForceCorrectionState correction;
  correction.normal_force_bias_n = 0.5;

  mppi_core::GraspStateRolloutModel model(1);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd dq = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd measured_tau = Eigen::VectorXd::Constant(1, 1.0);
  mppi_core::GraspObservation observation;
  observation.q_meas = q;
  observation.qdot_meas = dq;
  observation.tau_meas = measured_tau;
  observation.tactile_meas = MakeTactileSensors(tactile, inactive_tactile);
  observation.robot_system = &robot_system;
  const auto state = mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(1),
                                               tactile, inactive_tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  mppi_core::GraspState next_state;
  mppi_core::RolloutContext context;
  context.observation = &observation;
  context.robot_system = &robot_system;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, inactive_tactile);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;
  context.contact_force_correction_state = &correction;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(), 0.5,
              1.0e-6);
  EXPECT_EQ(next_state.tactile_sensors[0].contact_state,
            mppi_core::TactileState::kFewContacts);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 1U);
  EXPECT_EQ(next_state.tactile_sensors[1].activeHemisphereCount(), 0U);
  EXPECT_NEAR(next_state.robot.q[0], 0.0, kTolerance);
  EXPECT_NEAR(next_state.robot.tau[0], 0.0, kTolerance);
}

TEST(GraspStateRolloutModelTest,
     TwoActiveTactileSensorsUseGraspStateRolloutWhenResidualRequired) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.regularization = 1.0e-9;
  projection_config.tactile_prior_weight = 0.0;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.force_lowpass_alpha = 1.0;
  force_rollout_config.min_active_hemisphere_count = 1;

  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd dq = Eigen::VectorXd::Zero(1);
  mppi_core::GraspObservation observation;
  observation.q_meas = q;
  observation.qdot_meas = dq;
  observation.tau_meas = Eigen::VectorXd::Constant(1, 1.0);
  observation.tactile_meas = MakeTactileSensors(tactile, tactile);

  const auto state = mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(1),
                                               tactile, tactile);

  mppi_core::RolloutContext context;
  context.observation = &observation;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;

  mppi_core::GraspStateRolloutModel model(1);
  mppi_core::GraspState next_state;
  model.Step(state, Eigen::VectorXd::Zero(1), context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 1U);
  EXPECT_EQ(next_state.tactile_sensors[1].activeHemisphereCount(), 1U);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(),
              state.tactile_sensors[0].total_force_n.z(), kTolerance);
  EXPECT_NEAR(next_state.tactile_sensors[1].total_force_n.z(),
              state.tactile_sensors[1].total_force_n.z(), kTolerance);
}

TEST(GraspStateRolloutModelTest,
     PinocchioRolloutCanBirthInactiveHemisphere) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.confidence = 1.0;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  tactile.hemispheres.push_back(
      MakeHemisphere(1, Eigen::Vector2d{1.0e-4, 0.0}, 0.0, false));
  tactile.hemispheres[1].confidence = 0.0;
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::RobotSystem robot_system(sensor_model.model);

  mppi_core::TactileTransitionConfig transition_config;
  transition_config.birth_approach_velocity_mps = 0.001;
  transition_config.born_normal_force_n = 0.2;

  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, inactive_tactile);
  context.tactile_transition_config = &transition_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = mppi_core::MakeGraspState(
      pinocchio::neutral(sensor_model.model),
      Eigen::VectorXd::Zero(sensor_model.model.nv),
      Eigen::VectorXd::Zero(sensor_model.model.nv), tactile, inactive_tactile);
  mppi_core::GraspState next_state;

  model.Step(state, Eigen::VectorXd::Constant(1, 0.2), context, 0.1,
             &next_state);

  ASSERT_TRUE(next_state.valid);
  ASSERT_EQ(next_state.tactile_sensors[0].hemispheres.size(), 2U);
  EXPECT_TRUE(next_state.tactile_sensors[0].hemispheres[1].contact);
  EXPECT_NEAR(next_state.tactile_sensors[0].hemispheres[1].normal_force_n,
              0.2, kTolerance);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 2U);
  EXPECT_EQ(next_state.tactile_sensors[0].contact_state,
            mppi_core::TactileState::kEnoughContacts);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(), 1.2,
              kTolerance);
  EXPECT_EQ(next_state.tactile_sensors[1].activeHemisphereCount(), 0U);
}

TEST(GraspStateRolloutModelTest,
     TwoActiveTactileSensorsRemainValidWhenOneSideLosesContact) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data first_data(sensor_model.model);
  pinocchio::Data second_data(sensor_model.model);

  mppi_core::TactileState first_tactile;
  first_tactile.valid = true;
  first_tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  first_tactile.confidence = 1.0;
  first_tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d::Zero(), 1.0, true));
  first_tactile.total_force_n.z() =
      first_tactile.activeHemisphereNormalForceN();
  mppi_core::TactileState second_tactile = first_tactile;

  mppi_core::PinocchioContactKinematicsContext first_kinematics;
  first_kinematics.model = &sensor_model.model;
  first_kinematics.data = &first_data;
  first_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::PinocchioContactKinematicsContext second_kinematics;
  second_kinematics.model = &sensor_model.model;
  second_kinematics.data = &second_data;
  second_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  second_kinematics.normal_axis_sign = -1.0;

  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::TactileTransitionConfig transition_config;
  transition_config.loss_unloading_velocity_mps = 0.001;

  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  context.tactile_contexts = MakeTactileContextsForStates(
      &first_kinematics, first_tactile, &second_kinematics, second_tactile);
  context.tactile_transition_config = &transition_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = mppi_core::MakeGraspState(
      pinocchio::neutral(sensor_model.model),
      Eigen::VectorXd::Zero(sensor_model.model.nv),
      Eigen::VectorXd::Zero(sensor_model.model.nv), first_tactile,
      second_tactile);
  mppi_core::GraspState next_state;

  model.Step(state, Eigen::VectorXd::Constant(1, 0.2), context, 0.1,
             &next_state);

  ASSERT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 1U);
  EXPECT_EQ(next_state.tactile_sensors[0].contact_state,
            mppi_core::TactileState::kFewContacts);
  EXPECT_EQ(next_state.tactile_sensors[1].activeHemisphereCount(), 0U);
  EXPECT_EQ(next_state.tactile_sensors[1].contact_state,
            mppi_core::TactileState::kNoContact);
  EXPECT_TRUE(next_state.robot.q.allFinite());
  EXPECT_TRUE(next_state.robot.qdot.allFinite());
  EXPECT_TRUE(next_state.robot.tau.allFinite());
}

TEST(GraspStateRolloutModelTest, RejectsNonFiniteAction) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;

  const auto state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), tactile, tactile);
  Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  action[0] = std::numeric_limits<double>::quiet_NaN();

  mppi_core::GraspStateRolloutModel model(1);
  mppi_core::RolloutContext context;
  mppi_core::GraspState next_state;

  EXPECT_THROW(model.Step(state, action, context, 0.01, &next_state),
               std::invalid_argument);
}

TEST(GraspStateRolloutModelTest,
     IntegratesAccelerationAndUsesZeroFeedForwardWithoutDynamics) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, 0.4),
      Eigen::VectorXd::Zero(1), tactile, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.02);
  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(nullptr, nullptr);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.robot.q[0], 0.0402, kTolerance);
  EXPECT_NEAR(next_state.robot.qdot[0], 0.402, kTolerance);
  EXPECT_NEAR(next_state.robot.tau[0], 0.0, kTolerance);
}

TEST(GraspStateRolloutModelTest, RejectsTactileContextCountMismatch) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;

  const auto state = MakeState(1, tactile);
  mppi_core::RolloutContext context;
  context.tactile_contexts.resize(1);

  mppi_core::GraspStateRolloutModel model(1);
  mppi_core::GraspState next_state;
  model.Step(state, Eigen::VectorXd::Zero(1), context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors.size(), 2U);
}

TEST(GraspStateRolloutModelTest, RejectsTactileSensorIndexMismatch) {
  mppi_core::TactileState first_tactile;
  first_tactile.valid = true;
  first_tactile.sensor_index = 0;

  mppi_core::TactileState second_tactile;
  second_tactile.valid = true;
  second_tactile.sensor_index = 1;

  const auto state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), first_tactile, second_tactile);

  mppi_core::RolloutContext context;
  context.tactile_contexts.resize(2);
  context.tactile_contexts[0].sensor_index = 0;
  context.tactile_contexts[1].sensor_index = 2;

  mppi_core::GraspStateRolloutModel model(1);
  mppi_core::GraspState next_state;
  model.Step(state, Eigen::VectorXd::Zero(1), context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
}

TEST(GraspStateRolloutModelTest,
     TactileOneUsesItsOwnKinematicsForFallbackRollout) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile1;
  tactile1.valid = true;
  tactile1.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile1.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(&tactile1, 1, TestTactileCentroidM(tactile1),
                     tactile1.total_force_n.z());
  tactile1.confidence = 1.0;
  mppi_core::TactileState tactile0 = MakeInactiveTactileState(tactile1);

  mppi_core::PinocchioContactKinematicsContext opening_side_kinematics;
  opening_side_kinematics.model = &sensor_model.model;
  opening_side_kinematics.data = &data;
  opening_side_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::PinocchioContactKinematicsContext closing_side_kinematics;
  closing_side_kinematics.model = &sensor_model.model;
  closing_side_kinematics.data = &data;
  closing_side_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  closing_side_kinematics.normal_axis_sign = -1.0;

  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;
  mppi_core::TactileTransitionConfig transition_config;
  transition_config.loss_unloading_velocity_mps = 0.001;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &opening_side_kinematics, tactile0, &closing_side_kinematics, tactile1);
  context.tactile_transition_config = &transition_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutModel model(1);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd dq = Eigen::VectorXd::Zero(1);
  const auto state = mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(1),
                                               tactile0, tactile1);
  mppi_core::GraspState next_state;

  model.Step(state, Eigen::VectorXd::Constant(1, 0.1), context, 0.1,
             &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 0U);
  EXPECT_EQ(next_state.tactile_sensors[1].activeHemisphereCount(), 0U);
  EXPECT_NEAR(next_state.tactile_sensors[1].total_force_n.z(), 0.0,
              kTolerance);
}

TEST(GraspStateRolloutModelTest, ComputesFeedForwardTorqueWithRnea) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);
  pinocchio::Data expected_data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.0;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = 0;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;

  mppi_core::GraspStateRolloutModel model(1);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd qdot = Eigen::VectorXd::Constant(1, 0.4);
  const auto state = mppi_core::MakeGraspState(
      q, qdot, Eigen::VectorXd::Zero(1), tactile, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.2);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.robot.q[0], 0.042, kTolerance);
  EXPECT_NEAR(next_state.robot.qdot[0], 0.42, kTolerance);
  const Eigen::VectorXd expected_tau =
      pinocchio::rnea(sensor_model.model, expected_data, q, qdot, action);
  ASSERT_EQ(expected_tau.size(), next_state.robot.tau.size());
  EXPECT_NEAR(next_state.robot.tau[0], expected_tau[0], kTolerance);
}

TEST(GraspStateRolloutModelTest, PinocchioRolloutAllowsNqDifferentFromNv) {
  const auto sensor_model = MakeSingleUnboundedRevoluteZSensorModel();
  ASSERT_EQ(sensor_model.model.nq, 2);
  ASSERT_EQ(sensor_model.model.nv, 1);
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = tactile.hemispheres.size();
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  point.normal_force_n = 1.0;
  tactile.hemispheres.push_back(point);
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::ContactForceProjectionConfig projection_config;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::RolloutContext context;
  context.robot_system = &robot_system;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.2);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.robot.q.size(), 2);
  EXPECT_EQ(next_state.robot.qdot.size(), 1);
  EXPECT_EQ(next_state.robot.tau.size(), 1);
  EXPECT_TRUE(next_state.robot.q.allFinite());
  EXPECT_NEAR(next_state.robot.qdot[0], 0.02, kTolerance);
  const Eigen::VectorXd expected_tau =
      pinocchio::rnea(sensor_model.model, data, state.robot.q,
                      state.robot.qdot, action);
  ASSERT_EQ(expected_tau.size(), next_state.robot.tau.size());
  EXPECT_NEAR(next_state.robot.tau[0], expected_tau[0], kTolerance);
}

TEST(GraspStateRolloutModelTest,
     ProjectionDisabledKeepsAggregateForceInGraspStateRollout) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetActiveHemispheresAroundCentroid(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;
  mppi_core::TactileTransitionConfig transition_config;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.tactile_transition_config = &transition_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(), 0.5, 1.0e-6);
  EXPECT_EQ(next_state.tactile_sensors[0].contact_state,
            mppi_core::TactileState::kEnoughContacts);
}

TEST(GraspStateRolloutModelTest,
     ProjectionDisabledDoesNotMakeGraspStateRolloutInvalid) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetActiveHemispheresAroundCentroid(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;
  mppi_core::TactileTransitionConfig transition_config;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, inactive_tactile);
  context.tactile_transition_config = &transition_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = mppi_core::MakeGraspState(
      pinocchio::neutral(sensor_model.model),
      Eigen::VectorXd::Zero(sensor_model.model.nv),
      Eigen::VectorXd::Zero(sensor_model.model.nv), tactile, inactive_tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_TRUE(next_state.robot.q.allFinite());
  EXPECT_TRUE(next_state.robot.qdot.allFinite());
  EXPECT_TRUE(next_state.robot.tau.allFinite());
}

TEST(GraspStateRolloutModelTest,
     GraspStateRolloutDoesNotRequireActiveTactileContact) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.total_force_n.z() = 0.0;
  SetActiveHemispheresAroundCentroid(&tactile, 0, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(nullptr, nullptr);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
}

TEST(GraspStateRolloutModelTest,
     MissingKinematicsRejectsTactileTransition) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(nullptr, nullptr);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
}

TEST(GraspStateRolloutModelTest,
     PinocchioContactMotionsDriveTactileTransition) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = tactile.hemispheres.size();
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  point.normal_force_n = 1.0;
  tactile.hemispheres.push_back(point);
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::TactileTransitionConfig transition_config;
  transition_config.max_shear_m = 0.1;
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.tactile_transition_config = &transition_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.005);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_GT(next_state.tactile_sensors[0].shear_displacement_m.y(),
            tactile.shear_displacement_m.y());
  EXPECT_GT(next_state.tactile_sensors[0].slip_score,
            tactile.shear_displacement_m.norm());
  EXPECT_GT(TestTactileCentroidM(next_state.tactile_sensors[0]).y(),
            TestTactileCentroidM(tactile).y());
}

TEST(GraspStateRolloutModelTest,
     OpposingPinocchioTangentialMotionReducesPredictedShear) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::TactileTransitionConfig transition_config;
  transition_config.max_shear_m = 0.1;
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.tactile_transition_config = &transition_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, -0.005);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);
  EXPECT_LT(next_state.tactile_sensors[0].shear_displacement_m.norm(),
            tactile.shear_displacement_m.norm());
}

TEST(GraspStateRolloutModelTest,
     ZeroPinocchioActionLeavesHemisphereGeometryUnchanged) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d{0.001, 0.0}, tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 0.8;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = tactile.hemispheres.size();
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  point.normal_force_n = 1.0;
  tactile.hemispheres.push_back(point);
  tactile.total_force_n.z() = tactile.activeHemisphereNormalForceN();

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::TactileTransitionConfig transition_config;
  transition_config.max_shear_m = 0.1;
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);
  context.tactile_transition_config = &transition_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);
  EXPECT_NEAR(TestTactileCentroidM(next_state.tactile_sensors[0]).x(),
              TestTactileCentroidM(tactile).x(), kTolerance);
  EXPECT_NEAR(TestTactileCentroidM(next_state.tactile_sensors[0]).y(),
              TestTactileCentroidM(tactile).y(), kTolerance);
  EXPECT_NEAR(next_state.tactile_sensors[0].shear_displacement_m.x(),
              tactile.shear_displacement_m.x(), kTolerance);
  EXPECT_NEAR(next_state.tactile_sensors[0].shear_displacement_m.y(),
              tactile.shear_displacement_m.y(), kTolerance);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(),
              tactile.total_force_n.z(), kTolerance);
  EXPECT_NEAR(next_state.tactile_sensors[0].confidence, tactile.confidence,
              kTolerance);
}

TEST(GraspStateRolloutModelTest,
     NoActiveHemispheresRemainValidForCostHandling) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  tactile.confidence = 1.0;
  tactile.hemispheres.push_back(
      MakeHemisphere(0, Eigen::Vector2d{1.0, 0.0}, 0.0, false));

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, tactile);

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.005);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 0U);
}

TEST(GraspStabilityCostTest, TooFewActiveTactileSensorsIncreaseCost) {
  mppi_core::GraspStabilityCostConfig config;
  config.min_active_tactile_sensors = 2;
  config.target_active_hemisphere_total = 0;
  config.contact_loss_weight = 10.0;
  config.support_weight = 0.0;
  config.shear_weight = 0.0;
  config.rotation_weight = 0.0;
  config.qddot_weight = 0.0;
  config.tau_weight = 0.0;

  mppi_core::GraspStabilityCost cost(config);

  mppi_core::CostContext context;
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);

  mppi_core::TactileState active_tactile;
  active_tactile.valid = true;
  active_tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  SetActiveHemispheresAroundCentroid(&active_tactile, 1, Eigen::Vector2d::Zero(), 1.0);
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(active_tactile);

  const auto one_sensor_state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), active_tactile, inactive_tactile);
  const auto two_sensor_state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), active_tactile, active_tactile);

  EXPECT_GT(cost.Evaluate(one_sensor_state, action, context),
            cost.Evaluate(two_sensor_state, action, context));
}

TEST(GraspStabilityCostTest,
     TooFewActiveHemispheresIncreaseCostButExtraContactsDoNotReward) {
  mppi_core::GraspStabilityCostConfig config;
  config.min_active_tactile_sensors = 0;
  config.target_active_hemisphere_total = 4;
  config.contact_loss_weight = 0.0;
  config.support_weight = 1.0;
  config.shear_weight = 0.0;
  config.rotation_weight = 0.0;
  config.qddot_weight = 0.0;
  config.tau_weight = 0.0;

  mppi_core::GraspStabilityCost cost(config);
  mppi_core::CostContext context;
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;

  auto low_support_state = MakeState(1, tactile);
  SetActiveHemispheresAroundCentroid(&low_support_state.tactile_sensors[0], 1,
                     Eigen::Vector2d::Zero(), 1.0);
  SetActiveHemispheresAroundCentroid(&low_support_state.tactile_sensors[1], 1,
                     Eigen::Vector2d::Zero(), 1.0);

  auto target_support_state = MakeState(1, tactile);
  SetActiveHemispheresAroundCentroid(&target_support_state.tactile_sensors[0], 2,
                     Eigen::Vector2d::Zero(), 1.0);
  SetActiveHemispheresAroundCentroid(&target_support_state.tactile_sensors[1], 2,
                     Eigen::Vector2d::Zero(), 1.0);

  auto extra_support_state = MakeState(1, tactile);
  SetActiveHemispheresAroundCentroid(&extra_support_state.tactile_sensors[0], 3,
                     Eigen::Vector2d::Zero(), 1.0);
  SetActiveHemispheresAroundCentroid(&extra_support_state.tactile_sensors[1], 3,
                     Eigen::Vector2d::Zero(), 1.0);

  const double low_support_cost =
      cost.Evaluate(low_support_state, action, context);
  const double target_support_cost =
      cost.Evaluate(target_support_state, action, context);
  const double extra_support_cost =
      cost.Evaluate(extra_support_state, action, context);

  EXPECT_GT(low_support_cost, target_support_cost);
  EXPECT_NEAR(target_support_cost, 0.0, kTolerance);
  EXPECT_NEAR(extra_support_cost, target_support_cost, kTolerance);
}

TEST(GraspStabilityCostTest, ShearRotationActionAndTauIncreaseCost) {
  mppi_core::GraspStabilityCostConfig config;
  config.min_active_tactile_sensors = 0;
  config.target_active_hemisphere_total = 0;
  config.contact_loss_weight = 0.0;
  config.support_weight = 0.0;
  config.shear_weight = 2.0;
  config.rotation_weight = 3.0;
  config.qddot_weight = 4.0;
  config.tau_weight = 5.0;

  mppi_core::GraspStabilityCost cost(config);
  mppi_core::CostContext context;

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.shear_displacement_m = Eigen::Vector2d{0.1, 0.2};
  tactile.rotational_shear_rad = 0.3;

  auto state = MakeState(1, tactile);
  state.robot.tau = Eigen::VectorXd::Constant(1, 0.4);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.5);

  const double expected = 2.0 * (0.1 * 0.1 + 0.2 * 0.2) * 2.0 +
                          3.0 * 0.3 * 0.3 * 2.0 + 4.0 * 0.5 * 0.5 +
                          5.0 * 0.4 * 0.4;
  EXPECT_NEAR(cost.Evaluate(state, action, context), expected, kTolerance);
}

TEST(TaskConfigTest, ParsesTaskObjectiveToleranceStartAndCost) {
  const YAML::Node root = YAML::Load(R"(
task:
  name: jenga
  start:
    min_enough_contact_sensors: 2
    min_active_hemisphere_total: 3
  objective:
    min_active_tactile_sensors: 2
    target_active_hemisphere_total: 5
  tolerance:
    max_shear_m: 0.005
    max_rotation_rad: 0.06
  object:
    name: block
    geometry_type: box
    uri: package://mppi_core/object/jenga_block.urdf
    primitive_size_m: [0.15, 0.05, 0.03]
    initial_pose_world:
      xyz: [0.1, -0.2, 0.3]
      rpy: [0.0, 0.0, 1.57]
    initial_pose_uncertainty:
      xyz_std_m: [0.01, 0.02, 0.03]
      rpy_std_rad: [0.1, 0.2, 0.3]
  cost:
    contact_loss_weight: 51.0
    support_weight: 11.0
    shear_weight: 6.0
    rotation_weight: 7.0
    qddot_weight: 8.0
    tau_weight: 0.09
)");

  const auto task_config =
      mppi_core::ParseTaskConfig(root["task"], mppi_core::TaskConfig{});

  EXPECT_EQ(task_config.name, "jenga");
  EXPECT_EQ(task_config.start.min_enough_contact_sensors, 2U);
  EXPECT_EQ(task_config.start.min_active_hemispheres_total, 3U);
  EXPECT_EQ(task_config.cost.min_active_tactile_sensors, 2U);
  EXPECT_EQ(task_config.cost.target_active_hemisphere_total, 5U);
  EXPECT_NEAR(task_config.tolerance.max_shear_m, 0.005, kTolerance);
  EXPECT_NEAR(task_config.tolerance.max_rotation_rad, 0.06, kTolerance);
  EXPECT_NEAR(task_config.cost.contact_loss_weight, 51.0, kTolerance);
  EXPECT_NEAR(task_config.cost.support_weight, 11.0, kTolerance);
  EXPECT_NEAR(task_config.cost.shear_weight, 6.0, kTolerance);
  EXPECT_NEAR(task_config.cost.rotation_weight, 7.0, kTolerance);
  EXPECT_NEAR(task_config.cost.qddot_weight, 8.0, kTolerance);
  EXPECT_NEAR(task_config.cost.tau_weight, 0.09, kTolerance);
  EXPECT_TRUE(mppi_core::IsValidObjectPrior(task_config.object_prior));
  EXPECT_EQ(task_config.object_prior.name, "block");
  EXPECT_EQ(task_config.object_prior.geometry.type,
            mppi_core::ObjectGeometryType::kBox);
  EXPECT_EQ(task_config.object_prior.geometry.uri,
            "package://mppi_core/object/jenga_block.urdf");
  EXPECT_NEAR(task_config.object_prior.geometry.primitive_size_m.x(), 0.15,
              kTolerance);
  EXPECT_NEAR(task_config.object_prior.initial_pose_world.translation().y(),
              -0.2, kTolerance);
  EXPECT_NEAR(task_config.object_prior.position_std_m.z(), 0.03,
              kTolerance);
  EXPECT_NEAR(task_config.object_prior.rpy_std_rad.y(), 0.2,
              kTolerance);
}

TEST(RobustGraspPolicyConfigTest, ParsesObjectSupportCostConfig) {
  const YAML::Node root = YAML::Load(R"(
robust_grasp:
  control_mode: continuous_qddot_mppi
  continuous_control_rate_cost_weight: 0.123
  continuous_smoothing_alpha: 0.4
  skip_mppi_when_not_enough_contacts: true
  use_rnea_feedforward: true
  rnea:
    use_measured_state: true
    subtract_contact_torque: false
    tau_ff_scale: 0.2
    max_tau_ff_nm: 0.05
    max_tau_ff_rate_nm_s: 1.5
    zero_tau_when_not_ready: true
    zero_tau_on_contact_loss: false
    gravity_only_when_qddot_zero: true
  rollout:
    horizon_steps: 3
    dt: 0.02
    action_dim: 2
    num_rollouts: 17
    temperature: 0.9
    action_noise_clip: 3.0
    qdot_limit: 4.0
  cost:
    min_active_tactile_sensors: 1
    min_active_hemisphere_total: 3
    contact_loss_weight: 70.0
    support_weight: 8.0
    object_support:
      enabled: true
      object_pose_samples: 9
      hemisphere_radius_m: 0.0015
      support_distance_scale_m: 0.0025
      good_contact_gap_min_m: -0.0011
      good_contact_gap_max_m: 0.0012
      deep_contact_scale_m: 0.0023
      deep_contact_weight: 6.0
      edge_weight: 12.0
      target_edge_margin_m: 0.002
      use_particle_weights: false
  disturbance:
    object_motion:
      linear_velocity_std_mps: 0.01
      linear_velocity_max_mps: 0.02
      angular_velocity_std_radps: 0.3
      angular_velocity_max_radps: 0.4
)");

  const auto config =
      mppi_core::ParseRobustGraspPolicyConfig(
          root["robust_grasp"], 2, mppi_core::RobustGraspPolicyConfig{});

  EXPECT_EQ(config.rollout.horizon_steps, 3U);
  EXPECT_EQ(config.rollout.num_rollouts, 17U);
  EXPECT_DOUBLE_EQ(config.rollout.dt, 0.02);
  EXPECT_DOUBLE_EQ(config.rollout.temperature, 0.9);
  ASSERT_EQ(config.rollout.action_noise_clip.size(), 2);
  ASSERT_EQ(config.rollout.qdot_lower_bound.size(), 2);
  EXPECT_NEAR(config.rollout.action_noise_clip[0], 3.0, kTolerance);
  EXPECT_NEAR(config.rollout.qdot_lower_bound[0], -4.0, kTolerance);
  EXPECT_EQ(config.control_mode,
            mppi_core::RobustGraspControlMode::kContinuousQddotMppi);
  EXPECT_NEAR(config.continuous_control_rate_cost_weight, 0.123,
              kTolerance);
  EXPECT_NEAR(config.continuous_smoothing_alpha, 0.4, kTolerance);
  EXPECT_TRUE(config.skip_mppi_when_not_enough_contacts);
  EXPECT_TRUE(config.rnea_feedforward.enabled);
  EXPECT_TRUE(config.rnea_feedforward.use_measured_state);
  EXPECT_FALSE(config.rnea_feedforward.subtract_contact_torque);
  EXPECT_NEAR(config.rnea_feedforward.tau_ff_scale, 0.2, kTolerance);
  EXPECT_NEAR(config.rnea_feedforward.max_tau_ff_nm, 0.05, kTolerance);
  EXPECT_NEAR(config.rnea_feedforward.max_tau_ff_rate_nm_s, 1.5,
              kTolerance);
  EXPECT_TRUE(config.rnea_feedforward.zero_tau_when_not_ready);
  EXPECT_FALSE(config.rnea_feedforward.zero_tau_on_contact_loss);
  EXPECT_TRUE(config.rnea_feedforward.gravity_only_when_qddot_zero);
  EXPECT_EQ(config.cost.object_support.min_active_tactile_sensors, 1U);
  EXPECT_EQ(config.cost.object_support.min_active_hemisphere_total, 3U);
  EXPECT_EQ(config.cost.object_support.max_object_samples, 9U);
  EXPECT_NEAR(config.cost.object_support.contact_loss_weight, 70.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.support_weight, 8.0, kTolerance);
  EXPECT_NEAR(config.cost.object_support.hemisphere_radius_m, 0.0015,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.support_distance_scale_m, 0.0025,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.good_contact_gap_min_m, -0.0011,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.good_contact_gap_max_m, 0.0012,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.deep_contact_scale_m, 0.0023,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.deep_contact_weight, 6.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.contact_stiffness_n_per_m, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.target_predicted_normal_force_n, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.max_predicted_normal_force_n, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.max_predicted_force_per_sensor_n, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.edge_weight, 12.0, kTolerance);
  EXPECT_NEAR(config.cost.object_support.penetration_weight, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.predicted_force_low_weight, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.predicted_force_high_weight, 0.0,
              kTolerance);
  EXPECT_NEAR(config.cost.object_support.target_edge_margin_m, 0.002,
              kTolerance);
  EXPECT_FALSE(config.cost.object_support.use_particle_weights);
  EXPECT_NEAR(config.disturbance_sampler.object.linear_velocity_std_mps, 0.01,
              kTolerance);
  EXPECT_NEAR(config.disturbance_sampler.object.linear_velocity_max_mps, 0.02,
              kTolerance);
  EXPECT_NEAR(config.disturbance_sampler.object.angular_velocity_std_radps, 0.3,
              kTolerance);
  EXPECT_NEAR(config.disturbance_sampler.object.angular_velocity_max_radps, 0.4,
              kTolerance);
}

TEST(RobustGraspPolicyConfigTest,
     DeprecatedObjectPoseDisturbanceFieldsThrow) {
  const YAML::Node root = YAML::Load(R"(
robust_grasp:
  rollout:
    horizon_steps: 3
    action_dim: 2
  disturbance:
    object_disturbance:
      linear_velocity_std_mps: 0.01
      pose_xyz_std_m: 0.001
)");

  EXPECT_THROW(
      (void)mppi_core::ParseRobustGraspPolicyConfig(
          root["robust_grasp"], 2, mppi_core::RobustGraspPolicyConfig{}),
      std::invalid_argument);
}

TEST(RobustGraspPolicyConfigTest, ParsesDiscreteActionSelectorMode) {
  const YAML::Node root = YAML::Load(R"(
robust_grasp:
  control_mode: discrete_action_selector
  rollout:
    horizon_steps: 1
    action_dim: 2
)");

  const auto config =
      mppi_core::ParseRobustGraspPolicyConfig(
          root["robust_grasp"], 2, mppi_core::RobustGraspPolicyConfig{});

  EXPECT_EQ(config.control_mode,
            mppi_core::RobustGraspControlMode::kDiscreteActionSelector);
  EXPECT_EQ(config.rollout.horizon_steps, 1U);
}

TEST(TaskConfigTest, AppliesTaskToleranceToTransitionConfig) {
  const YAML::Node root = YAML::Load(R"(
task:
  tolerance:
    max_shear_m: 0.01
    max_rotation_rad: 0.2
)");

  const auto task_config =
      mppi_core::ParseTaskConfig(root["task"], mppi_core::TaskConfig{});
  mppi_core::TactileTransitionConfig transition;
  transition.birth_approach_velocity_mps = 0.123;
  transition.loss_unloading_velocity_mps = 0.456;
  transition.born_normal_force_n = 0.789;

  const auto merged =
      mppi_core::ApplyTaskToleranceToTransitionConfig(task_config, transition);

  EXPECT_NEAR(merged.birth_approach_velocity_mps, 0.123, kTolerance);
  EXPECT_NEAR(merged.loss_unloading_velocity_mps, 0.456, kTolerance);
  EXPECT_NEAR(merged.born_normal_force_n, 0.789, kTolerance);
  EXPECT_NEAR(merged.max_shear_m, 0.01, kTolerance);
  EXPECT_NEAR(merged.max_rotation_rad, 0.2, kTolerance);
}

TEST(RolloutConfigTest, ParsesTactileModelAndDisturbanceConfig) {
  const YAML::Node root = YAML::Load(R"(
rollout:
  tactile_model:
    birth_approach_velocity_mps: 0.003
    loss_unloading_velocity_mps: 0.004
    born_normal_force_n: 0.07
    born_confidence: 0.6
    contact_confidence_decay: 0.9
  disturbance:
    enabled: true
)");

  const auto rollout_config = mppi_core::ParseGraspStateRolloutConfig(
      root["rollout"], mppi_core::GraspStateRolloutConfig{});
  const auto tactile_transition_config =
      mppi_core::ParseTactileTransitionConfig(
          root["rollout"], mppi_core::TactileTransitionConfig{});

  EXPECT_TRUE(rollout_config.disturbance_enabled);
  EXPECT_NEAR(tactile_transition_config.birth_approach_velocity_mps, 0.003,
              kTolerance);
  EXPECT_NEAR(tactile_transition_config.loss_unloading_velocity_mps, 0.004,
              kTolerance);
  EXPECT_NEAR(tactile_transition_config.born_normal_force_n, 0.07,
              kTolerance);
  EXPECT_NEAR(tactile_transition_config.born_confidence, 0.6, kTolerance);
  EXPECT_NEAR(tactile_transition_config.contact_confidence_decay, 0.9,
              kTolerance);
}

TEST(ConfigFileLayoutTest, DefaultFilesUseSplitTaskAndRolloutSurface) {
  EXPECT_FALSE(std::filesystem::exists(
      MppiCorePackageRoot() / "config" / "grasp.yaml"));

  const std::filesystem::path rollout_path =
      MppiCorePackageRoot() / "config" / "rollout.yaml";
  const std::filesystem::path task_path =
      MppiCorePackageRoot() / "task" / "grasp_hold.yaml";
  const std::string rollout_contents = ReadTextFile(rollout_path);
  const std::string task_contents = ReadTextFile(task_path);

  EXPECT_EQ(rollout_contents.find("contact_loss_weight"),
            std::string::npos);
  EXPECT_EQ(task_contents.find("contact_force_rollout"), std::string::npos);
  EXPECT_EQ(task_contents.find("birth_neighbor_weight"), std::string::npos);

  const YAML::Node rollout_root = YAML::Load(rollout_contents);
  const YAML::Node task_root = YAML::Load(task_contents);
  (void)mppi_core::ParseGraspStateRolloutConfig(
      rollout_root["rollout"], mppi_core::GraspStateRolloutConfig{});
  (void)mppi_core::ParseTactileTransitionConfig(
      rollout_root["rollout"], mppi_core::TactileTransitionConfig{});
  (void)mppi_core::ParseTaskConfig(task_root["task"],
                                   mppi_core::TaskConfig{});
}

TEST(IncludeStructureTest, ProductionCodeDoesNotIncludeDeprecatedPaths) {
  const std::filesystem::path include_root =
      MppiCorePackageRoot() / "include" / "mppi_core";
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(include_root)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".hpp") {
      continue;
    }

    const std::string contents = ReadTextFile(entry.path());
    EXPECT_EQ(contents.find("mppi_core/grasp/"), std::string::npos)
        << entry.path();
    EXPECT_EQ(contents.find("mppi_core/model/"), std::string::npos)
        << entry.path();
    EXPECT_EQ(contents.find("mppi_core/contact/contact_force_rollout.hpp"),
              std::string::npos)
        << entry.path();
  }

  const std::filesystem::path source_root = MppiCorePackageRoot() / "src";
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(source_root)) {
    if (!entry.is_regular_file() || (entry.path().extension() != ".cpp" &&
                                     entry.path().extension() != ".hpp")) {
      continue;
    }

    const std::string contents = ReadTextFile(entry.path());
    EXPECT_EQ(contents.find("mppi_core/grasp/"), std::string::npos)
        << entry.path();
    EXPECT_EQ(contents.find("mppi_core/model/"), std::string::npos)
        << entry.path();
    EXPECT_EQ(contents.find("mppi_core/contact/contact_force_rollout.hpp"),
              std::string::npos)
        << entry.path();
  }
}

TEST(IncludeStructureTest, CompatibilityHeadersAreRemoved) {
  const std::filesystem::path include_root =
      MppiCorePackageRoot() / "include" / "mppi_core";
  const std::vector<std::filesystem::path> compatibility_headers = {
      include_root / "contact" / "contact_force_rollout.hpp",
      include_root / "grasp" / "grasp_state.hpp",
      include_root / "grasp" / "grasp_rollout.hpp",
      include_root / "grasp" / "grasp_contact_kinematics.hpp",
      include_root / "grasp" / "contact_force_projection.hpp",
      include_root / "grasp" / "contact_force_rollout.hpp",
      include_root / "grasp" / "contact_force_correction.hpp",
      include_root / "grasp_types.hpp",
      include_root / "robot_command.hpp",
      include_root / "robot" / "robot_command.hpp",
      include_root / "robot" / "robot_dynamics_context.hpp",
      include_root / "robot" / "robot_state.hpp",
      include_root / "tactile" / "tactile_rollout_policy.hpp",
      include_root / "model" / "rollout.hpp",
      include_root / "model" / "joint_acceleration_rollout_model.hpp",
      include_root / "model" / "delta_q_reference_rollout_model.hpp",
  };

  for (const auto& header : compatibility_headers) {
    EXPECT_FALSE(std::filesystem::exists(header)) << header;
  }
}

TEST(MPPIConfigTest, ParsesSamplingAndExpandsScalarActionParameters) {
  const YAML::Node root = YAML::Load(R"(
mppi:
  horizon_steps: 15
  dt: 0.02
  num_rollouts: 64
  temperature: 0.8
  random_seed: 42
  action:
    lower_bound: -0.003
    upper_bound: 0.004
    noise_std: 0.001
    noise_clip: 0.002
  state:
    qdot_limit: 0.7
)");

  const auto config =
      mppi_core::ParseMPPIConfig(root["mppi"], 3, mppi_core::MPPIConfig{});

  EXPECT_EQ(config.horizon_steps, 15U);
  EXPECT_EQ(config.num_rollouts, 64U);
  EXPECT_EQ(config.action_dim, 3U);
  EXPECT_NEAR(config.dt, 0.02, kTolerance);
  EXPECT_NEAR(config.temperature, 0.8, kTolerance);
  EXPECT_EQ(config.random_seed, 42U);
  ASSERT_EQ(config.action_lower_bound.size(), 3);
  ASSERT_EQ(config.action_upper_bound.size(), 3);
  ASSERT_EQ(config.action_noise_std.size(), 3);
  ASSERT_EQ(config.action_noise_clip.size(), 3);
  ASSERT_EQ(config.qdot_lower_bound.size(), 3);
  ASSERT_EQ(config.qdot_upper_bound.size(), 3);
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(config.action_lower_bound[i], -0.003, kTolerance);
    EXPECT_NEAR(config.action_upper_bound[i], 0.004, kTolerance);
    EXPECT_NEAR(config.action_noise_std[i], 0.001, kTolerance);
    EXPECT_NEAR(config.action_noise_clip[i], 0.002, kTolerance);
    EXPECT_NEAR(config.qdot_lower_bound[i], -0.7, kTolerance);
    EXPECT_NEAR(config.qdot_upper_bound[i], 0.7, kTolerance);
  }
}

TEST(MPPIConfigTest, RejectsDriverLocalCommandGains) {
  const YAML::Node root = YAML::Load(R"(
mppi:
  action:
    lower_bound: -1.0
    upper_bound: 1.0
    noise_std: 0.1
  command:
    kp: 30.0
    kd: 1.5
)");

  EXPECT_THROW(
      (void)mppi_core::ParseMPPIConfig(root["mppi"], 3, mppi_core::MPPIConfig{}),
      std::invalid_argument);
}

TEST(MPPIConfigTest, RejectsWrongSizedActionVectors) {
  const YAML::Node root = YAML::Load(R"(
mppi:
  action:
    lower_bound: [-0.1, -0.2]
)");

  EXPECT_THROW((void)mppi_core::ParseMPPIConfig(root["mppi"], 3,
                                                mppi_core::MPPIConfig{}),
               std::invalid_argument);
}

TEST(MPPIOptimizerTest, PredictRolloutRecordsActionsStatesAndStepCosts) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::MPPIConfig config;
  config.horizon_steps = 2;
  config.num_rollouts = 1;
  config.action_dim = 1;
  config.dt = 0.1;
  config.temperature = 1.0;
  config.action_lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  config.action_upper_bound = Eigen::VectorXd::Constant(1, 1.0);
  config.action_noise_std = Eigen::VectorXd::Zero(1);

  auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(1);
  mppi_core::MPPIOptimizer optimizer;
  optimizer.Initialize(config, model, nullptr);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = 0;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::RobotSystem robot_system(sensor_model.model);
  mppi_core::ContactForceProjectionConfig projection_config;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Constant(1, 0.5);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = Eigen::VectorXd::Constant(1, 0.25);
  observation.qdot_meas = Eigen::VectorXd::Constant(1, 0.02);
  observation.tau_meas = Eigen::VectorXd::Zero(1);
  observation.tactile_meas = MakeTactileSensors(tactile, inactive_tactile);
  observation.object_belief = MakeTestObjectBelief();
  observation.robot_system = &robot_system;
  observation.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, inactive_tactile);
  observation.contact_force_projection_config = &projection_config;
  observation.contact_force_rollout_config = &force_rollout_config;

  mppi_core::ActionSequence actions(1, 2);
  actions.setAction(0, Eigen::VectorXd::Constant(1, 0.1));
  actions.setAction(1, Eigen::VectorXd::Constant(1, -0.05));

  const auto trace = optimizer.PredictRollout(observation, actions);

  ASSERT_EQ(trace.actions.size(), 2U);
  ASSERT_EQ(trace.step_costs.size(), 2U);
  ASSERT_EQ(trace.states.size(), 3U);
  EXPECT_NEAR(trace.total_cost, 0.0, kTolerance);
  EXPECT_NEAR(trace.states[0].robot.q[0], 0.25, kTolerance);
  EXPECT_NEAR(trace.states[0].robot.qdot[0], 0.02, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.q[0], 0.253, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.qdot[0], 0.03, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.q[0], 0.2555, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.qdot[0], 0.025, kTolerance);
  EXPECT_NEAR(trace.actions[0][0], 0.1, kTolerance);
  EXPECT_NEAR(trace.actions[1][0], -0.05, kTolerance);
  EXPECT_TRUE(trace.states[1].valid);
  EXPECT_TRUE(trace.states[1].tactile_sensors[0].valid);
  EXPECT_TRUE(trace.states[1].hasObjectBelief());
}

TEST(MPPIOptimizerTest, InvalidRolloutStepGetsLargeCostAndStopsTrace) {
  mppi_core::MPPIConfig config;
  config.horizon_steps = 2;
  config.num_rollouts = 1;
  config.action_dim = 1;
  config.dt = 0.1;
  config.temperature = 1.0;
  config.action_lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  config.action_upper_bound = Eigen::VectorXd::Constant(1, 1.0);
  config.action_noise_std = Eigen::VectorXd::Zero(1);

  auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(1);
  mppi_core::MPPIOptimizer optimizer;
  optimizer.Initialize(config, model, nullptr);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.total_force_n.z() = 0.0;

  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Zero(1);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(1);
  observation.tactile_meas = MakeTactileSensors(tactile, tactile);

  mppi_core::ActionSequence actions(1, 2);
  actions.setAction(0, Eigen::VectorXd::Constant(1, 0.1));
  actions.setAction(1, Eigen::VectorXd::Constant(1, 0.1));

  const auto trace = optimizer.PredictRollout(observation, actions);

  ASSERT_EQ(trace.states.size(), 2U);
  ASSERT_EQ(trace.actions.size(), 1U);
  ASSERT_EQ(trace.step_costs.size(), 1U);
  EXPECT_FALSE(trace.states[1].valid);
  EXPECT_NEAR(trace.step_costs[0], 1.0e30, 0.0);
  EXPECT_NEAR(trace.total_cost, 1.0e30, 0.0);
}

TEST(MPPIOptimizerTest, PredictRolloutRequiresMeasuredTorqueFeedback) {
  mppi_core::MPPIConfig config;
  config.horizon_steps = 1;
  config.num_rollouts = 1;
  config.action_dim = 1;
  config.dt = 0.1;
  config.temperature = 1.0;
  config.action_lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  config.action_upper_bound = Eigen::VectorXd::Constant(1, 1.0);
  config.action_noise_std = Eigen::VectorXd::Zero(1);

  auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(1);
  mppi_core::MPPIOptimizer optimizer;
  optimizer.Initialize(config, model, nullptr);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;

  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Zero(1);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tactile_meas = MakeTactileSensors(tactile, tactile);

  mppi_core::ActionSequence actions(1, 1);
  actions.setAction(0, Eigen::VectorXd::Zero(1));

  EXPECT_THROW((void)optimizer.PredictRollout(observation, actions),
               std::invalid_argument);
}

TEST(MPPIOptimizerTest, AllInvalidRolloutsReturnHoldCommand) {
  mppi_core::MPPIConfig config;
  config.horizon_steps = 2;
  config.num_rollouts = 8;
  config.action_dim = 1;
  config.dt = 0.1;
  config.temperature = 1.0;
  config.random_seed = 7;
  config.action_lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  config.action_upper_bound = Eigen::VectorXd::Constant(1, 1.0);
  config.action_noise_std = Eigen::VectorXd::Constant(1, 0.5);

  auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(1);
  auto cost = std::make_shared<mppi_core::GraspStabilityCost>(
      mppi_core::GraspStabilityCostConfig{});
  mppi_core::MPPIOptimizer optimizer;
  optimizer.Initialize(config, model, cost);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.total_force_n.z() = 0.0;

  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Constant(1, 0.25);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(1);
  observation.tactile_meas = MakeTactileSensors(tactile, tactile);
  observation.time_s = 12.34;

  const auto command = optimizer.Update(observation);

  ASSERT_EQ(command.q_cmd.size(), 1);
  ASSERT_EQ(command.qdot_cmd.size(), 1);
  ASSERT_EQ(command.tau_cmd.size(), 1);
  ASSERT_EQ(command.kp.size(), 1);
  ASSERT_EQ(command.kd.size(), 1);
  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.IsUsable());
  EXPECT_NEAR(command.q_cmd[0], 0.25, kTolerance);
  EXPECT_NEAR(command.qdot_cmd[0], 0.0, kTolerance);
  EXPECT_NEAR(command.tau_cmd[0], 0.0, kTolerance);
  EXPECT_NEAR(command.kp[0], 0.0, kTolerance);
  EXPECT_NEAR(command.kd[0], 0.0, kTolerance);
  EXPECT_NEAR(command.stamp_sec, 12.34, kTolerance);
}

TEST(MPPIOptimizerTest, UpdateExposesSelectedActionMetadata) {
  mppi_core::MPPIConfig config;
  config.horizon_steps = 2;
  config.num_rollouts = 1;
  config.action_dim = 1;
  config.dt = 0.1;
  config.temperature = 1.0;
  config.action_lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  config.action_upper_bound = Eigen::VectorXd::Constant(1, 1.0);
  config.action_noise_std = Eigen::VectorXd::Zero(1);

  auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(1);
  mppi_core::MPPIOptimizer optimizer;
  optimizer.Initialize(config, model, nullptr);

  mppi_core::GraspObservation observation;
  observation.q_ref_current = Eigen::VectorXd::Constant(1, 0.25);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(1);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(1);
  observation.time_s = 2.0;

  const auto command = optimizer.Update(observation);

  EXPECT_TRUE(command.IsUsable());
  EXPECT_TRUE(optimizer.hasLastSelectedAction());
  EXPECT_EQ(optimizer.lastSelectedAction().size(), 1);
  EXPECT_TRUE(optimizer.lastSelectedAction().isApprox(Eigen::VectorXd::Zero(1)));
  EXPECT_TRUE(optimizer.hasLastSelectedActionSequence());
  EXPECT_EQ(optimizer.lastSelectedActionSequence().actionDim(), 1U);
  EXPECT_EQ(optimizer.lastSelectedActionSequence().horizonSteps(), 2U);
  EXPECT_DOUBLE_EQ(optimizer.lastNominalTotalCost(), 0.0);
}

TEST(MPPIOptimizerTest, PredictRolloutAllowsPinocchioNqDifferentFromNv) {
  const auto sensor_model = MakeSingleUnboundedRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::MPPIConfig config;
  config.horizon_steps = 1;
  config.num_rollouts = 1;
  config.action_dim = sensor_model.model.nv;
  config.dt = 0.1;
  config.temperature = 1.0;
  config.action_lower_bound =
      Eigen::VectorXd::Constant(sensor_model.model.nv, -1.0);
  config.action_upper_bound =
      Eigen::VectorXd::Constant(sensor_model.model.nv, 1.0);
  config.action_noise_std = Eigen::VectorXd::Zero(sensor_model.model.nv);

  auto model = std::make_shared<mppi_core::GraspStateRolloutModel>(
      sensor_model.model.nv);
  mppi_core::MPPIOptimizer optimizer;
  optimizer.Initialize(config, model, nullptr);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetActiveHemispheresAroundCentroid(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.hemisphere_index = 0;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::ContactForceProjectionConfig projection_config;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::RobotSystem robot_system(sensor_model.model);

  mppi_core::GraspObservation observation;
  observation.q_ref_current = pinocchio::neutral(sensor_model.model);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.tactile_meas = MakeTactileSensors(tactile, inactive_tactile);
  observation.robot_system = &robot_system;
  observation.tactile_contexts = MakeTactileContextsForStates(
      &kinematics, tactile, &kinematics, inactive_tactile);
  observation.contact_force_projection_config = &projection_config;
  observation.contact_force_rollout_config = &force_rollout_config;

  mppi_core::ActionSequence actions(sensor_model.model.nv, 1);
  actions.setAction(0, Eigen::VectorXd::Constant(sensor_model.model.nv, 0.2));

  const auto trace = optimizer.PredictRollout(observation, actions);

  ASSERT_EQ(trace.states.size(), 2U);
  EXPECT_TRUE(trace.states[1].valid);
  EXPECT_EQ(trace.states[1].robot.q.size(), sensor_model.model.nq);
  EXPECT_EQ(trace.states[1].robot.qdot.size(), sensor_model.model.nv);
  EXPECT_EQ(trace.states[1].robot.tau.size(), sensor_model.model.nv);
  EXPECT_TRUE(trace.states[1].robot.q.allFinite());
}
