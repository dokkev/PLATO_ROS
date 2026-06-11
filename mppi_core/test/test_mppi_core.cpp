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
#include <pinocchio/spatial/se3.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mppi_core/config/mppi_config.hpp"
#include "mppi_core/config/rollout_config.hpp"
#include "mppi_core/contact/contact_force_correction.hpp"
#include "mppi_core/contact/contact_force_projection.hpp"
#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/core/mppi_optimizer.hpp"
#include "mppi_core/costs/grasp_stability_cost.hpp"
#include "mppi_core/logging/mppi_rollout_logger.hpp"
#include "mppi_core/logging/vector_csv.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/rollout/contact_force_rollout.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
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
      MppiCorePackageRoot() / "task" / "jenga.yaml";
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
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(config.action_lower_bound[i], -0.003, kTolerance);
    EXPECT_NEAR(config.action_upper_bound[i], 0.004, kTolerance);
    EXPECT_NEAR(config.action_noise_std[i], 0.001, kTolerance);
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
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(1);
  observation.tactile_meas = MakeTactileSensors(tactile, inactive_tactile);
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
  EXPECT_NEAR(trace.states[0].robot.q[0], 0.5, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.q[0], 0.501, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.qdot[0], 0.01, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.q[0], 0.5015, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.qdot[0], 0.005, kTolerance);
  EXPECT_NEAR(trace.actions[0][0], 0.1, kTolerance);
  EXPECT_NEAR(trace.actions[1][0], -0.05, kTolerance);
  EXPECT_TRUE(trace.states[1].valid);
  EXPECT_TRUE(trace.states[1].tactile_sensors[0].valid);
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
