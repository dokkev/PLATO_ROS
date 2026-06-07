// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>
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
#include <vector>

#include "mppi_core/config/grasp_config.hpp"
#include "mppi_core/config/mppi_config.hpp"
#include "mppi_core/contact/contact_force_correction.hpp"
#include "mppi_core/contact/contact_force_projection.hpp"
#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/core/mppi_optimizer.hpp"
#include "mppi_core/costs/grasp_stability_cost.hpp"
#include "mppi_core/robot/robot_command.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/rollout/contact_force_rollout.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/nari_touch_adapter.hpp"
#include "mppi_core/tactile/tactile_transition.hpp"

namespace {

constexpr double kTolerance = 1.0e-9;

mppi_core::GraspState MakeState(std::size_t dim,
                                const mppi_core::TactileState& tactile) {
  const Eigen::VectorXd zero =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim));
  return mppi_core::MakeGraspState(zero, zero, zero, zero, tactile, tactile);
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
      Eigen::VectorXd::Zero(model.nv), Eigen::VectorXd::Zero(model.nv), tactile,
      tactile);
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

void SetHemispherePatch(mppi_core::TactileState* tactile,
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

mppi_core::RobotDynamicsContext MakeRobotDynamicsContext(
    const pinocchio::Model& model, pinocchio::Data* data) {
  mppi_core::RobotDynamicsContext dynamics;
  dynamics.model = &model;
  dynamics.data = data;
  return dynamics;
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
  EXPECT_EQ(command.q_des.size(), 7);
  EXPECT_EQ(command.qdot_des.size(), 6);
  EXPECT_EQ(command.qddot_des.size(), 6);
  EXPECT_EQ(command.tau_ff.size(), 6);
  EXPECT_EQ(command.kp.size(), 6);
  EXPECT_EQ(command.kd.size(), 6);

  command.qdot_des = Eigen::VectorXd::Zero(5);
  EXPECT_FALSE(command.HasValidDimensions());
  EXPECT_FALSE(command.IsUsable());
}

TEST(RobotCommandTest, ZeroHoldAndInvalidHelpersSetUsability) {
  const Eigen::VectorXd q_current = Eigen::VectorXd::Constant(2, 0.3);
  const Eigen::VectorXd qdot_current = Eigen::VectorXd::Constant(2, 0.4);
  const Eigen::VectorXd kp_hold = Eigen::VectorXd::Constant(2, 20.0);
  const Eigen::VectorXd kd_hold = Eigen::VectorXd::Constant(2, 2.0);

  const auto hold = mppi_core::MakeZeroHoldRobotCommand(q_current, qdot_current,
                                                        kp_hold, kd_hold);

  EXPECT_TRUE(hold.valid);
  EXPECT_TRUE(hold.IsUsable());
  EXPECT_NEAR(hold.q_des[0], 0.3, kTolerance);
  EXPECT_NEAR(hold.qdot_des.norm(), 0.0, kTolerance);
  EXPECT_NEAR(hold.qddot_des.norm(), 0.0, kTolerance);
  EXPECT_NEAR(hold.tau_ff.norm(), 0.0, kTolerance);
  EXPECT_NEAR(hold.kp[0], 20.0, kTolerance);
  EXPECT_NEAR(hold.kd[0], 2.0, kTolerance);

  const auto invalid = mppi_core::MakeInvalidRobotCommand(3, 2);
  EXPECT_FALSE(invalid.valid);
  EXPECT_TRUE(invalid.HasValidDimensions());
  EXPECT_TRUE(invalid.AllFinite());
  EXPECT_FALSE(invalid.IsUsable());
}

TEST(RobotSystemTest, StoresAndReturnsRobotStateForPinocchioModel) {
  const auto model = MakeSingleRevoluteZSensorModel();
  mppi_core::RobotSystem robot_system(model.model);

  EXPECT_TRUE(robot_system.hasModel());
  EXPECT_FALSE(robot_system.hasState());
  EXPECT_EQ(robot_system.nq(), model.model.nq);
  EXPECT_EQ(robot_system.nv(), model.model.nv);

  const Eigen::VectorXd q_des = pinocchio::neutral(model.model);
  const Eigen::VectorXd qdot_des =
      Eigen::VectorXd::Constant(model.model.nv, 0.2);
  const Eigen::VectorXd qddot_des =
      Eigen::VectorXd::Constant(model.model.nv, 0.3);
  const Eigen::VectorXd tau_ff = Eigen::VectorXd::Constant(model.model.nv, 0.4);

  robot_system.UpdateState(q_des, qdot_des, qddot_des, tau_ff, 1.25);

  ASSERT_TRUE(robot_system.hasState());
  const auto& state = robot_system.state();
  EXPECT_TRUE(state.q_des.isApprox(q_des));
  EXPECT_TRUE(state.qdot_des.isApprox(qdot_des));
  EXPECT_TRUE(state.qddot_des.isApprox(qddot_des));
  EXPECT_TRUE(state.tau_ff.isApprox(tau_ff));
  EXPECT_NEAR(state.time_s, 1.25, kTolerance);
}

TEST(RobotSystemTest, RejectsRobotStateDimensionMismatch) {
  const auto model = MakeSingleRevoluteZSensorModel();
  mppi_core::RobotSystem robot_system(model.model);

  const Eigen::VectorXd wrong_q = Eigen::VectorXd::Zero(model.model.nq + 1);
  const Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(model.model.nv);

  EXPECT_THROW(robot_system.UpdateState(wrong_q, qdot_des),
               std::invalid_argument);
}

TEST(GraspStateTest, MakeGraspStateValidatesDimensionsAndTactileValidity) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;

  const Eigen::VectorXd q = Eigen::VectorXd::Constant(2, 0.25);
  const Eigen::VectorXd dq = Eigen::VectorXd::Constant(2, -0.5);
  const Eigen::VectorXd qddot = Eigen::VectorXd::Zero(2);
  const Eigen::VectorXd tau = Eigen::VectorXd::Constant(2, 1.25);

  const auto valid_state =
      mppi_core::MakeGraspState(q, dq, qddot, tau, tactile, tactile);
  EXPECT_TRUE(valid_state.valid);
  EXPECT_EQ(valid_state.robot.q_des.size(), 2);
  EXPECT_EQ(valid_state.robot.qdot_des.size(), 2);
  EXPECT_EQ(valid_state.robot.tau_ff.size(), 2);
  EXPECT_NEAR(valid_state.robot.q_des[0], 0.25, kTolerance);
  EXPECT_NEAR(valid_state.robot.qdot_des[1], -0.5, kTolerance);
  EXPECT_NEAR(valid_state.robot.tau_ff[0], 1.25, kTolerance);
  EXPECT_TRUE(valid_state.tactile_sensors[0].valid);
  EXPECT_NEAR(valid_state.tactile_sensors[0].total_force_n.z(), 1.0,
              kTolerance);

  tactile.valid = false;
  const auto invalid_tactile_state =
      mppi_core::MakeGraspState(q, dq, qddot, tau, tactile, tactile);
  EXPECT_FALSE(invalid_tactile_state.valid);

  tactile.valid = true;
  const Eigen::VectorXd mismatched_dq = Eigen::VectorXd::Zero(3);
  const auto mismatched_state = mppi_core::MakeGraspState(
      q, mismatched_dq, Eigen::VectorXd::Zero(3), tau, tactile, tactile);
  EXPECT_FALSE(mismatched_state.valid);
  EXPECT_EQ(mismatched_state.robot.q_des.size(), 2);
  EXPECT_EQ(mismatched_state.robot.qdot_des.size(), 3);

  const Eigen::VectorXd wrong_tau = Eigen::VectorXd::Zero(3);
  const auto bad_torque_state =
      mppi_core::MakeGraspState(q, dq, qddot, wrong_tau, tactile, tactile);
  EXPECT_FALSE(bad_torque_state.valid);

  const Eigen::VectorXd q_nq_not_nv = Eigen::VectorXd::Constant(3, 0.2);
  const auto pinocchio_shaped_state =
      mppi_core::MakeGraspState(q_nq_not_nv, dq, qddot, tau, tactile, tactile);
  EXPECT_TRUE(pinocchio_shaped_state.valid);
  EXPECT_EQ(pinocchio_shaped_state.robot.q_des.size(), 3);
  EXPECT_EQ(pinocchio_shaped_state.robot.qdot_des.size(), 2);
  EXPECT_EQ(pinocchio_shaped_state.robot.tau_ff.size(), 2);
}

TEST(GraspStateTest, MakeGraspStateRejectsNonFiniteJointVectors) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(2);

  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(2), tau,
                                         tactile, tactile)
                   .valid);

  q[0] = 0.0;
  dq[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(2), tau,
                                         tactile, tactile)
                   .valid);

  dq[1] = 0.0;
  tau[0] = std::numeric_limits<double>::quiet_NaN();
  const auto bad_torque_state = mppi_core::MakeGraspState(
      q, dq, Eigen::VectorXd::Zero(2), tau, tactile, tactile);
  EXPECT_FALSE(bad_torque_state.valid);
}

TEST(GraspStateTest, ExplicitTwoSensorStateRequiresBothTactileStates) {
  const mppi_core::RobotState robot = mppi_core::MakeRobotState(
      Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2));

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
  state.robot.q_des[0] = 0.35;
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
                               state.robot.q_des);
  pinocchio::updateFramePlacements(sensor_model.model, finite_difference_data);
  const Eigen::Matrix3d current_sensor_rotation =
      finite_difference_data.oMf[sensor_model.sensor_frame_id].rotation();
  const Eigen::Vector3d point_world_before = WorldPointPosition(
      sensor_model.model, &finite_difference_data, sensor_model.sensor_frame_id,
      state.robot.q_des, HemispherePointSensorM(point));
  const Eigen::VectorXd q_next = pinocchio::integrate(
      sensor_model.model, state.robot.q_des, eps * tangent_step);
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
      Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), tactile, tactile);
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
  SetHemispherePatch(&tactile, 2, TestTactileCentroidM(tactile),
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
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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

TEST(GraspRolloutTest, TangentialMotionUpdatesCentroidAndShear) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d::Zero();
  tactile.rotational_shear_rad = 0.0;
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::HemisphereMotion first;
  first.position_sensor_m = Eigen::Vector3d{-0.001, 0.0, 0.0};
  first.delta_position_sensor_m = Eigen::Vector3d{0.001, -0.0005, 0.0};
  mppi_core::HemisphereMotion second;
  second.position_sensor_m = Eigen::Vector3d{0.001, 0.0, 0.0};
  second.delta_position_sensor_m = first.delta_position_sensor_m;

  mppi_core::GraspRolloutConfig config;
  config.tangential_confidence_loss_per_m = 0.0;
  config.edge_confidence_loss_gain = 0.0;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{first, second}, 0.01,
      config);
  EXPECT_NEAR(TestTactileCentroidM(next).x(), 0.001, kTolerance);
  EXPECT_NEAR(TestTactileCentroidM(next).y(), -0.0005, kTolerance);
  EXPECT_NEAR(next.shear_displacement_m.x(), 0.001, kTolerance);
  EXPECT_NEAR(next.shear_displacement_m.y(), -0.0005, kTolerance);
  EXPECT_NEAR(next.rotational_shear_rad, 0.0, kTolerance);
  EXPECT_NEAR(next.total_force_n.z(), 1.0, kTolerance);
  EXPECT_EQ(next.contact_state, mppi_core::TactileState::kEnoughContacts);
}

TEST(GraspRolloutTest, TangentialMotionAlongShearIncreasesSlipScore) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{1.0e-3, 0.0};
  SetHemispherePatch(&tactile, 2, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d::Zero();
  motion.delta_position_sensor_m = Eigen::Vector3d{1.0e-3, 0.0, 0.0};

  mppi_core::GraspRolloutConfig config;
  config.min_active_hemisphere_count = 2;
  config.tangential_confidence_loss_per_m = 0.0;
  config.edge_confidence_loss_gain = 0.0;
  config.shear_ref_m = 1.0e-3;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  EXPECT_GT(next.shear_displacement_m.norm(),
            tactile.shear_displacement_m.norm());
  EXPECT_GT(next.incipient_slip_score, 1.0);
}

TEST(GraspRolloutTest, TangentialMotionOppositeShearReducesSlipScore) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{2.0e-3, 0.0};
  SetHemispherePatch(&tactile, 2, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d::Zero();
  motion.delta_position_sensor_m = Eigen::Vector3d{-1.0e-3, 0.0, 0.0};

  mppi_core::GraspRolloutConfig config;
  config.min_active_hemisphere_count = 2;
  config.tangential_confidence_loss_per_m = 0.0;
  config.edge_confidence_loss_gain = 0.0;
  config.shear_ref_m = 1.0e-3;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  EXPECT_LT(next.shear_displacement_m.norm(),
            tactile.shear_displacement_m.norm());
  EXPECT_LT(next.incipient_slip_score, 2.0);
}

TEST(GraspRolloutTest, RelativeTangentialMotionUpdatesRotationalShear) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.rotational_shear_rad = 0.0;
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::HemisphereMotion left;
  left.position_sensor_m = Eigen::Vector3d{-0.001, 0.0, 0.0};
  left.delta_position_sensor_m = Eigen::Vector3d{0.0, -0.0001, 0.0};
  mppi_core::HemisphereMotion right;
  right.position_sensor_m = Eigen::Vector3d{0.001, 0.0, 0.0};
  right.delta_position_sensor_m = Eigen::Vector3d{0.0, 0.0001, 0.0};

  mppi_core::GraspRolloutConfig config;
  config.tangential_confidence_loss_per_m = 0.0;
  config.edge_confidence_loss_gain = 0.0;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{left, right}, 0.01,
      config);
  EXPECT_NEAR(TestTactileCentroidM(next).norm(), 0.0, kTolerance);
  EXPECT_NEAR(next.rotational_shear_rad, 0.1, kTolerance);
}

TEST(GraspRolloutTest,
     SeparatingMotionReducesNormalForceConfidenceAndContactQuality) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d::Zero();
  motion.delta_position_sensor_m = Eigen::Vector3d{0.0, 0.0, -0.002};

  mppi_core::GraspRolloutConfig config;
  config.normal_force_gain_n_per_m = 100.0;
  config.separating_confidence_loss_per_m = 200.0;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  EXPECT_NEAR(next.total_force_n.z(), 0.8, kTolerance);
  EXPECT_NEAR(next.confidence, 0.6, kTolerance);
  EXPECT_EQ(next.contact_state, mppi_core::TactileState::kEnoughContacts);
}

TEST(GraspRolloutTest, ClosingMotionIncreasesNormalForceAndConfidence) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kFewContacts;
  tactile.total_force_n.z() = 0.5;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 2, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 0.5;

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d::Zero();
  motion.delta_position_sensor_m = Eigen::Vector3d{0.0, 0.0, 0.002};

  mppi_core::GraspRolloutConfig config;
  config.min_active_hemisphere_count = 2;
  config.normal_force_gain_n_per_m = 100.0;
  config.closing_confidence_gain_per_m = 20.0;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  EXPECT_NEAR(next.total_force_n.z(), 0.7, kTolerance);
  EXPECT_NEAR(next.confidence, 0.54, kTolerance);
  EXPECT_EQ(next.contact_state, mppi_core::TactileState::kEnoughContacts);
}

TEST(GraspRolloutTest,
     EdgeMigrationReducesConfidenceAndKeepsEdgeRiskDiagnostic) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d{0.0029, 0.0}, tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d{0.0029, 0.0, 0.0};
  motion.delta_position_sensor_m = Eigen::Vector3d{0.001, 0.0, 0.0};

  mppi_core::GraspRolloutConfig config;
  config.tangential_confidence_loss_per_m = 0.0;
  config.edge_confidence_loss_gain = 0.5;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  const double next_edge_risk = mppi_core::ComputeTactileContactEdgeRisk(
      TestTactileCentroidM(next), config);
  EXPECT_GT(next_edge_risk, 1.0);
  EXPECT_LT(next.confidence, tactile.confidence);
}

TEST(GraspRolloutTest, CenteringMotionReducesEdgeRiskAndMaintainsConfidence) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d{0.0027, 0.0}, tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 0.8;

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d{0.0027, 0.0, 0.0};
  motion.delta_position_sensor_m = Eigen::Vector3d{-0.001, 0.0, 0.0};

  mppi_core::GraspRolloutConfig config;
  config.tangential_confidence_loss_per_m = 0.0;
  config.edge_confidence_loss_gain = 0.5;

  const double initial_edge_risk = mppi_core::ComputeTactileContactEdgeRisk(
      TestTactileCentroidM(tactile), config);
  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  EXPECT_LT(mppi_core::ComputeTactileContactEdgeRisk(TestTactileCentroidM(next),
                                                     config),
            initial_edge_risk);
  EXPECT_NEAR(next.confidence, tactile.confidence, kTolerance);
}

TEST(GraspRolloutTest, LargeSeparatingMotionCanLoseContactAndDeactivatePoints) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d::Zero();
  tactile.hemispheres.push_back(point);

  mppi_core::HemisphereMotion motion;
  motion.position_sensor_m = Eigen::Vector3d::Zero();
  motion.delta_position_sensor_m = Eigen::Vector3d{0.0, 0.0, -0.01};

  mppi_core::GraspRolloutConfig config;
  config.min_active_hemisphere_count = 1;
  config.normal_force_gain_n_per_m = 100.0;
  config.separating_confidence_loss_per_m = 200.0;

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{motion}, 0.01, config);

  EXPECT_NEAR(next.total_force_n.z(), 0.0, kTolerance);
  EXPECT_NEAR(next.confidence, 0.0, kTolerance);
  EXPECT_EQ(next.contact_state, mppi_core::TactileState::kNoContact);
  EXPECT_EQ(next.activeHemisphereCount(), 0U);
  EXPECT_EQ(next.activeHemisphereCount(), 0U);
}

TEST(GraspRolloutTest, InvalidInputsDoNotProduceNonFiniteState) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d::Zero();
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 0.7;

  mppi_core::HemisphereMotion invalid;
  invalid.position_sensor_m =
      Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  invalid.delta_position_sensor_m = Eigen::Vector3d{1.0, 0.0, 0.0};

  const auto next = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{invalid}, 0.01);

  EXPECT_TRUE(TestTactileCentroidM(next).allFinite());
  EXPECT_TRUE(next.shear_displacement_m.allFinite());
  EXPECT_TRUE(std::isfinite(next.total_force_n.z()));
  EXPECT_TRUE(std::isfinite(next.confidence));

  mppi_core::HemisphereMotion valid;
  valid.position_sensor_m = Eigen::Vector3d::Zero();
  valid.delta_position_sensor_m = Eigen::Vector3d{0.001, 0.0, 0.0};
  const auto zero_dt = mppi_core::StepTactileTransition(
      tactile, std::vector<mppi_core::HemisphereMotion>{valid}, 0.0);
  EXPECT_NEAR(TestTactileCentroidM(zero_dt).x(), 0.0, kTolerance);
  EXPECT_NEAR(zero_dt.shear_displacement_m.x(), 0.0, kTolerance);
}

TEST(GraspRolloutTest, RefreshUsesNormalizedPredictedSlipScore) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  tactile.shear_displacement_m = Eigen::Vector2d{1.0e-3, 0.0};
  tactile.rotational_shear_rad = 2.0e-2;
  SetHemispherePatch(&tactile, 4, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::GraspRolloutConfig config;
  config.shear_ref_m = 1.0e-3;
  config.rotational_shear_ref_rad = 2.0e-2;

  mppi_core::RefreshPredictedTactileFields(&tactile, config);

  EXPECT_EQ(tactile.contact_state, mppi_core::TactileState::kEnoughContacts);
  EXPECT_NEAR(tactile.slip_score, 2.0, kTolerance);
  EXPECT_NEAR(tactile.incipient_slip_score, 2.0, kTolerance);
}

TEST(GraspRolloutTest, RefreshMarksHemispheresInactiveOnContactLoss) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.0;
  SetHemispherePatch(&tactile, 2, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::GraspRolloutConfig config;
  mppi_core::RefreshPredictedTactileFields(&tactile, config);

  EXPECT_EQ(tactile.contact_state, mppi_core::TactileState::kNoContact);
  EXPECT_EQ(tactile.activeHemisphereCount(), 0U);
  EXPECT_EQ(tactile.hemispheres.size(), 2U);
  EXPECT_FALSE(tactile.hemispheres[0].contact);
  EXPECT_FALSE(tactile.hemispheres[1].contact);
}

TEST(GraspStateRolloutModelTest,
     MeasuredTorqueResidualDrivesGraspStateTransition) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;
  const mppi_core::TactileState inactive_tactile =
      MakeInactiveTactileState(tactile);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  const auto dynamics = MakeRobotDynamicsContext(sensor_model.model, &data);

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
  observation.robot_dynamics = &dynamics;
  const auto state = mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(1),
                                               Eigen::VectorXd::Zero(1),
                                               tactile, inactive_tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  mppi_core::GraspState next_state;
  mppi_core::RolloutContext context;
  context.observation = &observation;
  context.robot_dynamics = &dynamics;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;
  context.contact_force_correction_state = &correction;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(), 1.5, 1.0e-6);
  EXPECT_EQ(next_state.tactile_sensors[0].contact_state,
            mppi_core::TactileState::kEnoughContacts);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 1U);
  EXPECT_EQ(next_state.tactile_sensors[1].activeHemisphereCount(), 0U);
  EXPECT_NEAR(next_state.robot.q_des[0], 0.0, kTolerance);
  EXPECT_NEAR(next_state.robot.tau_ff[0], 0.0, kTolerance);
}

TEST(GraspStateRolloutModelTest,
     TwoActiveTactileSensorsUseKinematicFallbackWhenResidualRequired) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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

  const auto state =
      mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(1),
                                Eigen::VectorXd::Zero(1), tactile, tactile);

  mppi_core::RolloutContext context;
  context.observation = &observation;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
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

TEST(GraspStateRolloutModelTest, RejectsNonFiniteAction) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;

  const auto state = mppi_core::MakeGraspState(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1),
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), tactile, tactile);
  Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  action[0] = std::numeric_limits<double>::quiet_NaN();

  mppi_core::GraspStateRolloutConfig config;
  config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, config);
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
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), tactile, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.02);
  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(nullptr, nullptr);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
  EXPECT_NEAR(next_state.robot.q_des[0], 0.0402, kTolerance);
  EXPECT_NEAR(next_state.robot.qdot_des[0], 0.402, kTolerance);
  EXPECT_NEAR(next_state.robot.qddot_des[0], 0.02, kTolerance);
  EXPECT_NEAR(next_state.robot.tau_ff[0], 0.0, kTolerance);
}

TEST(GraspStateRolloutModelTest,
     TactileOneUsesItsOwnKinematicsForFallbackRollout) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile1;
  tactile1.valid = true;
  tactile1.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile1.total_force_n.z() = 1.0;
  SetHemispherePatch(&tactile1, 1, TestTactileCentroidM(tactile1),
                     tactile1.total_force_n.z());
  tactile1.confidence = 1.0;
  mppi_core::TactileState tactile0 = MakeInactiveTactileState(tactile1);

  mppi_core::PinocchioContactKinematicsContext closing_side_kinematics;
  closing_side_kinematics.model = &sensor_model.model;
  closing_side_kinematics.data = &data;
  closing_side_kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  closing_side_kinematics.normal_axis_sign = -1.0;

  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.normal_force_gain_n_per_m = 100.0;

  mppi_core::RolloutContext context;
  context.tactile_contexts =
      MakeTactileContexts(nullptr, &closing_side_kinematics);
  context.grasp_rollout_config = &rollout_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd dq = Eigen::VectorXd::Zero(1);
  const auto state =
      mppi_core::MakeGraspState(q, dq, Eigen::VectorXd::Zero(1),
                                Eigen::VectorXd::Zero(1), tactile0, tactile1);
  mppi_core::GraspState next_state;

  model.Step(state, Eigen::VectorXd::Constant(1, 0.1), context, 0.1,
             &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 0U);
  EXPECT_LT(next_state.tactile_sensors[1].total_force_n.z(),
            tactile1.total_force_n.z());
}

TEST(GraspStateRolloutModelTest, ComputesFeedForwardTorqueWithRnea) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);
  pinocchio::Data expected_data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.0;
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  const auto dynamics = MakeRobotDynamicsContext(sensor_model.model, &data);
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::RolloutContext context;
  context.robot_dynamics = &dynamics;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd qdot = Eigen::VectorXd::Constant(1, 0.4);
  const auto state =
      mppi_core::MakeGraspState(q, qdot, Eigen::VectorXd::Zero(1),
                                Eigen::VectorXd::Zero(1), tactile, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.2);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.robot.q_des[0], 0.042, kTolerance);
  EXPECT_NEAR(next_state.robot.qdot_des[0], 0.42, kTolerance);
  EXPECT_NEAR(next_state.robot.qddot_des[0], 0.2, kTolerance);
  const Eigen::VectorXd expected_tau =
      pinocchio::rnea(sensor_model.model, expected_data, next_state.robot.q_des,
                      next_state.robot.qdot_des, next_state.robot.qddot_des);
  ASSERT_EQ(expected_tau.size(), next_state.robot.tau_ff.size());
  EXPECT_NEAR(next_state.robot.tau_ff[0], expected_tau[0], kTolerance);
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
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  const auto dynamics = MakeRobotDynamicsContext(sensor_model.model, &data);
  mppi_core::ContactForceProjectionConfig projection_config;
  mppi_core::ContactForceRolloutConfig force_rollout_config;
  force_rollout_config.min_active_hemisphere_count = 1;

  mppi_core::RolloutContext context;
  context.robot_dynamics = &dynamics;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.contact_force_projection_config = &projection_config;
  context.contact_force_rollout_config = &force_rollout_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.2);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_EQ(next_state.robot.q_des.size(), 2);
  EXPECT_EQ(next_state.robot.qdot_des.size(), 1);
  EXPECT_EQ(next_state.robot.tau_ff.size(), 1);
  EXPECT_TRUE(next_state.robot.q_des.allFinite());
  EXPECT_NEAR(next_state.robot.qdot_des[0], 0.02, kTolerance);
  EXPECT_NEAR(next_state.robot.qddot_des[0], 0.2, kTolerance);
  const Eigen::VectorXd expected_tau =
      pinocchio::rnea(sensor_model.model, data, next_state.robot.q_des,
                      next_state.robot.qdot_des, next_state.robot.qddot_des);
  ASSERT_EQ(expected_tau.size(), next_state.robot.tau_ff.size());
  EXPECT_NEAR(next_state.robot.tau_ff[0], expected_tau[0], kTolerance);
}

TEST(GraspStateRolloutModelTest,
     ResidualTransitionFailureFallsBackToKinematicPatchRollout) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.normal_force_gain_n_per_m = 100.0;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.grasp_rollout_config = &rollout_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_TRUE(next_state.valid);
  EXPECT_NEAR(next_state.tactile_sensors[0].total_force_n.z(), 0.51, 1.0e-6);
  EXPECT_EQ(next_state.tactile_sensors[0].contact_state,
            mppi_core::TactileState::kEnoughContacts);
}

TEST(GraspStateRolloutModelTest,
     RequireResidualTransitionMarksForceProjectionFailureInvalid) {
  const auto sensor_model = MakeSinglePrismaticZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 0.5;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.normal_force_gain_n_per_m = 100.0;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.grasp_rollout_config = &rollout_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutConfig config;
  config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualRequired;
  mppi_core::GraspStateRolloutModel model(1, config);
  const auto state = MakeContactKinematicsState(sensor_model.model, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
  EXPECT_TRUE(next_state.robot.q_des.allFinite());
  EXPECT_TRUE(next_state.robot.qdot_des.allFinite());
  EXPECT_TRUE(next_state.robot.tau_ff.allFinite());
}

TEST(GraspStateRolloutModelTest,
     RequireResidualTransitionRequiresActiveTactileContact) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kNoContact;
  tactile.total_force_n.z() = 0.0;
  SetHemispherePatch(&tactile, 0, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::GraspStateRolloutConfig config;
  config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualRequired;
  mppi_core::GraspStateRolloutModel model(1, config);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);
  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(nullptr, nullptr);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
}

TEST(GraspStateRolloutModelTest,
     ResidualThenKinematicFallbackRejectsMissingKinematics) {
  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 1.0;

  mppi_core::GraspStateRolloutConfig config;
  config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, config);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.01);
  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(nullptr, nullptr);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
}

TEST(GraspStateRolloutModelTest,
     PinocchioContactMotionsDriveTactilePatchRollout) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.tangential_confidence_loss_per_m = 0.0;
  rollout_config.edge_confidence_loss_gain = 0.0;
  rollout_config.shear_ref_m = 1.0;
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.grasp_rollout_config = &rollout_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
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
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d::Zero(), tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.tangential_confidence_loss_per_m = 0.0;
  rollout_config.edge_confidence_loss_gain = 0.0;
  rollout_config.shear_ref_m = 1.0;
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.grasp_rollout_config = &rollout_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, -0.005);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);
  EXPECT_LT(next_state.tactile_sensors[0].shear_displacement_m.norm(),
            tactile.shear_displacement_m.norm());
}

TEST(GraspStateRolloutModelTest,
     ZeroPinocchioActionLeavesPatchGeometryUnchanged) {
  const auto sensor_model = MakeSingleRevoluteZSensorModel();
  pinocchio::Data data(sensor_model.model);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(
      &tactile,
      tactile.activeHemisphereCount() > 0 ? tactile.activeHemisphereCount() : 1,
      Eigen::Vector2d{0.001, 0.0}, tactile.total_force_n.z());
  tactile.shear_displacement_m = Eigen::Vector2d{0.0, 0.01};
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());
  tactile.confidence = 0.8;
  mppi_core::HemisphereState point;
  point.contact = true;
  point.cop_sensor_m = Eigen::Vector2d{1.0, 0.0};
  tactile.hemispheres.push_back(point);

  mppi_core::PinocchioContactKinematicsContext kinematics;
  kinematics.model = &sensor_model.model;
  kinematics.data = &data;
  kinematics.sensor_frame_id = sensor_model.sensor_frame_id;
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.tangential_confidence_loss_per_m = 0.0;
  rollout_config.edge_confidence_loss_gain = 0.0;
  rollout_config.shear_ref_m = 1.0;
  mppi_core::ContactForceProjectionConfig projection_config;
  projection_config.enabled = false;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.grasp_rollout_config = &rollout_config;
  context.contact_force_projection_config = &projection_config;

  mppi_core::GraspStateRolloutConfig model_config;
  model_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback;
  mppi_core::GraspStateRolloutModel model(1, model_config);
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
     ResidualRequiredMarksNoActiveContactPointsInvalid) {
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
  mppi_core::GraspRolloutConfig rollout_config;
  rollout_config.min_active_hemisphere_count = 1;
  rollout_config.shear_ref_m = 1.0;

  mppi_core::RolloutContext context;
  context.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  context.grasp_rollout_config = &rollout_config;

  mppi_core::GraspStateRolloutModel model(1);
  const auto state = MakeState(1, tactile);
  const Eigen::VectorXd action = Eigen::VectorXd::Constant(1, 0.005);
  mppi_core::GraspState next_state;

  model.Step(state, action, context, 0.1, &next_state);

  EXPECT_FALSE(next_state.valid);
  EXPECT_EQ(next_state.tactile_sensors[0].activeHemisphereCount(), 0U);
}

TEST(GraspStabilityCostTest, PenalizesSmallPredictedContactPatch) {
  mppi_core::GraspStabilityCostConfig config;
  config.force_min_n = 0.0;
  config.force_max_n = 10.0;
  config.force_under_weight = 0.0;
  config.force_over_weight = 0.0;
  config.slip_risk_weight = 0.0;
  config.contact_centroid_enabled = true;
  config.centroid_boundary_weight = 0.0;
  config.contact_loss_weight = 0.0;
  config.tracking_weight = 0.0;
  config.tracking_action_scale_weight = 0.0;
  config.action_smoothness_weight = 0.0;
  config.joint_limit_weight = 0.0;
  config.hemisphere_contact_enabled = true;
  config.target_active_hemisphere_count = 6.0;
  config.hemisphere_contact_weight = 1.0;

  mppi_core::GraspStabilityCost cost(config);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  mppi_core::RolloutContext rollout;
  mppi_core::CostContext context;
  context.rollout = &rollout;
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);

  auto low_patch_state = MakeState(1, tactile);
  SetHemispherePatch(&low_patch_state.tactile_sensors[0], 2,
                     TestTactileCentroidM(low_patch_state.tactile_sensors[0]),
                     low_patch_state.tactile_sensors[0].total_force_n.z());
  SetHemispherePatch(&low_patch_state.tactile_sensors[1], 2,
                     TestTactileCentroidM(low_patch_state.tactile_sensors[1]),
                     low_patch_state.tactile_sensors[1].total_force_n.z());

  auto wide_patch_state = low_patch_state;
  SetHemispherePatch(&wide_patch_state.tactile_sensors[0], 6,
                     TestTactileCentroidM(wide_patch_state.tactile_sensors[0]),
                     wide_patch_state.tactile_sensors[0].total_force_n.z());
  SetHemispherePatch(&wide_patch_state.tactile_sensors[1], 6,
                     TestTactileCentroidM(wide_patch_state.tactile_sensors[1]),
                     wide_patch_state.tactile_sensors[1].total_force_n.z());

  const double low_patch_cost = cost.Evaluate(low_patch_state, action, context);
  const double wide_patch_cost =
      cost.Evaluate(wide_patch_state, action, context);

  EXPECT_GT(low_patch_cost, wide_patch_cost);
  EXPECT_NEAR(wide_patch_cost, 0.0, kTolerance);
}

TEST(GraspStabilityCostTest, DisablingCentroidCostDoesNotAddContactLossCost) {
  mppi_core::GraspStabilityCostConfig config;
  config.force_min_n = 0.0;
  config.force_max_n = 10.0;
  config.force_under_weight = 0.0;
  config.force_over_weight = 0.0;
  config.slip_risk_weight = 0.0;
  config.contact_centroid_enabled = false;
  config.centroid_boundary_weight = 0.0;
  config.contact_loss_weight = 10.0;
  config.hemisphere_contact_enabled = false;
  config.tracking_weight = 0.0;
  config.tracking_action_scale_weight = 0.0;
  config.action_smoothness_weight = 0.0;
  config.joint_limit_weight = 0.0;

  mppi_core::GraspStabilityCost cost(config);

  mppi_core::TactileState tactile;
  tactile.valid = true;
  tactile.contact_state = mppi_core::TactileState::kEnoughContacts;
  tactile.total_force_n.z() = 1.0;
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
                     tactile.total_force_n.z());

  auto state = MakeState(1, tactile);
  mppi_core::RolloutContext rollout;
  mppi_core::CostContext context;
  context.rollout = &rollout;
  const Eigen::VectorXd action = Eigen::VectorXd::Zero(1);

  EXPECT_NEAR(cost.Evaluate(state, action, context), 0.0, kTolerance);
}

TEST(GraspConfigTest, ParsesContactLocalCostAndForceRolloutConfig) {
  const YAML::Node root = YAML::Load(R"(
grasp:
  grasp_state_transition:
    rollout_policy: residual_then_kinematic_fallback
  contact_force_rollout:
    enable_force_projection_update: true
    force_lowpass_alpha: 0.7
    max_predicted_normal_force_n: 12.0
    shear_force_gain_m_per_n_s: 0.0002
    rotational_shear_gain_rad_per_nm_s: 0.03
    friction_violation_confidence_decay: 0.4
    negative_normal_confidence_decay: 0.6
    min_active_hemisphere_count: 2
    min_contact_confidence: 0.02
    shear_ref_m: 0.004
    rotational_shear_ref_rad: 0.05
  slip_risk:
    velocity_weight: 0.05
  hemisphere_contact:
    enabled: true
    target_active_hemisphere_count: 7
    weight: 3.5
)");

  const auto cost_config = mppi_core::ParseGraspConfig(
      root["grasp"], 2, mppi_core::GraspStabilityCostConfig{});
  const auto rollout_config = mppi_core::ParseGraspStateRolloutConfig(
      root["grasp"], mppi_core::GraspStateRolloutConfig{});
  const auto force_rollout_config = mppi_core::ParseContactForceRolloutConfig(
      root["grasp"], mppi_core::ContactForceRolloutConfig{});

  EXPECT_NEAR(cost_config.slip_velocity_weight, 0.05, kTolerance);
  EXPECT_TRUE(cost_config.hemisphere_contact_enabled);
  EXPECT_NEAR(cost_config.target_active_hemisphere_count, 7.0, kTolerance);
  EXPECT_NEAR(cost_config.hemisphere_contact_weight, 3.5, kTolerance);
  EXPECT_EQ(rollout_config.tactile_rollout_policy,
            mppi_core::TactileRolloutPolicy::kResidualThenKinematicFallback);
  EXPECT_TRUE(force_rollout_config.enable_force_projection_update);
  EXPECT_NEAR(force_rollout_config.force_lowpass_alpha, 0.7, kTolerance);
  EXPECT_NEAR(force_rollout_config.max_predicted_normal_force_n, 12.0,
              kTolerance);
  EXPECT_NEAR(force_rollout_config.shear_force_gain_m_per_n_s, 0.0002,
              kTolerance);
  EXPECT_NEAR(force_rollout_config.rotational_shear_gain_rad_per_nm_s, 0.03,
              kTolerance);
  EXPECT_NEAR(force_rollout_config.friction_violation_confidence_decay, 0.4,
              kTolerance);
  EXPECT_NEAR(force_rollout_config.negative_normal_confidence_decay, 0.6,
              kTolerance);
  EXPECT_EQ(force_rollout_config.min_active_hemisphere_count, 2U);
  EXPECT_NEAR(force_rollout_config.min_contact_confidence, 0.02, kTolerance);
  EXPECT_NEAR(force_rollout_config.shear_ref_m, 0.004, kTolerance);
  EXPECT_NEAR(force_rollout_config.rotational_shear_ref_rad, 0.05, kTolerance);
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
  command:
    kp: 30.0
    kd: 1.5
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
  ASSERT_EQ(config.command_kp.size(), 3);
  ASSERT_EQ(config.command_kd.size(), 3);
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(config.action_lower_bound[i], -0.003, kTolerance);
    EXPECT_NEAR(config.action_upper_bound[i], 0.004, kTolerance);
    EXPECT_NEAR(config.action_noise_std[i], 0.001, kTolerance);
    EXPECT_NEAR(config.command_kp[i], 30.0, kTolerance);
    EXPECT_NEAR(config.command_kd[i], 1.5, kTolerance);
  }
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
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  const auto dynamics = MakeRobotDynamicsContext(sensor_model.model, &data);
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
  observation.robot_dynamics = &dynamics;
  observation.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
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
  EXPECT_NEAR(trace.states[0].robot.q_des[0], 0.5, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.q_des[0], 0.501, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.qdot_des[0], 0.01, kTolerance);
  EXPECT_NEAR(trace.states[1].robot.qddot_des[0], 0.1, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.q_des[0], 0.5015, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.qdot_des[0], 0.005, kTolerance);
  EXPECT_NEAR(trace.states[2].robot.qddot_des[0], -0.05, kTolerance);
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

  mppi_core::GraspStateRolloutConfig prediction_config;
  prediction_config.tactile_rollout_policy =
      mppi_core::TactileRolloutPolicy::kResidualRequired;
  auto model =
      std::make_shared<mppi_core::GraspStateRolloutModel>(1, prediction_config);
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

  ASSERT_EQ(command.q_des.size(), 1);
  ASSERT_EQ(command.qdot_des.size(), 1);
  ASSERT_EQ(command.qddot_des.size(), 1);
  ASSERT_EQ(command.tau_ff.size(), 1);
  ASSERT_EQ(command.kp.size(), 1);
  ASSERT_EQ(command.kd.size(), 1);
  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.IsUsable());
  EXPECT_NEAR(command.q_des[0], 0.25, kTolerance);
  EXPECT_NEAR(command.qdot_des[0], 0.0, kTolerance);
  EXPECT_NEAR(command.qddot_des[0], 0.0, kTolerance);
  EXPECT_NEAR(command.tau_ff[0], 0.0, kTolerance);
  EXPECT_NEAR(command.kp[0], 20.0, kTolerance);
  EXPECT_NEAR(command.kd[0], 1.0, kTolerance);
  EXPECT_NEAR(command.stamp_sec, 12.34, kTolerance);
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
  SetHemispherePatch(&tactile, 1, TestTactileCentroidM(tactile),
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
  const auto dynamics = MakeRobotDynamicsContext(sensor_model.model, &data);

  mppi_core::GraspObservation observation;
  observation.q_ref_current = pinocchio::neutral(sensor_model.model);
  observation.qdot_ref_current = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.q_meas = observation.q_ref_current;
  observation.qdot_meas = observation.qdot_ref_current;
  observation.tau_meas = Eigen::VectorXd::Zero(sensor_model.model.nv);
  observation.tactile_meas = MakeTactileSensors(tactile, inactive_tactile);
  observation.robot_dynamics = &dynamics;
  observation.tactile_contexts = MakeTactileContexts(&kinematics, &kinematics);
  observation.contact_force_projection_config = &projection_config;
  observation.contact_force_rollout_config = &force_rollout_config;

  mppi_core::ActionSequence actions(sensor_model.model.nv, 1);
  actions.setAction(0, Eigen::VectorXd::Constant(sensor_model.model.nv, 0.2));

  const auto trace = optimizer.PredictRollout(observation, actions);

  ASSERT_EQ(trace.states.size(), 2U);
  EXPECT_TRUE(trace.states[1].valid);
  EXPECT_EQ(trace.states[1].robot.q_des.size(), sensor_model.model.nq);
  EXPECT_EQ(trace.states[1].robot.qdot_des.size(), sensor_model.model.nv);
  EXPECT_EQ(trace.states[1].robot.tau_ff.size(), sensor_model.model.nv);
  EXPECT_TRUE(trace.states[1].robot.q_des.allFinite());
}
