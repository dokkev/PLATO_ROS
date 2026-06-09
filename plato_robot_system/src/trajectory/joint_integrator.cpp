#include "plato_robot_system/trajectory/joint_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace plato_robot_system::trajectory
{

namespace
{
constexpr double pi = 3.141592653589793238462643383279502884;
}

JointIntegrator::JointIntegrator(
  int num_joints, double dt, const Eigen::VectorXd & pos_min, const Eigen::VectorXd & pos_max,
  const Eigen::VectorXd & vel_min, const Eigen::VectorXd & vel_max)
: num_joints_(num_joints),
  dt_(dt),
  pos_min_(pos_min),
  pos_max_(pos_max),
  vel_min_(vel_min),
  vel_max_(vel_max),
  alpha_pos_(0.0),
  alpha_vel_(0.0),
  pos_max_error_(Eigen::VectorXd::Zero(num_joints)),
  q_cmd_(Eigen::VectorXd::Zero(num_joints)),
  qdot_cmd_(Eigen::VectorXd::Zero(num_joints)),
  is_initialized_(false)
{
}

void JointIntegrator::SetCutoffFrequency(double pos_cutoff_freq, double vel_cutoff_freq)
{
  alpha_pos_ = AlphaFromFrequency(pos_cutoff_freq, dt_);
  alpha_vel_ = AlphaFromFrequency(vel_cutoff_freq, dt_);
}

void JointIntegrator::SetMaxPositionError(double pos_max_error)
{
  pos_max_error_ = pos_max_error * Eigen::VectorXd::Ones(num_joints_);
}

void JointIntegrator::Initialize(const Eigen::VectorXd & q_init, const Eigen::VectorXd & qdot_init)
{
  q_cmd_ = q_init;
  qdot_cmd_ = qdot_init;
  is_initialized_ = true;
}

void JointIntegrator::Integrate(
  const Eigen::VectorXd & qddot_cmd, const Eigen::VectorXd & q,
  const Eigen::VectorXd & /*qdot*/, Eigen::VectorXd & q_cmd, Eigen::VectorXd & qdot_cmd)
{
  if (!is_initialized_) {
    std::cerr << "[plato_robot_system::trajectory::JointIntegrator] Not initialized. "
              << "Call Initialize() first." << std::endl;
    return;
  }

  qdot_cmd_ = (1.0 - alpha_vel_) * qdot_cmd_;
  qdot_cmd_ += qddot_cmd * dt_;
  qdot_cmd = Clamp(qdot_cmd_, vel_min_, vel_max_);
  qdot_cmd_ = qdot_cmd;

  q_cmd_ = (1.0 - alpha_pos_) * q_cmd_ + alpha_pos_ * q;
  q_cmd_ += qdot_cmd_ * dt_;
  q_cmd_ = Clamp(q_cmd_, q - pos_max_error_, q + pos_max_error_);
  q_cmd = Clamp(q_cmd_, pos_min_, pos_max_);
  q_cmd_ = q_cmd;
}

double JointIntegrator::AlphaFromFrequency(double hz, double dt)
{
  const double omega = 2.0 * pi * hz;
  return std::clamp((omega * dt) / (1.0 + omega * dt), 0.0, 1.0);
}

Eigen::VectorXd JointIntegrator::Clamp(
  const Eigen::VectorXd & value, const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
{
  return value.cwiseMax(lower).cwiseMin(upper);
}

}  // namespace plato_robot_system::trajectory
