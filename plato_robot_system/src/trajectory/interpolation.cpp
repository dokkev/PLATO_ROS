#include "plato_robot_system/trajectory/interpolation.hpp"

#include <algorithm>
#include <cmath>

namespace plato_robot_system::trajectory
{

namespace
{
constexpr double pi = 3.141592653589793238462643383279502884;
}

double Smooth(double start, double finish, double ratio)
{
  if (ratio < 0.0) {
    return start;
  }
  if (ratio > 1.0) {
    return finish;
  }
  return start + (finish - start) * ratio;
}

double SmoothPos(double start, double finish, double duration, double time)
{
  if (duration <= 0.0) {
    return finish;
  }
  if (time > duration) {
    return finish;
  }
  return start + (finish - start) * 0.5 * (1.0 - std::cos(time / duration * pi));
}

double SmoothVel(double start, double finish, double duration, double time)
{
  if (duration <= 0.0 || time > duration) {
    return 0.0;
  }
  return (finish - start) * 0.5 * (pi / duration) * std::sin(time / duration * pi);
}

double SmoothAcc(double start, double finish, double duration, double time)
{
  if (duration <= 0.0 || time > duration) {
    return 0.0;
  }
  return (finish - start) * 0.5 * (pi / duration) * (pi / duration) *
         std::cos(time / duration * pi);
}

void SinusoidTrajectory(
  const Eigen::VectorXd & midpoint, const Eigen::VectorXd & amp, const Eigen::VectorXd & freq,
  double eval_time, Eigen::VectorXd & position, Eigen::VectorXd & velocity,
  Eigen::VectorXd & acceleration, double smoothing_duration)
{
  assert(amp.size() == freq.size());
  assert(midpoint.size() == amp.size());

  const int n = amp.size();
  position = Eigen::VectorXd::Zero(n);
  velocity = Eigen::VectorXd::Zero(n);
  acceleration = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    const double omega = 2.0 * pi * freq[i];
    position[i] = amp[i] * std::sin(omega * eval_time) + midpoint[i];
    velocity[i] = amp[i] * omega * std::cos(omega * eval_time);
    acceleration[i] = -amp[i] * omega * omega * std::sin(omega * eval_time);
  }

  if (eval_time < smoothing_duration) {
    const double phase = SmoothPos(0.0, 1.0, smoothing_duration, eval_time);
    for (int i = 0; i < n; ++i) {
      position[i] = (1.0 - phase) * midpoint[i] + phase * position[i];
      velocity[i] *= phase;
      acceleration[i] *= phase;
    }
  }
}

void SinusoidTrajectory(
  const Eigen::VectorXd & amp, const Eigen::VectorXd & freq, double eval_time,
  Eigen::VectorXd & position, Eigen::VectorXd & velocity, Eigen::VectorXd & acceleration,
  double smoothing_duration)
{
  assert(amp.size() == freq.size());

  const int n = amp.size();
  position = Eigen::VectorXd::Zero(n);
  velocity = Eigen::VectorXd::Zero(n);
  acceleration = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    const double omega = 2.0 * pi * freq[i];
    position[i] = amp[i] * std::sin(omega * eval_time);
    velocity[i] = amp[i] * omega * std::cos(omega * eval_time);
    acceleration[i] = -amp[i] * omega * omega * std::sin(omega * eval_time);
  }

  if (eval_time < smoothing_duration) {
    const double phase = SmoothPos(0.0, 1.0, smoothing_duration, eval_time);
    position *= phase;
    velocity *= phase;
    acceleration *= phase;
  }
}

HermiteCurve::HermiteCurve()
: start_pos_(0.0),
  start_vel_(0.0),
  end_pos_(0.0),
  end_vel_(0.0),
  duration_(0.5),
  s_(0.0)
{
}

HermiteCurve::HermiteCurve(
  double start_pos, double start_vel, double end_pos, double end_vel, double duration)
: start_pos_(start_pos),
  start_vel_(start_vel),
  end_pos_(end_pos),
  end_vel_(end_vel),
  duration_(std::max(duration, 1.0e-3)),
  s_(0.0)
{
}

double HermiteCurve::Evaluate(double time)
{
  s_ = Clamp(time / duration_);
  return start_pos_ * (2.0 * std::pow(s_, 3) - 3.0 * std::pow(s_, 2) + 1.0) +
         end_pos_ * (-2.0 * std::pow(s_, 3) + 3.0 * std::pow(s_, 2)) +
         start_vel_ * duration_ * (std::pow(s_, 3) - 2.0 * std::pow(s_, 2) + s_) +
         end_vel_ * duration_ * (std::pow(s_, 3) - std::pow(s_, 2));
}

double HermiteCurve::EvaluateFirstDerivative(double time)
{
  s_ = Clamp(time / duration_);
  return (start_pos_ * (6.0 * std::pow(s_, 2) - 6.0 * s_) +
          end_pos_ * (-6.0 * std::pow(s_, 2) + 6.0 * s_) +
          start_vel_ * duration_ * (3.0 * std::pow(s_, 2) - 4.0 * s_ + 1.0) +
          end_vel_ * duration_ * (3.0 * std::pow(s_, 2) - 2.0 * s_)) /
         duration_;
}

double HermiteCurve::EvaluateSecondDerivative(double time)
{
  s_ = Clamp(time / duration_);
  return (start_pos_ * (12.0 * s_ - 6.0) + end_pos_ * (-12.0 * s_ + 6.0) +
          start_vel_ * duration_ * (6.0 * s_ - 4.0) +
          end_vel_ * duration_ * (6.0 * s_ - 2.0)) /
         duration_ / duration_;
}

double HermiteCurve::Clamp(double value, double lower, double upper)
{
  return std::clamp(value, lower, upper);
}

HermiteCurveVec::HermiteCurveVec(
  const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
  const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, double duration)
{
  Initialize(start_pos, start_vel, end_pos, end_vel, duration);
}

void HermiteCurveVec::Initialize(
  const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
  const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, double duration)
{
  start_pos_ = start_pos;
  const int n = start_pos.size();
  if (static_cast<int>(curves_.size()) != n) {
    curves_.resize(n);
    output_ = Eigen::VectorXd::Zero(n);
  }
  for (int i = 0; i < n; ++i) {
    curves_[i] = HermiteCurve(start_pos[i], start_vel[i], end_pos[i], end_vel[i], duration);
  }
}

Eigen::VectorXd HermiteCurveVec::Evaluate(double time)
{
  for (int i = 0; i < start_pos_.size(); ++i) {
    output_[i] = curves_[i].Evaluate(time);
  }
  return output_;
}

Eigen::VectorXd HermiteCurveVec::EvaluateFirstDerivative(double time)
{
  for (int i = 0; i < start_pos_.size(); ++i) {
    output_[i] = curves_[i].EvaluateFirstDerivative(time);
  }
  return output_;
}

Eigen::VectorXd HermiteCurveVec::EvaluateSecondDerivative(double time)
{
  for (int i = 0; i < start_pos_.size(); ++i) {
    output_[i] = curves_[i].EvaluateSecondDerivative(time);
  }
  return output_;
}

HermiteQuaternionCurve::HermiteQuaternionCurve(
  const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
  const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end,
  double duration)
{
  Initialize(quat_start, angular_velocity_start, quat_end, angular_velocity_end, duration);
}

void HermiteQuaternionCurve::Initialize(
  const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
  const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end,
  double duration)
{
  quat_start_ = quat_start;
  angular_velocity_start_ = angular_velocity_start;
  quat_end_ = quat_end;
  angular_velocity_end_ = angular_velocity_end;
  duration_ = std::max(duration, 1.0e-3);
  InitializeDataStructures();
}

void HermiteQuaternionCurve::InitializeDataStructures()
{
  const Eigen::AngleAxisd delta_aa(quat_end_ * quat_start_.inverse());
  const Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd start_vel = angular_velocity_start_;
  const Eigen::VectorXd end_pos = delta_aa.axis() * delta_aa.angle();
  const Eigen::VectorXd end_vel = angular_velocity_end_;
  theta_ab_.Initialize(start_pos, start_vel, end_pos, end_vel, duration_);
}

void HermiteQuaternionCurve::Evaluate(double time, Eigen::Quaterniond & quat_out)
{
  const Eigen::VectorXd delta = theta_ab_.Evaluate(time);
  Eigen::Quaterniond delta_quat = Eigen::Quaterniond::Identity();
  if (delta.norm() >= 1.0e-6) {
    delta_quat = Eigen::AngleAxisd(delta.norm(), delta / delta.norm());
  }
  quat_out = delta_quat * quat_start_;
}

void HermiteQuaternionCurve::GetAngularVelocity(double time, Eigen::Vector3d & angular_velocity_out)
{
  angular_velocity_out = theta_ab_.EvaluateFirstDerivative(time);
}

void HermiteQuaternionCurve::GetAngularAcceleration(
  double time, Eigen::Vector3d & angular_acceleration_out)
{
  angular_acceleration_out = theta_ab_.EvaluateSecondDerivative(time);
}

NormalizedHermiteQuaternionCurve::NormalizedHermiteQuaternionCurve(
  const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
  const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end)
{
  Initialize(quat_start, angular_velocity_start, quat_end, angular_velocity_end);
}

void NormalizedHermiteQuaternionCurve::Initialize(
  const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
  const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end)
{
  quat_start_ = quat_start;
  angular_velocity_start_ = angular_velocity_start;
  quat_end_ = quat_end;
  angular_velocity_end_ = angular_velocity_end;
  phase_ = 0.0;
  InitializeDataStructures();
}

void NormalizedHermiteQuaternionCurve::SetDesired(
  const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end)
{
  quat_end_ = quat_end;
  angular_velocity_end_ = angular_velocity_end;
  InitializeDataStructures();
}

void NormalizedHermiteQuaternionCurve::SetInitial(
  const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start)
{
  quat_start_ = quat_start;
  angular_velocity_start_ = angular_velocity_start;
  phase_ = 0.0;
  InitializeDataStructures();
}

void NormalizedHermiteQuaternionCurve::InitializeDataStructures()
{
  q0_ = quat_start_;
  q1_ = angular_velocity_start_.norm() < 1.0e-6 ?
    quat_start_ :
    quat_start_ * Eigen::Quaterniond(Eigen::AngleAxisd(
                    angular_velocity_start_.norm() / 3.0,
                    angular_velocity_start_ / angular_velocity_start_.norm()));

  q2_ = angular_velocity_end_.norm() < 1.0e-6 ?
    quat_end_ :
    quat_end_ * Eigen::Quaterniond(Eigen::AngleAxisd(
                  angular_velocity_end_.norm() / 3.0,
                  -angular_velocity_end_ / angular_velocity_end_.norm()));
  q3_ = quat_end_;

  omega_1_aa_ = q1_ * q0_.inverse();
  omega_2_aa_ = q2_ * q1_.inverse();
  omega_3_aa_ = q3_ * q2_.inverse();

  omega_1_ = omega_1_aa_.axis() * omega_1_aa_.angle();
  omega_2_ = omega_2_aa_.axis() * omega_2_aa_.angle();
  omega_3_ = omega_3_aa_.axis() * omega_3_aa_.angle();
}

void NormalizedHermiteQuaternionCurve::ComputeBasis(double phase)
{
  phase_ = Clamp(phase);
  b1_ = 1.0 - std::pow(1.0 - phase_, 3);
  b2_ = 3.0 * std::pow(phase_, 2) - 2.0 * std::pow(phase_, 3);
  b3_ = std::pow(phase_, 3);

  bdot1_ = 3.0 * std::pow(1.0 - phase_, 2);
  bdot2_ = 6.0 * phase_ - 6.0 * std::pow(phase_, 2);
  bdot3_ = 3.0 * std::pow(phase_, 2);

  bddot1_ = -6.0 * (1.0 - phase_);
  bddot2_ = 6.0 - 12.0 * phase_;
  bddot3_ = 6.0 * phase_;
}

Eigen::Quaterniond NormalizedHermiteQuaternionCurve::GetOrientation(double phase)
{
  ComputeBasis(phase);
  const Eigen::Quaterniond qtmp1(Eigen::AngleAxisd(omega_1_aa_.angle() * b1_, omega_1_aa_.axis()));
  const Eigen::Quaterniond qtmp2(Eigen::AngleAxisd(omega_2_aa_.angle() * b2_, omega_2_aa_.axis()));
  const Eigen::Quaterniond qtmp3(Eigen::AngleAxisd(omega_3_aa_.angle() * b3_, omega_3_aa_.axis()));
  return qtmp3 * qtmp2 * qtmp1 * q0_;
}

Eigen::Vector3d NormalizedHermiteQuaternionCurve::GetAngularVelocity(double phase)
{
  ComputeBasis(phase);
  return omega_1_ * bdot1_ + omega_2_ * bdot2_ + omega_3_ * bdot3_;
}

Eigen::Vector3d NormalizedHermiteQuaternionCurve::GetAngularAcceleration(double phase)
{
  ComputeBasis(phase);
  return omega_1_ * bddot1_ + omega_2_ * bddot2_ + omega_3_ * bddot3_;
}

double NormalizedHermiteQuaternionCurve::Clamp(double value, double lower, double upper)
{
  return std::clamp(value, lower, upper);
}

MinJerkCurve::MinJerkCurve()
{
  Initialize();
}

MinJerkCurve::MinJerkCurve(
  const Eigen::Vector3d & start, const Eigen::Vector3d & finish, double start_time,
  double finish_time)
{
  Initialize();
  SetParams(start, finish, start_time, finish_time);
}

void MinJerkCurve::Initialize()
{
  for (double & coeff : b_) {
    coeff = 0.0;
  }
  inv_duration_ = 1.0;
  start_cond_.setZero();
  finish_cond_.setZero();
  start_time_ = 0.0;
  finish_time_ = 1.0;
}

void MinJerkCurve::SetParams(
  const Eigen::Vector3d & start, const Eigen::Vector3d & finish, double start_time,
  double finish_time)
{
  start_cond_ = start;
  finish_cond_ = finish;
  start_time_ = start_time;
  finish_time_ = finish_time;

  const double duration = finish_time_ - start_time_;
  inv_duration_ = duration > 1.0e-9 ? 1.0 / duration : 0.0;
  const double duration2 = duration * duration;

  b_[0] = start_cond_[0];
  b_[1] = start_cond_[1] * duration;
  b_[2] = start_cond_[2] * duration2 * 0.5;

  const double d0 = finish_cond_[0] - b_[0] - b_[1] - b_[2];
  const double d1 = finish_cond_[1] * duration - b_[1] - 2.0 * b_[2];
  const double d2 = finish_cond_[2] * duration2 - 2.0 * b_[2];

  b_[3] = 10.0 * d0 - 4.0 * d1 + 0.5 * d2;
  b_[4] = -15.0 * d0 + 7.0 * d1 - d2;
  b_[5] = 6.0 * d0 - 3.0 * d1 + 0.5 * d2;
}

void MinJerkCurve::GetPos(double time, double & position) const
{
  const double tau = std::clamp((time - start_time_) * inv_duration_, 0.0, 1.0);
  position =
    b_[0] + tau * (b_[1] + tau * (b_[2] + tau * (b_[3] + tau * (b_[4] + tau * b_[5]))));
}

void MinJerkCurve::GetVel(double time, double & velocity) const
{
  if (time >= finish_time_) {
    velocity = finish_cond_[1];
    return;
  }
  if (time <= start_time_) {
    velocity = start_cond_[1];
    return;
  }
  const double tau = (time - start_time_) * inv_duration_;
  velocity = (b_[1] + tau * (2.0 * b_[2] + tau * (3.0 * b_[3] +
              tau * (4.0 * b_[4] + tau * 5.0 * b_[5])))) * inv_duration_;
}

void MinJerkCurve::GetAcc(double time, double & acceleration) const
{
  if (time >= finish_time_) {
    acceleration = finish_cond_[2];
    return;
  }
  if (time <= start_time_) {
    acceleration = start_cond_[2];
    return;
  }
  const double tau = (time - start_time_) * inv_duration_;
  acceleration = (2.0 * b_[2] + tau * (6.0 * b_[3] +
                  tau * (12.0 * b_[4] + tau * 20.0 * b_[5]))) *
                 inv_duration_ * inv_duration_;
}

MinJerkCurveVec::MinJerkCurveVec(
  const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
  const Eigen::VectorXd & start_acc, const Eigen::VectorXd & end_pos,
  const Eigen::VectorXd & end_vel, const Eigen::VectorXd & end_acc, double duration)
{
  Initialize(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, duration);
}

void MinJerkCurveVec::Initialize(
  const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
  const Eigen::VectorXd & start_acc, const Eigen::VectorXd & end_pos,
  const Eigen::VectorXd & end_vel, const Eigen::VectorXd & end_acc, double duration)
{
  duration_ = duration;
  const int n = start_pos.size();
  curves_.resize(n);
  for (int i = 0; i < n; ++i) {
    curves_[i].SetParams(
      Eigen::Vector3d(start_pos[i], start_vel[i], start_acc[i]),
      Eigen::Vector3d(end_pos[i], end_vel[i], end_acc[i]), 0.0, duration_);
  }
  pos_out_.setZero(n);
  vel_out_.setZero(n);
  acc_out_.setZero(n);
  sample_out_.Resize(static_cast<unsigned int>(n));
}

const Eigen::VectorXd & MinJerkCurveVec::Evaluate(double time)
{
  for (int i = 0; i < static_cast<int>(curves_.size()); ++i) {
    curves_[i].GetPos(time, pos_out_[i]);
  }
  return pos_out_;
}

const Eigen::VectorXd & MinJerkCurveVec::EvaluateFirstDerivative(double time)
{
  for (int i = 0; i < static_cast<int>(curves_.size()); ++i) {
    curves_[i].GetVel(time, vel_out_[i]);
  }
  return vel_out_;
}

const Eigen::VectorXd & MinJerkCurveVec::EvaluateSecondDerivative(double time)
{
  for (int i = 0; i < static_cast<int>(curves_.size()); ++i) {
    curves_[i].GetAcc(time, acc_out_[i]);
  }
  return acc_out_;
}

const TrajectorySample & MinJerkCurveVec::EvaluateSample(double time)
{
  sample_out_.value = Evaluate(time);
  sample_out_.derivative = EvaluateFirstDerivative(time);
  sample_out_.second_derivative = EvaluateSecondDerivative(time);
  return sample_out_;
}

}  // namespace plato_robot_system::trajectory
