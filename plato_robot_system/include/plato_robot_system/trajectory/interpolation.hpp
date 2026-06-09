#ifndef PLATO_ROBOT_SYSTEM__TRAJECTORY__INTERPOLATION_HPP_
#define PLATO_ROBOT_SYSTEM__TRAJECTORY__INTERPOLATION_HPP_

#include <cassert>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

#include "plato_robot_system/trajectory/trajectory_base.hpp"

namespace plato_robot_system::trajectory
{

double Smooth(double start, double finish, double ratio);
double SmoothPos(double start, double finish, double duration, double time);
double SmoothVel(double start, double finish, double duration, double time);
double SmoothAcc(double start, double finish, double duration, double time);

void SinusoidTrajectory(
  const Eigen::VectorXd & amp, const Eigen::VectorXd & freq, double eval_time,
  Eigen::VectorXd & position, Eigen::VectorXd & velocity, Eigen::VectorXd & acceleration,
  double smoothing_duration = 1.0);

void SinusoidTrajectory(
  const Eigen::VectorXd & midpoint, const Eigen::VectorXd & amp, const Eigen::VectorXd & freq,
  double eval_time, Eigen::VectorXd & position, Eigen::VectorXd & velocity,
  Eigen::VectorXd & acceleration, double smoothing_duration = 1.0);

class HermiteCurve
{
public:
  HermiteCurve();
  HermiteCurve(
    double start_pos, double start_vel, double end_pos, double end_vel, double duration);

  double Evaluate(double time);
  double EvaluateFirstDerivative(double time);
  double EvaluateSecondDerivative(double time);

private:
  static double Clamp(double value, double lower = 0.0, double upper = 1.0);

  double start_pos_;
  double start_vel_;
  double end_pos_;
  double end_vel_;
  double duration_;
  double s_;
};

class HermiteCurveVec
{
public:
  HermiteCurveVec() = default;
  HermiteCurveVec(
    const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
    const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, double duration);

  void Initialize(
    const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
    const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, double duration);

  Eigen::VectorXd Evaluate(double time);
  Eigen::VectorXd EvaluateFirstDerivative(double time);
  Eigen::VectorXd EvaluateSecondDerivative(double time);

private:
  Eigen::VectorXd start_pos_;
  std::vector<HermiteCurve> curves_;
  Eigen::VectorXd output_;
};

class HermiteQuaternionCurve
{
public:
  HermiteQuaternionCurve() = default;
  HermiteQuaternionCurve(
    const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
    const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end,
    double duration);

  void Initialize(
    const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
    const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end,
    double duration);

  void Evaluate(double time, Eigen::Quaterniond & quat_out);
  void GetAngularVelocity(double time, Eigen::Vector3d & angular_velocity_out);
  void GetAngularAcceleration(double time, Eigen::Vector3d & angular_acceleration_out);

private:
  void InitializeDataStructures();

  double duration_{1.0};
  Eigen::Quaterniond quat_start_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_start_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond quat_end_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_end_{Eigen::Vector3d::Zero()};
  HermiteCurveVec theta_ab_;
};

class NormalizedHermiteQuaternionCurve
{
public:
  NormalizedHermiteQuaternionCurve() = default;
  NormalizedHermiteQuaternionCurve(
    const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
    const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end);

  void Initialize(
    const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
    const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end);

  void SetDesired(
    const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end);
  void SetInitial(
    const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start);

  Eigen::Quaterniond GetOrientation(double phase);
  Eigen::Vector3d GetAngularVelocity(double phase);
  Eigen::Vector3d GetAngularAcceleration(double phase);

  const Eigen::Quaterniond & GetInitialOrientation() const { return quat_start_; }
  const Eigen::Quaterniond & GetFinalOrientation() const { return quat_end_; }

private:
  static double Clamp(double value, double lower = 0.0, double upper = 1.0);
  void InitializeDataStructures();
  void ComputeBasis(double phase);

  Eigen::Quaterniond quat_start_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_start_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond quat_end_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_end_{Eigen::Vector3d::Zero()};

  Eigen::Quaterniond q0_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond q1_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond q2_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond q3_{Eigen::Quaterniond::Identity()};

  Eigen::Vector3d omega_1_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_2_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_3_{Eigen::Vector3d::Zero()};
  Eigen::AngleAxisd omega_1_aa_{Eigen::AngleAxisd::Identity()};
  Eigen::AngleAxisd omega_2_aa_{Eigen::AngleAxisd::Identity()};
  Eigen::AngleAxisd omega_3_aa_{Eigen::AngleAxisd::Identity()};

  double phase_{0.0};
  double b1_{0.0};
  double b2_{0.0};
  double b3_{0.0};
  double bdot1_{0.0};
  double bdot2_{0.0};
  double bdot3_{0.0};
  double bddot1_{0.0};
  double bddot2_{0.0};
  double bddot3_{0.0};
};

class MinJerkCurve
{
public:
  MinJerkCurve();
  MinJerkCurve(
    const Eigen::Vector3d & start, const Eigen::Vector3d & finish, double start_time,
    double finish_time);

  void SetParams(
    const Eigen::Vector3d & start, const Eigen::Vector3d & finish, double start_time,
    double finish_time);

  void GetPos(double time, double & position) const;
  void GetVel(double time, double & velocity) const;
  void GetAcc(double time, double & acceleration) const;

private:
  void Initialize();

  double b_[6]{};
  double inv_duration_{1.0};
  Eigen::Vector3d start_cond_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d finish_cond_{Eigen::Vector3d::Zero()};
  double start_time_{0.0};
  double finish_time_{1.0};
};

class MinJerkCurveVec
{
public:
  MinJerkCurveVec() = default;
  MinJerkCurveVec(
    const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
    const Eigen::VectorXd & start_acc, const Eigen::VectorXd & end_pos,
    const Eigen::VectorXd & end_vel, const Eigen::VectorXd & end_acc, double duration);

  void Initialize(
    const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel,
    const Eigen::VectorXd & start_acc, const Eigen::VectorXd & end_pos,
    const Eigen::VectorXd & end_vel, const Eigen::VectorXd & end_acc, double duration);

  const Eigen::VectorXd & Evaluate(double time);
  const Eigen::VectorXd & EvaluateFirstDerivative(double time);
  const Eigen::VectorXd & EvaluateSecondDerivative(double time);
  const TrajectorySample & EvaluateSample(double time);

private:
  double duration_{1.0};
  std::vector<MinJerkCurve> curves_;
  Eigen::VectorXd pos_out_;
  Eigen::VectorXd vel_out_;
  Eigen::VectorXd acc_out_;
  TrajectorySample sample_out_;
};

template<typename Y, typename X>
Y Lerp(Y start, Y finish, X phase)
{
  static_assert(std::is_floating_point<X>::value, "phase must be floating point");
  assert(phase >= 0 && phase <= 1);
  return start + (finish - start) * phase;
}

template<typename Y, typename X>
Y CubicBezier(Y start, Y finish, X phase)
{
  static_assert(std::is_floating_point<X>::value, "phase must be floating point");
  assert(phase >= 0 && phase <= 1);
  const Y diff = finish - start;
  const X bezier = phase * phase * phase + X(3) * (phase * phase * (X(1) - phase));
  return start + bezier * diff;
}

template<typename Y, typename X>
Y CubicBezierFirstDerivative(Y start, Y finish, X phase)
{
  static_assert(std::is_floating_point<X>::value, "phase must be floating point");
  assert(phase >= 0 && phase <= 1);
  const Y diff = finish - start;
  const X bezier = X(6) * phase * (X(1) - phase);
  return bezier * diff;
}

template<typename Y, typename X>
Y CubicBezierSecondDerivative(Y start, Y finish, X phase)
{
  static_assert(std::is_floating_point<X>::value, "phase must be floating point");
  assert(phase >= 0 && phase <= 1);
  const Y diff = finish - start;
  const X bezier = X(6) - X(12) * phase;
  return bezier * diff;
}

}  // namespace plato_robot_system::trajectory

#endif  // PLATO_ROBOT_SYSTEM__TRAJECTORY__INTERPOLATION_HPP_
