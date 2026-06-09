#ifndef PLATO_ROBOT_SYSTEM__TRAJECTORY__JOINT_INTEGRATOR_HPP_
#define PLATO_ROBOT_SYSTEM__TRAJECTORY__JOINT_INTEGRATOR_HPP_

#include <Eigen/Dense>

namespace plato_robot_system::trajectory
{

class JointIntegrator
{
public:
  JointIntegrator(
    int num_joints, double dt, const Eigen::VectorXd & pos_min, const Eigen::VectorXd & pos_max,
    const Eigen::VectorXd & vel_min, const Eigen::VectorXd & vel_max);

  void SetCutoffFrequency(double pos_cutoff_freq, double vel_cutoff_freq);
  void SetMaxPositionError(double pos_max_error);
  void Initialize(const Eigen::VectorXd & q_init, const Eigen::VectorXd & qdot_init);

  void Integrate(
    const Eigen::VectorXd & qddot_cmd, const Eigen::VectorXd & q,
    const Eigen::VectorXd & qdot, Eigen::VectorXd & q_cmd, Eigen::VectorXd & qdot_cmd);

  bool IsInitialized() const { return is_initialized_; }

private:
  static double AlphaFromFrequency(double hz, double dt);
  static Eigen::VectorXd Clamp(
    const Eigen::VectorXd & value, const Eigen::VectorXd & lower, const Eigen::VectorXd & upper);

  int num_joints_;
  double dt_;
  Eigen::VectorXd pos_min_;
  Eigen::VectorXd pos_max_;
  Eigen::VectorXd vel_min_;
  Eigen::VectorXd vel_max_;
  double alpha_pos_;
  double alpha_vel_;
  Eigen::VectorXd pos_max_error_;
  Eigen::VectorXd q_cmd_;
  Eigen::VectorXd qdot_cmd_;
  bool is_initialized_;
};

}  // namespace plato_robot_system::trajectory

#endif  // PLATO_ROBOT_SYSTEM__TRAJECTORY__JOINT_INTEGRATOR_HPP_
