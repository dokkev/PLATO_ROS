#ifndef PLATO_ROBOT_SYSTEM__TRAJECTORY__SE3_TRAJECTORY_HPP_
#define PLATO_ROBOT_SYSTEM__TRAJECTORY__SE3_TRAJECTORY_HPP_

#include <string>

#include <pinocchio/spatial/se3.hpp>

#include "plato_robot_system/trajectory/trajectory_base.hpp"

namespace plato_robot_system::trajectory
{

class SE3ConstantTrajectory : public TrajectoryBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SE3ConstantTrajectory(const std::string & name);
  SE3ConstantTrajectory(const std::string & name, const pinocchio::SE3 & ref);

  unsigned int Size() const override;
  void SetReference(const pinocchio::SE3 & ref);

  const TrajectorySample & Evaluate(double time) override;
  const TrajectorySample & ComputeNext() override;
  void GetLastSample(TrajectorySample & sample) const override;
  bool HasTrajectoryEnded() const override;

private:
  static void SE3ToVector(const pinocchio::SE3 & ref, Eigen::Ref<Eigen::VectorXd> vector);
};

}  // namespace plato_robot_system::trajectory

#endif  // PLATO_ROBOT_SYSTEM__TRAJECTORY__SE3_TRAJECTORY_HPP_
