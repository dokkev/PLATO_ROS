#ifndef PLATO_ROBOT_SYSTEM__TRAJECTORY__EUCLIDEAN_TRAJECTORY_HPP_
#define PLATO_ROBOT_SYSTEM__TRAJECTORY__EUCLIDEAN_TRAJECTORY_HPP_

#include <string>

#include <Eigen/Dense>

#include "plato_robot_system/trajectory/trajectory_base.hpp"

namespace plato_robot_system::trajectory
{

class EuclideanConstantTrajectory : public TrajectoryBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EuclideanConstantTrajectory(const std::string & name);
  EuclideanConstantTrajectory(const std::string & name, const Eigen::Ref<const Eigen::VectorXd> ref);

  unsigned int Size() const override;
  void SetReference(const Eigen::Ref<const Eigen::VectorXd> ref);

  const TrajectorySample & Evaluate(double time) override;
  const TrajectorySample & ComputeNext() override;
  void GetLastSample(TrajectorySample & sample) const override;
  bool HasTrajectoryEnded() const override;
};

}  // namespace plato_robot_system::trajectory

#endif  // PLATO_ROBOT_SYSTEM__TRAJECTORY__EUCLIDEAN_TRAJECTORY_HPP_
