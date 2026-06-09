#include "plato_robot_system/trajectory/euclidean_trajectory.hpp"

namespace plato_robot_system::trajectory
{

EuclideanConstantTrajectory::EuclideanConstantTrajectory(const std::string & name)
: TrajectoryBase(name)
{
}

EuclideanConstantTrajectory::EuclideanConstantTrajectory(
  const std::string & name, const Eigen::Ref<const Eigen::VectorXd> ref)
: TrajectoryBase(name)
{
  SetReference(ref);
}

unsigned int EuclideanConstantTrajectory::Size() const
{
  return static_cast<unsigned int>(sample_.value.size());
}

void EuclideanConstantTrajectory::SetReference(const Eigen::Ref<const Eigen::VectorXd> ref)
{
  sample_.Resize(static_cast<unsigned int>(ref.size()));
  sample_.value = ref;
}

const TrajectorySample & EuclideanConstantTrajectory::Evaluate(double /*time*/)
{
  return sample_;
}

const TrajectorySample & EuclideanConstantTrajectory::ComputeNext()
{
  return sample_;
}

void EuclideanConstantTrajectory::GetLastSample(TrajectorySample & sample) const
{
  sample = sample_;
}

bool EuclideanConstantTrajectory::HasTrajectoryEnded() const
{
  return true;
}

}  // namespace plato_robot_system::trajectory
