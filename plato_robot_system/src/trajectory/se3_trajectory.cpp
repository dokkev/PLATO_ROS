#include "plato_robot_system/trajectory/se3_trajectory.hpp"

#include <stdexcept>

namespace plato_robot_system::trajectory
{

SE3ConstantTrajectory::SE3ConstantTrajectory(const std::string & name)
: TrajectoryBase(name)
{
  sample_.Resize(12, 6);
}

SE3ConstantTrajectory::SE3ConstantTrajectory(const std::string & name, const pinocchio::SE3 & ref)
: TrajectoryBase(name)
{
  sample_.Resize(12, 6);
  SE3ToVector(ref, sample_.value);
}

unsigned int SE3ConstantTrajectory::Size() const
{
  return 6;
}

void SE3ConstantTrajectory::SetReference(const pinocchio::SE3 & ref)
{
  sample_.Resize(12, 6);
  SE3ToVector(ref, sample_.value);
}

const TrajectorySample & SE3ConstantTrajectory::Evaluate(double /*time*/)
{
  return sample_;
}

const TrajectorySample & SE3ConstantTrajectory::ComputeNext()
{
  return sample_;
}

void SE3ConstantTrajectory::GetLastSample(TrajectorySample & sample) const
{
  sample = sample_;
}

bool SE3ConstantTrajectory::HasTrajectoryEnded() const
{
  return true;
}

void SE3ConstantTrajectory::SE3ToVector(
  const pinocchio::SE3 & ref, Eigen::Ref<Eigen::VectorXd> vector)
{
  if (vector.size() != 12) {
    throw std::invalid_argument("SE3 trajectory vector must have size 12");
  }

  vector.head<3>() = ref.translation();
  using Vector9 = Eigen::Matrix<double, 9, 1>;
  vector.tail<9>() = Eigen::Map<const Vector9>(&ref.rotation()(0), 9);
}

}  // namespace plato_robot_system::trajectory
