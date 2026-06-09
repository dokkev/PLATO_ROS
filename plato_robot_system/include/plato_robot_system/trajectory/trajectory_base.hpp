#ifndef PLATO_ROBOT_SYSTEM__TRAJECTORY__TRAJECTORY_BASE_HPP_
#define PLATO_ROBOT_SYSTEM__TRAJECTORY__TRAJECTORY_BASE_HPP_

#include <string>

#include <Eigen/Dense>

namespace plato_robot_system::trajectory
{

struct TrajectorySample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd value;
  Eigen::VectorXd derivative;
  Eigen::VectorXd second_derivative;

  explicit TrajectorySample(unsigned int size = 0) { Resize(size); }

  TrajectorySample(unsigned int value_size, unsigned int derivative_size)
  {
    Resize(value_size, derivative_size);
  }

  void Resize(unsigned int size) { Resize(size, size); }

  void Resize(unsigned int value_size, unsigned int derivative_size)
  {
    value.setZero(value_size);
    derivative.setZero(derivative_size);
    second_derivative.setZero(derivative_size);
  }
};

class TrajectoryBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TrajectoryBase(std::string name);
  virtual ~TrajectoryBase() = default;

  virtual unsigned int Size() const = 0;
  virtual const TrajectorySample & Evaluate(double time) = 0;
  virtual const TrajectorySample & ComputeNext() = 0;
  virtual void GetLastSample(TrajectorySample & sample) const = 0;
  virtual bool HasTrajectoryEnded() const = 0;

  const std::string & Name() const { return name_; }
  const TrajectorySample & LastSample() const { return sample_; }

protected:
  std::string name_;
  TrajectorySample sample_;
};

}  // namespace plato_robot_system::trajectory

#endif  // PLATO_ROBOT_SYSTEM__TRAJECTORY__TRAJECTORY_BASE_HPP_
