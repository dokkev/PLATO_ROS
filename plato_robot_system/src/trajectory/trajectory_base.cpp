#include "plato_robot_system/trajectory/trajectory_base.hpp"

#include <utility>

namespace plato_robot_system::trajectory
{

TrajectoryBase::TrajectoryBase(std::string name)
: name_(std::move(name))
{
}

}  // namespace plato_robot_system::trajectory
