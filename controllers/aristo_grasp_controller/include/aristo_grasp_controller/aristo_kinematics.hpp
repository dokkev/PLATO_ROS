#ifndef ARISTO_GRASP_CONTROLLER__ARISTO_KINEMATICS_HPP_
#define ARISTO_GRASP_CONTROLLER__ARISTO_KINEMATICS_HPP_

#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <vector>

#include "plato_robot_system/robot/robot_system.hpp"

namespace aristo_grasp_controller
{

class AristoKinematics
{
public:
  explicit AristoKinematics(std::string urdf_path);

  int nq() const { return robot_.nq(); }
  int nv() const { return robot_.nv(); }
  const plato_robot_system::RobotSystem & robot() const { return robot_; }

  Eigen::VectorXd make_configuration(const std::vector<double> & joint_positions) const;
  void update(const Eigen::Ref<const Eigen::VectorXd> & q);

  pinocchio::SE3 frame_pose(const std::string & frame_name) const;
  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian(const std::string & frame_name);
  double frame_distance(const std::string & first_frame, const std::string & second_frame) const;
  double frame_axis_alignment(
    const std::string & first_frame,
    const std::string & second_frame,
    int axis_index) const;

private:
  plato_robot_system::RobotSystem robot_;
};

}  // namespace aristo_grasp_controller

#endif  // ARISTO_GRASP_CONTROLLER__ARISTO_KINEMATICS_HPP_
