#ifndef PLATO_GRASP_CONTROLLER__PLATO_GRASP_PLANNER_HPP_
#define PLATO_GRASP_CONTROLLER__PLATO_GRASP_PLANNER_HPP_

#include <optional>
#include <string>
#include <vector>

namespace plato_grasp_controller
{

struct GraspPlanConfig
{
  std::vector<double> grasp_force_effort_ff;
};

struct GraspTaskConfig
{
  std::string pos_preset_name;
  bool use_current_position = false;
  double impedance_level = 0.0;
  double wait_sec = 0.0;
  GraspPlanConfig grasp_plan;
  double grasp_duration_sec = 0.0;
};

struct PlatoGraspPlannedCommand
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort_ff;
};

class PlatoGraspPlanner
{
public:
  enum class Phase
  {
    Idle,
    Motion,
    Grasp
  };

  PlatoGraspPlanner(
    int joint_count,
    std::string saved_joint_positions_yaml_path);

  void update_joint_state(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions);

  bool save_current_joint_position(
    const std::string & requested_name,
    std::string * saved_name_out,
    std::string * error_out);

  bool make_motion_command(
    const std::string & pos_preset_name,
    PlatoGraspPlannedCommand * command_out,
    std::string * error_out);

  bool make_current_motion_command(
    PlatoGraspPlannedCommand * command_out,
    std::string * error_out);

  bool make_grasp_command(
    const GraspPlanConfig & grasp_plan,
    PlatoGraspPlannedCommand * command_out,
    std::string * error_out);

  bool has_joint_state() const;
  Phase phase() const;
  const std::string & active_pos_preset_name() const;
  const std::string & saved_joint_positions_yaml_path() const;

private:
  int joint_count_;
  std::string saved_joint_positions_yaml_path_;
  std::vector<double> last_positions_;
  std::vector<double> active_target_positions_;
  bool has_joint_state_ = false;
  bool has_active_target_ = false;
  Phase phase_ = Phase::Idle;
  std::string active_pos_preset_name_;
};

}  // namespace plato_grasp_controller

#endif  // PLATO_GRASP_CONTROLLER__PLATO_GRASP_PLANNER_HPP_
