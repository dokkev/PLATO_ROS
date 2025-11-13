/**
 * @file grasp_node.cpp
 * @brief Direct replication of NaritouchGraspDemo using ParallelGraspController
 *
 * This node replicates the exact behavior of NaritouchGraspDemo but uses
 * ParallelGraspController for computing impedance commands instead of
 * direct position commands.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "plato2_interfaces/msg/impedance_commands.hpp"
#include "sdr_grasp_msgs/msg/tactile.hpp"
#include "sdr_grasp_msgs/msg/grasp_control.hpp"
#include "sdr_grasp_msgs/msg/grasp_request.hpp"
#include "grasp_srvs/srv/start_adaptive_grasp_ctrl.hpp"
#include "grasp_srvs/srv/start_constant_grasp_ctrl.hpp"
#include "grasp_srvs/srv/start_load_ctrl.hpp"
#include "grasp_srvs/srv/stop_grasp_ctrl.hpp"
#include "grasp_srvs/srv/start_adaptive_release_ctrl.hpp"
#include "grasp_srvs/srv/stop_release_ctrl.hpp"
#include "grasp_srvs/srv/tactile_zero_reset.hpp"
#include "std_srvs/srv/empty.hpp"
#include "joint_position_controller/parallel_grasp_controller.hpp"
#include "joint_position_controller/pid.hpp"

#define ROS_HZ (100)  // Hz

using GraspControl = sdr_grasp_msgs::msg::GraspControl;

class ParallelGraspNode : public rclcpp::Node {
public:
  ParallelGraspNode()
    : Node("parallel_grasp_node")
  {
    is_exit_process_ = false;
    GetParameters();
    grasp_info_.state = GraspControl::STANDBY;
    grasp_info_.grasp_force_control.reference_force = 0.0;
    init_force_ = kInitForce;
    cnt_ = 0;

    StartROS();

    RCLCPP_INFO(this->get_logger(), "Waiting for grasp start...");
  }

  ~ParallelGraspNode() {
    ShutdownROS();
  }

  void ExitState() {
    is_exit_process_ = true;
  }

  bool IsBusy() {
    return is_busy_;
  }

private:
  // Constants
  static constexpr double kInitForce = 0.0;
  static constexpr double kInitForceDelta = 0.05;
  static constexpr double kStableTimeSec = 0.5;
  static constexpr double kMinPosRefMM = 0.5;
  static constexpr double kForceToPosGain = 0.12;
  static constexpr double kContactDetTime = 0.01;
  static constexpr double kNoContactDetTime = 0.5;
  static constexpr double kContactEnoughTime = 0.5;
  // U regulator gains (force tracking): simple PI on force error (desired - measured)
  static constexpr double kU_KP = 0.02;   // proportional gain for u feedback
  static constexpr double kU_KI = 0.005;  // integral gain for u feedback
  static constexpr double kU_I_LIMIT = 0.5; // anti-windup limit for integral term
  static constexpr int TERMINATE = -1;

  // State machine
  enum class State {
    kIdle,
    kCloseGripper,
    kCtrlGraspForce,
    kCtrlReleaseForce,
    kOpenGripper,
    kTerminated,
    kApplyForce,
  };

  enum class ContactStatus {
    kNoContact = 0,
    kFewContact,
    kEnoughContact,
  };

  enum class GraspForceState {
    kStandby  = 0,
    kLoadCtrl,
    kAdaptiveGraspCtrl,
    kConstantGraspCtrl,
    kAdaptiveReleaseCtrl,
    kUnloadCtrl,
    kFinished,
  };

  void GetParameters() {
    // Add any parameters if needed
  }

  void StartROS() {
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // Subscribers
    tac_stat_fin0_sub_ = this->create_subscription<sdr_grasp_msgs::msg::Tactile>(
      "tactile_0/tactile_states", qos,
      std::bind(&ParallelGraspNode::SubTacStatusCallback0, this, std::placeholders::_1));

    tac_stat_fin1_sub_ = this->create_subscription<sdr_grasp_msgs::msg::Tactile>(
      "tactile_1/tactile_states", qos,
      std::bind(&ParallelGraspNode::SubTacStatusCallback1, this, std::placeholders::_1));

    des_force_fin0_sub_ = this->create_subscription<sdr_grasp_msgs::msg::GraspControl>(
      "grasp_force_0/des_grasp_force", qos,
      std::bind(&ParallelGraspNode::SubDesForceCallback0, this, std::placeholders::_1));

    des_force_fin1_sub_ = this->create_subscription<sdr_grasp_msgs::msg::GraspControl>(
      "grasp_force_1/des_grasp_force", qos,
      std::bind(&ParallelGraspNode::SubDesForceCallback1, this, std::placeholders::_1));

    grasp_req_sub_ = this->create_subscription<sdr_grasp_msgs::msg::GraspRequest>(
      "parallel_grasp_node/grasp_req", qos,
      std::bind(&ParallelGraspNode::GraspReqCallback, this, std::placeholders::_1));

    des_jstate_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "plato2/joint_states", qos,
      std::bind(&ParallelGraspNode::SubJointStates, this, std::placeholders::_1));

    u_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "parallel_grasp_node/u_cmd", qos,
      std::bind(&ParallelGraspNode::UCommandCallback, this, std::placeholders::_1));

    // Publishers
    grasp_info_pub_ = this->create_publisher<sdr_grasp_msgs::msg::GraspControl>(
      "parallel_grasp_node/grasp_info", 1);

    trajectory_pub_ = this->create_publisher<std_msgs::msg::String>(
      "plato2/trajectory_execute", 1);

    impedance_cmd_pub_ = this->create_publisher<plato2_interfaces::msg::ImpedanceCommands>(
      "plato2/joint_impedance_controller/commands", qos);

    // Service clients
    zero_reset_start_ = this->create_client<grasp_srvs::srv::TactileZeroReset>(
      "tactile/zero_reset");

    adaptive_grasp_start_ = this->create_client<grasp_srvs::srv::StartAdaptiveGraspCtrl>(
      "grasp_force_ctrl/start_adaptive_grasp");
    constant_grasp_start_ = this->create_client<grasp_srvs::srv::StartConstantGraspCtrl>(
      "grasp_force_ctrl/start_constant_grasp");
    load_start_ = this->create_client<grasp_srvs::srv::StartLoadCtrl>(
      "grasp_force_ctrl/start_load");
    grasp_stop_ = this->create_client<grasp_srvs::srv::StopGraspCtrl>(
      "grasp_force_ctrl/stop_grasp");
    adaptive_release_start_ = this->create_client<grasp_srvs::srv::StartAdaptiveReleaseCtrl>(
      "grasp_force_ctrl/start_adaptive_release");
    release_stop_ = this->create_client<grasp_srvs::srv::StopReleaseCtrl>(
      "grasp_force_ctrl/stop_release");

    // Service servers
    start_grasp_server_ = this->create_service<std_srvs::srv::Empty>(
      "parallel_grasp_node/start_grasp",
      std::bind(&ParallelGraspNode::StartGrasp, this, std::placeholders::_1, std::placeholders::_2));

    start_release_server_ = this->create_service<std_srvs::srv::Empty>(
      "parallel_grasp_node/start_release",
      std::bind(&ParallelGraspNode::StartRelease, this, std::placeholders::_1, std::placeholders::_2));

    // Timer for control loop
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / ROS_HZ),
      std::bind(&ParallelGraspNode::TickCB, this));
  }

  void ShutdownROS() {
    tac_stat_fin0_sub_.reset();
    tac_stat_fin1_sub_.reset();
    des_force_fin0_sub_.reset();
    des_force_fin1_sub_.reset();
    grasp_req_sub_.reset();
    des_jstate_sub_.reset();
    u_cmd_sub_.reset();
    grasp_info_pub_.reset();
    trajectory_pub_.reset();
    impedance_cmd_pub_.reset();
    zero_reset_start_.reset();
    adaptive_grasp_start_.reset();
    constant_grasp_start_.reset();
    load_start_.reset();
    grasp_stop_.reset();
    adaptive_release_start_.reset();
    release_stop_.reset();
    start_grasp_server_.reset();
    start_release_server_.reset();
    timer_.reset();
  }

  void TickCB() {
    time_sec += 1.0 / ROS_HZ;
    if (is_exit_process_) return;
    if (is_busy_) return;
    is_busy_ = true;

    if (!grasp_requests_.empty()) {
      sdr_grasp_msgs::msg::GraspRequest grasp_req = grasp_requests_[0];
      grasp_requests_.erase(grasp_requests_.begin());
      GraspReqExec(grasp_req);
    }

    // State machine
    TickState();

    // Publish grasp_info
    grasp_info_.grasp_force_control.reference_force = force_;
    int topic_state = GraspControl::STANDBY;
    switch (state_) {
      case State::kCloseGripper:
        topic_state = GraspControl::PREGRASP;
        break;
      case State::kApplyForce:
        topic_state = GraspControl::LOAD;
        break;
      case State::kCtrlGraspForce:
        topic_state = GraspControl::GRASP_STABLE;
        break;
      case State::kCtrlReleaseForce:
        topic_state = GraspControl::RELEASE_WAIT;
        break;
      case State::kTerminated:
        topic_state = GraspControl::POSTGRASP;
        break;
      default:
        topic_state = GraspControl::STANDBY;
    }
    grasp_info_.state = topic_state;
    if (grasp_info_pub_ != NULL) {
      grasp_info_pub_->publish(grasp_info_);
    }

    // Send impedance commands using ParallelGraspController + PI effort control
    if (!sorted_positions_.empty() && sorted_positions_.size() >= 8) {
      static bool first_pub = true;
      if (first_pub) {
        RCLCPP_INFO(this->get_logger(), "Publishing impedance commands...");
        first_pub = false;
      }
      // Get position commands from controller
      auto pos_cmd = controller_.get_commands(u_cmd_, sorted_positions_);

      // Create impedance command message
      plato2_interfaces::msg::ImpedanceCommands impedance_cmd;
      impedance_cmd.position.resize(8);
      impedance_cmd.velocity.resize(8, 0.0);
      impedance_cmd.stiffness.resize(8, 4.0);  // Default stiffness
      impedance_cmd.damping.resize(8, 0.1);    // Default damping
      impedance_cmd.effort_ff.resize(8, 0.0);

      // Copy positions
      for (size_t i = 0; i < 8; ++i) {
        impedance_cmd.position[i] = pos_cmd[i];
      }

      // Compute PI effort feedforward for grasp joints (2,3,4,5)
      static constexpr double dt = 1.0 / ROS_HZ;
      impedance_cmd.effort_ff[2] = pi_joint2_(pos_cmd[2] - sorted_positions_[2], dt);
      impedance_cmd.effort_ff[3] = pi_joint3_(pos_cmd[3] - sorted_positions_[3], dt);
      impedance_cmd.effort_ff[4] = pi_joint4_(pos_cmd[4] - sorted_positions_[4], dt);
      impedance_cmd.effort_ff[5] = pi_joint5_(pos_cmd[5] - sorted_positions_[5], dt);

      impedance_cmd_pub_->publish(impedance_cmd);
    }

    is_busy_ = false;
  }

  void TickState() {
    if (tick_is_exec_) return;
    tick_is_exec_ = true;

    switch (state_) {
      /* 0: Idle */
      case State::kIdle: {
        vel_ref_ = 0;
        grasp_force_0_ = 0;
        grasp_force_1_ = 0;
        force_ = 0;
        prev_force_ = 0;
        applied_force = 0;

        // Ensure gripper is in a mostly-open standby position (u in [0,1],
        // where 1.0 is fully open). Use 0.9 so we don't fully open.
        u_cmd_ = 0.9;

        if (start_close_) {
          start_close_ = false;
          time_sec = 0;
          OpenCloseGripper();
          state_ = State::kCloseGripper;
          RCLCPP_INFO(this->get_logger(), "Change state to CloseGripper");
        }
        break;
      }

      /* 1: Close */
      case State::kCloseGripper: {
        // Gradually close gripper until contact is detected
        // Close at ~0.1/sec (will take ~3 seconds to fully close from 0.3)
        const double CLOSE_RATE = 0.1 / ROS_HZ;  // per control loop iteration
        u_cmd_ = std::max(0.0, u_cmd_ - CLOSE_RATE);

        if ((contact_state_0 >= ContactStatus::kFewContact) &&
            (contact_state_1 >= ContactStatus::kFewContact)) {
          contact_cnt_++;
          if (contact_cnt_ > kContactDetTime * ROS_HZ) {
            CallStartLoad();
            contact_cnt_ = 0;
            state_ = State::kApplyForce;
            SaveContactPosition();
            RCLCPP_INFO(this->get_logger(), "Change state to ApplyForce");
          }
        }
        break;
      }

      /* 2: Apply force */
      case State::kApplyForce: {
        force_ = std::max(grasp_force_0_, grasp_force_1_);
        RCLCPP_INFO(this->get_logger(), "Desired force: %.2f N (force_0: %.2f, force_1: %.2f)", 
                    force_, grasp_force_0_, grasp_force_1_);
        SaveContactPosition();
        ApplyForce(force_);
        prev_force_ = force_;

        if ((contact_state_0 == ContactStatus::kEnoughContact) &&
            (contact_state_1 == ContactStatus::kEnoughContact)) {
          area_enough_cnt_++;
          if (area_enough_cnt_ > (int)(kContactEnoughTime * ROS_HZ)) {
            if (control_mode_ == sdr_grasp_msgs::msg::GraspRequest::ADAPTIVE_FORCE) {
              CallStartAdaptiveGrasp();
            } else if (control_mode_ == sdr_grasp_msgs::msg::GraspRequest::FIXED_FORCE) {
              CallStartConstantGrasp();
            }
            area_enough_cnt_ = 0;
            time_sec = 0;
            state_ = State::kCtrlGraspForce;
            RCLCPP_INFO(this->get_logger(), "Change state to CtrlGraspForce");
          }
        }
        break;
      }

      /* 3: Ctrl Grasp Force */
      case State::kCtrlGraspForce: {
        force_ = std::max(grasp_force_0_, grasp_force_1_);
        RCLCPP_INFO(this->get_logger(), "Controlling grasp force: %.2f N (force_0: %.2f, force_1: %.2f)", 
                    force_, grasp_force_0_, grasp_force_1_);
        ApplyForce(force_);
        prev_force_ = force_;

        if ((contact_state_0 == ContactStatus::kNoContact) ||
            (contact_state_1 == ContactStatus::kNoContact)) {
          no_contact_cnt_++;
          if (no_contact_cnt_ > (int)(kNoContactDetTime * ROS_HZ)) {
            no_contact_cnt_ = 0;
            CallStopGrasp();
            Terminate();
          }
        } else if (start_release_) {
          start_release_ = false;
          state_ = State::kCtrlReleaseForce;
          CallStartAdaptiveRelease();
          RCLCPP_INFO(this->get_logger(), "Change state to CtrlReleaseForce.");
        } else {
          no_contact_cnt_ = 0;
        }
        break;
      }

      /* 4: Ctrl Release Force */
      case State::kCtrlReleaseForce: {
        force_ = std::max(grasp_force_0_, grasp_force_1_);
        RCLCPP_INFO(this->get_logger(), "Controlling release force: %.2f N (force_0: %.2f, force_1: %.2f)", 
                    force_, grasp_force_0_, grasp_force_1_);
        ApplyForce(force_);
        prev_force_ = force_;

        bool release_control_finished = (grasp_state_0_ == GraspForceState::kUnloadCtrl) ||
                                        (grasp_state_1_ == GraspForceState::kUnloadCtrl);
        bool no_contact = (contact_state_0 == ContactStatus::kNoContact) ||
                         (contact_state_1 == ContactStatus::kNoContact);
        if (release_control_finished || no_contact) {
          CallStopRelease();
          Terminate();
        }
        break;
      }

      /* 5: Open */
      case State::kOpenGripper: {
        if (time_sec > 0.05) {
          OpenGripper();
          CallZeroReset();
          state_ = State::kIdle;
          RCLCPP_INFO(this->get_logger(), "Change state to Idle");
        }
        break;
      }

      case State::kTerminated: {
        if (time_sec > 0.5) {
          state_ = State::kOpenGripper;
          RCLCPP_INFO(this->get_logger(), "Change state to OpenGripper");
          time_sec = 0;
        }
        break;
      }

      default:
        break;
    }
    tick_is_exec_ = false;
  }

  // Callback functions
  void SubTacStatusCallback0(const sdr_grasp_msgs::msg::Tactile& tac) {
    tactile_0_ = tac;
    contact_state_0 = static_cast<ContactStatus>(tac.contact_state);
  }

  void SubTacStatusCallback1(const sdr_grasp_msgs::msg::Tactile& tac) {
    tactile_1_ = tac;
    contact_state_1 = static_cast<ContactStatus>(tac.contact_state);
  }

  void SubDesForceCallback0(const sdr_grasp_msgs::msg::GraspControl& force) {
    grasp_state_0_ = static_cast<GraspForceState>(force.grasp_force_control.state);
    grasp_force_0_ = force.grasp_force_control.reference_force;
  }

  void SubDesForceCallback1(const sdr_grasp_msgs::msg::GraspControl& force) {
    grasp_state_1_ = static_cast<GraspForceState>(force.grasp_force_control.state);
    grasp_force_1_ = force.grasp_force_control.reference_force;
  }

  void SubJointStates(const sensor_msgs::msg::JointState& jstate) {
    jstate_ = jstate;

    // Reorder joint states to [joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8]
    // Incoming order: [joint2, joint3, joint6, joint8, joint1, joint4, joint7, joint5]
    if (jstate.position.size() >= 8) {
      sorted_positions_.resize(8);
      // Map: incoming[i] -> sorted[j]
      sorted_positions_[0] = jstate.position[4];  // joint1
      sorted_positions_[1] = jstate.position[0];  // joint2
      sorted_positions_[2] = jstate.position[1];  // joint3
      sorted_positions_[3] = jstate.position[5];  // joint4
      sorted_positions_[4] = jstate.position[7];  // joint5
      sorted_positions_[5] = jstate.position[2];  // joint6
      sorted_positions_[6] = jstate.position[6];  // joint7
      sorted_positions_[7] = jstate.position[3];  // joint8
    }

    static bool first_msg = true;
    if (first_msg) {
      RCLCPP_INFO(this->get_logger(), "Joint states received! Size: %zu", jstate.position.size());
      first_msg = false;
    }
  }

  void UCommandCallback(const std_msgs::msg::Float64& msg) {
    u_cmd_ = msg.data;
  }

  // Service functions
  void StartGrasp(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void)req;
    (void)res;
    start_close_ = true;
  }

  void StartRelease(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void)req;
    (void)res;
    start_release_ = true;
  }

  void CallStartAdaptiveGrasp() {
    auto request = std::make_shared<grasp_srvs::srv::StartAdaptiveGraspCtrl::Request>();
    // No request fields for adaptive grasp
    adaptive_grasp_start_->async_send_request(request);
  }

  void CallStartConstantGrasp() {
    auto request = std::make_shared<grasp_srvs::srv::StartConstantGraspCtrl::Request>();
    request->referece_force = fixed_force_ref_;  // Note: typo in service definition
    constant_grasp_start_->async_send_request(request);
  }

  void CallStartLoad() {
    auto request = std::make_shared<grasp_srvs::srv::StartLoadCtrl::Request>();
    load_start_->async_send_request(request);
  }

  void CallStopGrasp() {
    auto request = std::make_shared<grasp_srvs::srv::StopGraspCtrl::Request>();
    grasp_stop_->async_send_request(request);
  }

  void CallStartAdaptiveRelease() {
    auto request = std::make_shared<grasp_srvs::srv::StartAdaptiveReleaseCtrl::Request>();
    adaptive_release_start_->async_send_request(request);
  }

  void CallStopRelease() {
    auto request = std::make_shared<grasp_srvs::srv::StopReleaseCtrl::Request>();
    release_stop_->async_send_request(request);
  }

  void CallZeroReset() {
    auto request = std::make_shared<grasp_srvs::srv::TactileZeroReset::Request>();
    zero_reset_start_->async_send_request(request);
  }

  void OpenCloseGripper() {
    // Start closing gripper slowly (u=0.0 is fully closed, u=1.0 is fully open)
    // Start from slightly open position and slowly close until contact detected
    // Do NOT reset u_cmd_ here so closing will start from the current u_cmd_
    // (which is set to 0.9 in standby). This prevents a jump to a different
    // start position when transitioning to CloseGripper.

    auto trajectory_name = std_msgs::msg::String();
    trajectory_name.data = "close_block";
    trajectory_pub_->publish(trajectory_name);
  }

  void OpenGripper() {
    auto trajectory_name = std_msgs::msg::String();
    trajectory_name.data = "open_block";
    trajectory_pub_->publish(trajectory_name);
  }

  void SaveContactPosition() {
    pos_contact_.clear();
    for (const auto &q : jstate_.position) {
      pos_contact_.push_back(q);
    }
  }

  // Measure current grasp force from tactile sensors. Use the Z component
  // (normal force) and return whichever sensor reports the larger value.
  // If no data yet, values default to 0.
  double MeasuredForce() {
    double f0z = tactile_0_.force.z;
    double f1z = tactile_1_.force.z;
    f0z = std::max(0.0, f0z);
    f1z = std::max(0.0, f1z);
    return std::max(f0z, f1z);
  }

  void ClearContactPosition() {
    pos_contact_.clear();
  }

  void ApplyForce(double force) {
    applied_force = force;
    // position control
    double pos = force * kForceToPosGain;
    pos_ref_ = pos;
    SendPosCommand(pos);
  }

  void SendPosCommand(double diff) {
    // Map position difference to normalized grasp command [0,1] (feedforward)
    // diff > 0 means closing the gripper (applying force)
    // u=0.0 is fully closed, u=1.0 is fully open
    const double MAX_DIFF = 0.05;  // Maximum expected diff value

    double normalized_cmd = 0.3 - (diff / MAX_DIFF) * 0.3;
    normalized_cmd = std::clamp(normalized_cmd, 0.0, 1.0);
    double u_ff = normalized_cmd;

    // Force tracking error (desired - measured). Positive error means we
    // need more force -> close gripper -> decrease u, so feedback is
    // inverted (negative sign).
    double measured = MeasuredForce();
    double desired = force_;
    double err = desired - measured;

    const double dt = 1.0 / ROS_HZ;
    u_integral_ += err * dt;
    u_integral_ = std::clamp(u_integral_, -kU_I_LIMIT, kU_I_LIMIT);

    double u_fb = -(kU_KP * err + kU_KI * u_integral_);

    u_cmd_ = std::clamp(u_ff + u_fb, 0.0, 1.0);
  }

  void Terminate() {
    RCLCPP_INFO(this->get_logger(), "Change state to Terminate.");
    time_sec = 0;
    force_ = 0.0;
    state_ = State::kTerminated;

    // Publish topic before termination
    sdr_grasp_msgs::msg::GraspControl grasp_info;
    grasp_info.grasp_force_control.reference_force = force_;
    grasp_info.state = (int)state_;
    grasp_info_pub_->publish(grasp_info);

    ApplyForce(0.0);
    ClearContactPosition();
  }

  void GraspReqCallback(const sdr_grasp_msgs::msg::GraspRequest& req) {
    RCLCPP_INFO(this->get_logger(), "Receive Grasp request.");
    RCLCPP_INFO(this->get_logger(), "  control_mode   :%d", req.control_mode);
    RCLCPP_INFO(this->get_logger(), "  adaptive_force :%d", req.adaptive_force.action);
    RCLCPP_INFO(this->get_logger(), "  force          :%f", req.fixed_force.force);
    grasp_requests_.push_back(req);
  }

  void GraspReqExec(sdr_grasp_msgs::msg::GraspRequest& req) {
    if (req.control_mode == TERMINATE) {
      RCLCPP_INFO(this->get_logger(), "=====  TERMINATE  =====");
      if (grasp_info_.state == GraspControl::GRASP_STABLE) {
        CallStopGrasp();
      } else if (grasp_info_.state == GraspControl::RELEASE_WAIT) {
        CallStopRelease();
      }
      Terminate();
      return;
    }

    if (req.adaptive_force.action == sdr_grasp_msgs::msg::AdaptiveForceRequest::GRASP) {
      if (state_ == State::kIdle) {
        RCLCPP_INFO(this->get_logger(), "=====  GRASP START  =====");
        control_mode_ = req.control_mode;
        fixed_force_ref_ = req.fixed_force.force;
        start_close_ = true;
        start_grasp_ = true;
      } else if (state_ == State::kCtrlGraspForce) {
        if (control_mode_ == sdr_grasp_msgs::msg::GraspRequest::FIXED_FORCE) {
          RCLCPP_INFO(this->get_logger(), "=====  UPDATE GRASP FORCE  =====");
          fixed_force_ref_ = req.fixed_force.force;
        }
      }
    } else {  // Release
      if (state_ == State::kCtrlGraspForce) {
        RCLCPP_INFO(this->get_logger(), "=====  RELEASE START  =====");
        control_mode_ = req.control_mode;
        start_release_ = true;
      }
    }
  }

  // Member variables
  ParallelGraspController controller_;

  // PI controllers for effort feedforward (Kp, Ki, Kd, ramp_rate, output_limit)
  PIDController pi_joint2_{1.0, 0.1, 0.0, 1.0, 10.0};  // joint3
  PIDController pi_joint3_{1.0, 0.1, 0.0, 1.0, 10.0};  // joint4
  PIDController pi_joint4_{1.0, 0.1, 0.0, 1.0, 10.0};  // joint5
  PIDController pi_joint5_{1.0, 0.1, 0.0, 1.0, 10.0};  // joint6

  State state_ = State::kIdle;
  double time_sec = 0;
  double init_force_;
  double u_cmd_ = 0.9;  // Normalized grasp command [0, 1]
  double u_integral_ = 0.0;  // Integral state for u PI regulator

  ContactStatus contact_state_0 = ContactStatus::kNoContact;
  ContactStatus contact_state_1 = ContactStatus::kNoContact;

  GraspForceState grasp_state_0_ = GraspForceState::kStandby;
  GraspForceState grasp_state_1_ = GraspForceState::kStandby;

  double grasp_force_0_ = 0;
  double grasp_force_1_ = 0;

  sensor_msgs::msg::JointState jstate_;
  std::vector<double> sorted_positions_;  // Reordered to [joint1..joint8]
  std::vector<double> pos_contact_;

  sdr_grasp_msgs::msg::Tactile tactile_0_;
  sdr_grasp_msgs::msg::Tactile tactile_1_;

  double applied_force = -1;
  double prev_force_;
  double force_;
  int16_t pos_ref_ = 0;
  int16_t vel_ref_ = 0;

  bool start_close_ = false;
  bool start_grasp_ = false;
  bool start_release_ = false;

  int control_mode_ = sdr_grasp_msgs::msg::GraspRequest::ADAPTIVE_FORCE;
  double fixed_force_ref_ = 0;

  int contact_cnt_ = 0;
  int no_contact_cnt_ = 0;
  int area_enough_cnt_ = 0;

  int cnt_ = 0;
  bool tick_is_exec_ = false;

  bool is_exit_process_ = false;
  bool is_busy_ = false;

  std::vector<sdr_grasp_msgs::msg::GraspRequest> grasp_requests_;
  sdr_grasp_msgs::msg::GraspControl grasp_info_;

  // ROS interfaces
  rclcpp::Client<grasp_srvs::srv::StartAdaptiveGraspCtrl>::SharedPtr adaptive_grasp_start_;
  rclcpp::Client<grasp_srvs::srv::StartConstantGraspCtrl>::SharedPtr constant_grasp_start_;
  rclcpp::Client<grasp_srvs::srv::StartLoadCtrl>::SharedPtr load_start_;
  rclcpp::Client<grasp_srvs::srv::StopGraspCtrl>::SharedPtr grasp_stop_;
  rclcpp::Client<grasp_srvs::srv::StartAdaptiveReleaseCtrl>::SharedPtr adaptive_release_start_;
  rclcpp::Client<grasp_srvs::srv::StopReleaseCtrl>::SharedPtr release_stop_;
  rclcpp::Client<grasp_srvs::srv::TactileZeroReset>::SharedPtr zero_reset_start_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_grasp_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_release_server_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tac_stat_fin0_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::Tactile>::SharedPtr tac_stat_fin1_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::GraspControl>::SharedPtr des_force_fin0_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::GraspControl>::SharedPtr des_force_fin1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr des_jstate_sub_;
  rclcpp::Subscription<sdr_grasp_msgs::msg::GraspRequest>::SharedPtr grasp_req_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr u_cmd_sub_;

  rclcpp::Publisher<sdr_grasp_msgs::msg::GraspControl>::SharedPtr grasp_info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<plato2_interfaces::msg::ImpedanceCommands>::SharedPtr impedance_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParallelGraspNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
