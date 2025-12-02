#include "joint_position_controller/impedance_gain_handler.hpp"

namespace joint_position_controller {

namespace {
ImpedanceGains makePreset(const std::vector<double>& stiff, const std::vector<double>& damp) {
  return ImpedanceGains{stiff, damp};
}

const char* toLabel(ImpedanceGainHandler::State state) {
  switch (state) {
    case ImpedanceGainHandler::State::Zero: return "zero";
    case ImpedanceGainHandler::State::Soft: return "soft";
    case ImpedanceGainHandler::State::Medium: return "medium";
    case ImpedanceGainHandler::State::Hard: return "hard";
  }
  return "unknown";
}
}  // namespace

ImpedanceGainHandler::ImpedanceGainHandler(rclcpp::Node& node)
    : node_(&node), logger_(node.get_logger()) {
  configurePresets();
  setState(State::Medium);

  auto make_cb = [this](State target_state) {
    return [this, target_state](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      setState(target_state);
      resp->success = true;
      resp->message = "Switched impedance state";
    };
  };

  zero_srv_ = node.create_service<std_srvs::srv::Trigger>(
      "impedance_gains/zero", make_cb(State::Zero));
  soft_srv_ = node.create_service<std_srvs::srv::Trigger>(
      "impedance_gains/soft", make_cb(State::Soft));
  medium_srv_ = node.create_service<std_srvs::srv::Trigger>(
      "impedance_gains/medium", make_cb(State::Medium));
  hard_srv_ = node.create_service<std_srvs::srv::Trigger>(
      "impedance_gains/hard", make_cb(State::Hard));

  RCLCPP_INFO(logger_, "ImpedanceGainHandler initialized (services: zero, soft, medium, hard)");
}

void ImpedanceGainHandler::configurePresets() {
  zero_ = makePreset(
      loadVectorParam("impedance_presets.zero.stiffness", uniformVector(0.0)),
      loadVectorParam("impedance_presets.zero.damping",   uniformVector(0.0)));

  soft_ = makePreset(
      loadVectorParam("impedance_presets.soft.stiffness", uniformVector(0.0)),
      loadVectorParam("impedance_presets.soft.damping",   uniformVector(0.0)));

  medium_ = makePreset(
      loadVectorParam("impedance_presets.medium.stiffness", uniformVector(0.0)),
      loadVectorParam("impedance_presets.medium.damping",   uniformVector(0.0)));

  hard_ = makePreset(
      loadVectorParam("impedance_presets.hard.stiffness", uniformVector(0.0)),
      loadVectorParam("impedance_presets.hard.damping",   uniformVector(0.0)));
}

void ImpedanceGainHandler::setState(State state) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = state;
  switch (state_) {
    case State::Zero:
      gains_ = zero_;
      break;
    case State::Soft:
      gains_ = soft_;
      break;
    case State::Medium:
      gains_ = medium_;
      break;
    case State::Hard:
      gains_ = hard_;
      break;
  }
  RCLCPP_INFO(logger_, "Impedance gains set to: %s", toLabel(state_));
  const auto& s = gains_.stiffness;
  const auto& d = gains_.damping;
  RCLCPP_INFO(logger_, "  stiffness: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
              s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);
  RCLCPP_INFO(logger_, "  damping  : [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
              d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
}

ImpedanceGains ImpedanceGainHandler::getGains() {
  std::lock_guard<std::mutex> lock(mutex_);
  return gains_;
}

ImpedanceGainHandler::State ImpedanceGainHandler::getState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::vector<double> ImpedanceGainHandler::loadVectorParam(const std::string& name,
                                                          const std::vector<double>& default_vec) {
  const auto raw = node_->declare_parameter<std::vector<double>>(name, default_vec);
  const double fallback = default_vec.empty() ? 0.0 : default_vec.front();
  return sanitizeVector(raw, fallback);
}

std::vector<double> ImpedanceGainHandler::uniformVector(double value) {
  return std::vector<double>(8, value);
}

std::vector<double> ImpedanceGainHandler::sanitizeVector(const std::vector<double>& in, double fallback) {
  std::vector<double> out = in;
  if (out.size() < 8) {
    out.resize(8, fallback);
  } else if (out.size() > 8) {
    out.resize(8);
  }
  return out;
}

}  // namespace joint_position_controller
