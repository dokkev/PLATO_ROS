#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace joint_position_controller {

struct ImpedanceGains {
  std::vector<double> stiffness;
  std::vector<double> damping;
};

/**
 * @brief Handles impedance gain presets and exposes services to switch between them.
 *
 * States: soft, medium, hard.
 */
class ImpedanceGainHandler {
public:
  enum class State { Zero, Soft, Medium, Hard };

  explicit ImpedanceGainHandler(rclcpp::Node& node);

  ImpedanceGains getGains();
  State getState() const;

private:
  void setState(State state);
  void configurePresets();
  std::vector<double> loadVectorParam(const std::string& name,
                                      const std::vector<double>& default_vec);
  static std::vector<double> uniformVector(double value);
  static std::vector<double> sanitizeVector(const std::vector<double>& in, double fallback);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr soft_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr medium_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hard_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;

  mutable std::mutex mutex_;
  State state_{State::Medium};
  ImpedanceGains gains_{uniformVector(0.0), uniformVector(0.0)};
  ImpedanceGains soft_{uniformVector(0.0), uniformVector(0.0)};
  ImpedanceGains medium_{uniformVector(0.0), uniformVector(0.0)};
  ImpedanceGains hard_{uniformVector(0.0), uniformVector(0.0)};
  ImpedanceGains zero_{uniformVector(0.0), uniformVector(0.0)};

  rclcpp::Node* node_;
  rclcpp::Logger logger_;
};

}  // namespace joint_position_controller
