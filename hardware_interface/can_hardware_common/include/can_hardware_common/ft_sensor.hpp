#ifndef CAN_HARDWARE_COMMON__FT_SENSOR_HPP_
#define CAN_HARDWARE_COMMON__FT_SENSOR_HPP_

#include <cstddef>
#include <cstdint>

#include <PCANBasic.h>

#include <Eigen/Dense>

namespace sensor
{

struct States
{
  Eigen::Vector3f force_raw = Eigen::Vector3f::Zero();
  Eigen::Vector3f torque_raw = Eigen::Vector3f::Zero();
  Eigen::Vector3f force_filtered = Eigen::Vector3f::Zero();
  Eigen::Vector3f torque_filtered = Eigen::Vector3f::Zero();
};

struct Config
{
  uint32_t force_rx_id;
  uint32_t torque_rx_id;
};

class FTSensor
{
public:
  explicit FTSensor(const Config & config);
  ~FTSensor() = default;

  void apply_low_pass_filter(States & states);
  void process_message(const TPCANMsg & msg);
  void update_bias();

  States get_states() const { return states_; }
  uint32_t get_force_rx_id() const { return config_.force_rx_id; }
  uint32_t get_torque_rx_id() const { return config_.torque_rx_id; }

private:
  Config config_;
  float alpha_;

  States states_;

  Eigen::Vector3f bias_force_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f bias_torque_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f bias_accum_force_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f bias_accum_torque_ = Eigen::Vector3f::Zero();

  std::size_t calibration_samples_;
  bool force_bias_calibrated_;
  bool torque_bias_calibrated_;
  std::size_t bias_samples_force_;
  std::size_t bias_samples_torque_;
};

}  // namespace sensor

#endif  // CAN_HARDWARE_COMMON__FT_SENSOR_HPP_
