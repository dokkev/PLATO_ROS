#ifndef PLATO_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
#define PLATO_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <vector>
#include <deque>
#include <array>
#include <string>
#include <utility>
#include <memory>
#include <cmath>

class ContactEstimator : public rclcpp::Node
{
public:
  ContactEstimator();

private:
  // Parameters
  double force_threshold_;
  int buffer_size_;
  double filter_alpha_;

  // FT sensor subscribers and publishers
  std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> ft_subs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> contact_pubs_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr combined_contact_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data buffers
  std::vector<std::deque<std::pair<rclcpp::Time, double>>> force_time_buffers_;
  std::vector<double> filtered_forces_;
  std::array<bool, 3> contact_states_;

  // Bias calibration
  std::vector<double> bias_force_;
  std::vector<int> calibration_counter_;
  bool calibration_done(int finger_idx) const;

  // Utility functions
  double calculate_force_magnitude(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  double apply_filter(double new_value, double previous_filtered, double alpha);
  void process_ft_data(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, int finger_idx);
  void timer_callback();
};

#endif  // PLATO_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
