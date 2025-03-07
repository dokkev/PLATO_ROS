#ifndef PLATO2_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
#define PLATO2_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_

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
  double derivative_threshold_;
  int buffer_size_;
  double filter_alpha_;
  double derivative_time_window_;

  // Containers for FT sensor subscriptions (order: index, middle, thumb)
  std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> ft_subs_;

  // Containers for individual finger contact publishers
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> contact_pubs_;

  // Publisher for combined contact states
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr combined_contact_pub_;

  // Timer for publishing combined contact state
  rclcpp::TimerBase::SharedPtr timer_;

  // Storage for force measurements and contact states
  std::vector<std::deque<std::pair<rclcpp::Time, double>>> force_time_buffers_;
  std::vector<double> filtered_forces_;
  std::array<bool, 3> contact_states_;

  // Utility functions
  double calculate_force_magnitude(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  bool detect_contact(const std::deque<std::pair<rclcpp::Time, double>> & force_buffer);
  double calculate_force_derivative(const std::deque<std::pair<rclcpp::Time, double>> & force_buffer, double time_window);
  double apply_filter(double new_value, double previous_filtered, double alpha);

  // Process FT sensor data for a given finger index
  void process_ft_data(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, int finger_idx);

  // Timer callback to publish combined state
  void timer_callback();
};

#endif  // PLATO2_STATE_ESTIMATOR_CONTACT_ESTIMATOR_HPP_
