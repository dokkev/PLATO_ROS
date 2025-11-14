#include "plato2_state_estimator/contact_estimator.hpp"
#include <iostream>
using std::placeholders::_1;

ContactEstimator::ContactEstimator()
: Node("contact_estimator")
{
  this->declare_parameter("force_threshold", 0.8);
  this->declare_parameter("buffer_size", 100);
  this->declare_parameter("filter_alpha", 0.2);
  this->declare_parameter("ft_sensor_thumb_topic", "/plato2/ft_sensor_broadcaster_1/wrench");
  this->declare_parameter("ft_sensor_index_topic", "/plato2/ft_sensor_broadcaster_2/wrench");
  this->declare_parameter("ft_sensor_middle_topic", "/plato2/ft_sensor_broadcaster_3/wrench");

  force_threshold_ = this->get_parameter("force_threshold").as_double();
  buffer_size_ = this->get_parameter("buffer_size").as_int();
  filter_alpha_ = this->get_parameter("filter_alpha").as_double();

  std::vector<std::string> ft_topics = {
    this->get_parameter("ft_sensor_thumb_topic").as_string(),
    this->get_parameter("ft_sensor_index_topic").as_string(),
    this->get_parameter("ft_sensor_middle_topic").as_string()
  };

  // Resize containers
  force_time_buffers_.resize(3);
  filtered_forces_.resize(3, 0.0);
  contact_states_.fill(false);
  bias_force_.resize(3, 0.0);
  calibration_counter_.resize(3, 0);

  // Subscribe to each FT sensor
  // Wait for FT sensor publishers before subscribing
  RCLCPP_INFO(this->get_logger(), "Waiting for FT sensor topics to become available...");
  for (size_t i = 0; i < ft_topics.size(); ++i) {
    const std::string &topic = ft_topics[i];
    bool ready = false;
    rclcpp::Rate rate(10);  // 10 Hz polling

    while (rclcpp::ok() && !ready) {
      auto topics_and_types = this->get_topic_names_and_types();
      if (topics_and_types.count(topic) > 0 &&
          std::find(topics_and_types[topic].begin(), topics_and_types[topic].end(),
                    "geometry_msgs/msg/WrenchStamped") != topics_and_types[topic].end()) {
        ready = true;
        break;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Waiting for topic '%s' to be available...", topic.c_str());
      rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Connected to FT sensor topic: %s", topic.c_str());

    ft_subs_.push_back(
      this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic, 10,
        [this, i](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          this->process_ft_data(msg, static_cast<int>(i));
        }
      )
    );
  }


  // Individual contact state publishers
  contact_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>("plato2/b_contact_thumb", 10));
  contact_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>("plato2/b_contact_index", 10));
  contact_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>("plato2/b_contact_middle", 10));

  // Combined state publisher
  combined_contact_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("b_array_contact", 10);

  // Timer for publishing combined state
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ContactEstimator::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Contact Estimator Node initialized");
}

bool ContactEstimator::calibration_done(int finger_idx) const {
  return calibration_counter_[finger_idx] >= 100;
}

double ContactEstimator::calculate_force_magnitude(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  double fx = msg->wrench.force.x;
  double fy = msg->wrench.force.y;
  double fz = msg->wrench.force.z;
  return std::sqrt(fx * fx + fy * fy + fz * fz);
}

double ContactEstimator::apply_filter(double new_value, double previous_filtered, double alpha)
{
  return alpha * new_value + (1.0 - alpha) * previous_filtered;
}

void ContactEstimator::process_ft_data(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, int finger_idx)
{
  try {
    if (!msg) return;

    rclcpp::Time timestamp = msg->header.stamp;
    double raw_force_mag = calculate_force_magnitude(msg);

    // === Skip 0.0 or near-zero force ===
    if (raw_force_mag < 1e-5) {
      // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      //                      "Skipping near-zero FT data for finger %d (%.5f)", finger_idx, raw_force_mag);
      return;
    }

    // Calibration phase
    if (!calibration_done(finger_idx)) {
      bias_force_[finger_idx] += raw_force_mag;
      calibration_counter_[finger_idx] += 1;

      if (calibration_counter_[finger_idx] == 100) {
        bias_force_[finger_idx] /= 100.0;
        RCLCPP_INFO(this->get_logger(), "Bias calibrated for finger %d: %.3f", finger_idx, bias_force_[finger_idx]);
      }
      return;  // Skip until calibration is complete
    }

    // Subtract bias and filter
    double unbiased_force = raw_force_mag - bias_force_[finger_idx];
    filtered_forces_[finger_idx] = apply_filter(unbiased_force, filtered_forces_[finger_idx], filter_alpha_);

    // Store in buffer
    force_time_buffers_[finger_idx].push_back(std::make_pair(timestamp, filtered_forces_[finger_idx]));
    while (force_time_buffers_[finger_idx].size() > static_cast<size_t>(buffer_size_)) {
      force_time_buffers_[finger_idx].pop_front();
    }

    // Contact detection: threshold only
    bool contact = (filtered_forces_[finger_idx] > force_threshold_);
    contact_states_[finger_idx] = contact;

    // Publish individual state
    auto contact_msg = std::make_unique<std_msgs::msg::Bool>();
    contact_msg->data = contact;
    contact_pubs_[finger_idx]->publish(std::move(contact_msg));
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing FT data for finger %d: %s", finger_idx, e.what());
  }
}

void ContactEstimator::timer_callback()
{
  try {
    auto byte_msg = std::make_unique<std_msgs::msg::ByteMultiArray>();
    byte_msg->layout.dim.resize(1);
    byte_msg->layout.dim[0].label = "fingers";
    byte_msg->layout.dim[0].size = 3;
    byte_msg->layout.dim[0].stride = 3;
    byte_msg->data.resize(3);

    for (size_t i = 0; i < contact_states_.size(); ++i) {
      byte_msg->data[i] = contact_states_[i] ? 1 : 0;
    }
    combined_contact_pub_->publish(std::move(byte_msg));
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in timer callback: %s", e.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
