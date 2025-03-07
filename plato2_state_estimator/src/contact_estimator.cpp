#include "plato2_state_estimator/contact_estimator.hpp"

using std::placeholders::_1;

ContactEstimator::ContactEstimator()
: Node("contact_estimator")
{
  // Declare and get parameters
  this->declare_parameter("force_threshold", 1.0);
  this->declare_parameter("derivative_threshold", 5.0);
  this->declare_parameter("buffer_size", 50);
  this->declare_parameter("filter_alpha", 0.2);
  this->declare_parameter("derivative_time_window", 0.05);
  this->declare_parameter("ft_sensor_index_topic", "ft_index");
  this->declare_parameter("ft_sensor_middle_topic", "ft_middle");
  this->declare_parameter("ft_sensor_thumb_topic", "ft_thumb");

  force_threshold_ = this->get_parameter("force_threshold").as_double();
  derivative_threshold_ = this->get_parameter("derivative_threshold").as_double();
  buffer_size_ = this->get_parameter("buffer_size").as_int();
  filter_alpha_ = this->get_parameter("filter_alpha").as_double();
  derivative_time_window_ = this->get_parameter("derivative_time_window").as_double();

  // Set up FT topics
  std::vector<std::string> ft_topics = {
    this->get_parameter("ft_sensor_index_topic").as_string(),
    this->get_parameter("ft_sensor_middle_topic").as_string(),
    this->get_parameter("ft_sensor_thumb_topic").as_string()
  };

  // Initialize data structures
  force_time_buffers_.resize(3);
  filtered_forces_.resize(3, 0.0);
  contact_states_.fill(false);

  // Create subscribers for all FT sensors
  for (size_t i = 0; i < ft_topics.size(); ++i) {
    ft_subs_.push_back(
      this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        ft_topics[i],
        10,
        [this, i](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          this->process_ft_data(msg, static_cast<int>(i));
        }
      )
    );
  }

  // Create publishers for individual contact states
  contact_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>("plato2/b_contact_thumb", 10));
  contact_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>("plato2/b_contact_index", 10));
  contact_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>("plato2/b_contact_middle", 10));

  // Create publisher for combined contact state
  combined_contact_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("b_array_contact", 10);

  // Create timer for publishing combined state
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ContactEstimator::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Contact Estimator Node initialized");
  RCLCPP_INFO(this->get_logger(), "Listening to FT sensors on topics: %s, %s, %s",
              ft_topics[0].c_str(), ft_topics[1].c_str(), ft_topics[2].c_str());
}

void ContactEstimator::process_ft_data(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, int finger_idx)
{
  try {
    if (!msg) {
      RCLCPP_WARN(this->get_logger(), "Received null message for finger %d", finger_idx);
      return;
    }

    rclcpp::Time timestamp = msg->header.stamp;
    double force_mag = calculate_force_magnitude(msg);

    // Apply low-pass filter
    filtered_forces_[finger_idx] = apply_filter(force_mag, filtered_forces_[finger_idx], filter_alpha_);

    // Store timestamp and filtered force in buffer
    force_time_buffers_[finger_idx].push_back(std::make_pair(timestamp, filtered_forces_[finger_idx]));

    // Maintain buffer size
    while (force_time_buffers_[finger_idx].size() > static_cast<size_t>(buffer_size_)) {
      force_time_buffers_[finger_idx].pop_front();
    }

    // Detect contact based on force magnitude and its derivative
    bool contact = detect_contact(force_time_buffers_[finger_idx]);
    contact_states_[finger_idx] = contact;

    // Publish individual contact state
    auto contact_msg = std::make_unique<std_msgs::msg::Bool>();
    contact_msg->data = contact;
    contact_pubs_[finger_idx]->publish(std::move(contact_msg));
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing FT data for finger %d: %s", finger_idx, e.what());
  }
}

double ContactEstimator::calculate_force_magnitude(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  double fx = msg->wrench.force.x;
  double fy = msg->wrench.force.y;
  double fz = msg->wrench.force.z;
  return std::sqrt(fx * fx + fy * fy + fz * fz);
}

bool ContactEstimator::detect_contact(const std::deque<std::pair<rclcpp::Time, double>> & force_buffer)
{
  if (force_buffer.size() < 2) {
    return false;
  }
  double derivative = calculate_force_derivative(force_buffer, derivative_time_window_);
  double current_force = force_buffer.back().second;
  return (current_force > force_threshold_ && std::abs(derivative) > derivative_threshold_);
}

double ContactEstimator::calculate_force_derivative(const std::deque<std::pair<rclcpp::Time, double>> & force_buffer, double time_window)
{
  const auto& current = force_buffer.back();
  double current_force = current.second;
  rclcpp::Time current_time = current.first;

  double oldest_force = current_force;
  rclcpp::Time oldest_time = current_time;
  bool found_old = false;

  for (auto it = force_buffer.rbegin(); it != force_buffer.rend(); ++it) {
    double dt = (current_time - it->first).seconds();
    if (dt >= time_window) {
      oldest_force = it->second;
      oldest_time = it->first;
      found_old = true;
      break;
    }
  }
  if (!found_old && force_buffer.size() > 1) {
    oldest_force = force_buffer.front().second;
    oldest_time = force_buffer.front().first;
  }
  double dt = (current_time - oldest_time).seconds();
  if (dt < 1e-6) {
    return 0.0;
  }
  return (current_force - oldest_force) / dt;
}

double ContactEstimator::apply_filter(double new_value, double previous_filtered, double alpha)
{
  return alpha * new_value + (1.0 - alpha) * previous_filtered;
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
