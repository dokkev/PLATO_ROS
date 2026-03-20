#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <unordered_map>
#include <iostream>
#include <iomanip>
#include <chrono>

struct WrenchSample {
  geometry_msgs::msg::Wrench wrench;
  rclcpp::Time stamp;
  bool valid{false};
};

class FTSensorMonitorNode : public rclcpp::Node {
public:
  FTSensorMonitorNode()
  : Node("ft_sensor_monitor_node") {
    // Default topics for three FT broadcasters
    std::vector<std::string> default_topics = {
      "/plato2/ft_sensor_broadcaster_1/wrench",
      "/plato2/ft_sensor_broadcaster_2/wrench",
      "/plato2/ft_sensor_broadcaster_3/wrench"
    };

    topics_ = this->declare_parameter<std::vector<std::string>>("topics", default_topics);
    double rate_hz = this->declare_parameter<double>("print_rate_hz", 20.0);

    // Subscriptions
    for (const auto & topic : topics_) {
      auto sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic,
        rclcpp::SensorDataQoS(),
        [this, topic](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          auto & sample = samples_[topic];
          sample.wrench = msg->wrench;
          sample.stamp = msg->header.stamp;
          sample.valid = true;
        }
      );
      subs_.push_back(sub);
      samples_[topic] = WrenchSample{};
    }

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FTSensorMonitorNode::print_table, this)
    );

    RCLCPP_INFO(this->get_logger(), "FT sensor monitor started with %zu topics", topics_.size());
  }

private:
  void print_table() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::cout << "\033[2J\033[H"; // clear screen
    std::cout << "--- FT Sensor Monitor ---" << std::endl;
    std::cout << std::left << std::setw(28) << "Topic"
              << std::right << std::setw(9) << "Fx"
              << std::setw(9) << "Fy"
              << std::setw(9) << "Fz"
              << std::setw(9) << "Tx"
              << std::setw(9) << "Ty"
              << std::setw(9) << "Tz"
              << std::setw(10) << "|F|"
              << std::setw(10) << "|T|" << std::endl;
    std::cout << std::string(92, '-') << std::endl;

    rclcpp::Time now = this->now();
    for (const auto & topic : topics_) {
      const auto & sample = samples_[topic];
      bool stale = !sample.valid || (now - sample.stamp).seconds() > 0.2;
      double fx = sample.wrench.force.x;
      double fy = sample.wrench.force.y;
      double fz = sample.wrench.force.z;
      double tx = sample.wrench.torque.x;
      double ty = sample.wrench.torque.y;
      double tz = sample.wrench.torque.z;
      double fmag = std::sqrt(fx*fx + fy*fy + fz*fz);
      double tmag = std::sqrt(tx*tx + ty*ty + tz*tz);

      std::cout << std::left << std::setw(28) << topic;
      if (stale) {
        std::cout << std::right << std::setw(9) << "-" << std::setw(9) << "-" << std::setw(9) << "-"
                  << std::setw(9) << "-" << std::setw(9) << "-" << std::setw(9) << "-"
                  << std::setw(10) << "-" << std::setw(10) << "-" << "  (stale)" << std::endl;
      } else {
        std::cout << std::fixed << std::setprecision(2)
                  << std::right << std::setw(9) << fx
                  << std::setw(9) << fy
                  << std::setw(9) << fz
                  << std::setw(9) << tx
                  << std::setw(9) << ty
                  << std::setw(9) << tz
                  << std::setw(10) << fmag
                  << std::setw(10) << tmag
                  << std::endl;
      }
    }
    std::cout.flush();
  }

  std::vector<std::string> topics_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> subs_;
  std::unordered_map<std::string, WrenchSample> samples_;
  std::mutex data_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FTSensorMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
