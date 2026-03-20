#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "aristo_hardware_interface/ft_sensor_can.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <deque>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

struct FTData {
  double fx{0.0}, fy{0.0}, fz{0.0};
  double tx{0.0}, ty{0.0}, tz{0.0};
  double off_fx{0.0}, off_fy{0.0}, off_fz{0.0};
  double off_tx{0.0}, off_ty{0.0}, off_tz{0.0};
  std::mutex mtx;
};

class FTSensorBringup : public rclcpp::Node {
public:
  FTSensorBringup()
  : Node("ft_sensor_bringup"), running_(true) {
    interface_name_ = this->declare_parameter<std::string>("interface", "can0");
    force_id_1_ = this->declare_parameter<int>("force_id_1", 0x02A);
    torque_id_1_ = this->declare_parameter<int>("torque_id_1", 0x02B);
    force_id_2_ = this->declare_parameter<int>("force_id_2", 0x03A);
    torque_id_2_ = this->declare_parameter<int>("torque_id_2", 0x03B);
    frame_id_1_ = this->declare_parameter<std::string>("frame_id_1", "ft_sensor1");
    frame_id_2_ = this->declare_parameter<std::string>("frame_id_2", "ft_sensor2");
    print_rate_hz_ = this->declare_parameter<double>("print_rate_hz", 20.0);
    ma_force_window_ = this->declare_parameter<int>("moving_average_force_window", 10);
    ma_torque_window_ = this->declare_parameter<int>("moving_average_torque_window", 4);
    if (ma_force_window_ < 1) ma_force_window_ = 1;
    if (ma_torque_window_ < 1) ma_torque_window_ = 1;

    pub1_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/plato2/ft_sensor1/wrench", 10);
    pub2_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/plato2/ft_sensor2/wrench", 10);

    start_sensors();

    auto period = std::chrono::duration<double>(1.0 / print_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FTSensorBringup::print_loop, this));

    keyboard_thread_ = std::thread(&FTSensorBringup::keyboard_loop, this);
    RCLCPP_INFO(this->get_logger(), "ft_sensor_bringup started on %s with 2 sensors", interface_name_.c_str());
  }

  ~FTSensorBringup() override {
    running_ = false;
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
    sensor1_.reset();
    sensor2_.reset();
  }

private:
  void start_sensors() {
    sensor1_ = std::make_unique<FTSensorCAN>(interface_name_, static_cast<uint32_t>(force_id_1_), static_cast<uint32_t>(torque_id_1_));
    sensor2_ = std::make_unique<FTSensorCAN>(interface_name_, static_cast<uint32_t>(force_id_2_), static_cast<uint32_t>(torque_id_2_));

    sensor1_->set_callback([this](double fx, double fy, double fz, double tx, double ty, double tz) {
      handle_sample(0, fx, fy, fz, tx, ty, tz);
    });
    sensor2_->set_callback([this](double fx, double fy, double fz, double tx, double ty, double tz) {
      handle_sample(1, fx, fy, fz, tx, ty, tz);
    });
  }

  void handle_sample(int idx, double fx, double fy, double fz, double tx, double ty, double tz) {
    auto & data = (idx == 0) ? data1_ : data2_;
    auto & samples = (idx == 0) ? samples1_ : samples2_;
    auto & fbuf = (idx == 0) ? fbuf1_ : fbuf2_;
    auto & fsum = (idx == 0) ? fsum1_ : fsum2_;
    auto & tbuf = (idx == 0) ? tbuf1_ : tbuf2_;
    auto & tsum = (idx == 0) ? tsum1_ : tsum2_;
    auto & pub = (idx == 0) ? pub1_ : pub2_;
    auto frame_id = (idx == 0) ? frame_id_1_ : frame_id_2_;

    if (is_calibrating_) {
      std::lock_guard<std::mutex> s_lock(samples_mtx_);
      samples.push_back({fx, fy, fz, tx, ty, tz});
    }

    double adj_fx, adj_fy, adj_fz, adj_tx, adj_ty, adj_tz;
    {
      std::lock_guard<std::mutex> lock(data.mtx);
      data.fx = fx - data.off_fx;
      data.fy = fy - data.off_fy;
      data.fz = fz - data.off_fz;
      data.tx = tx - data.off_tx;
      data.ty = ty - data.off_ty;
      data.tz = tz - data.off_tz;
      // Moving average smoothing (separate force/torque windows)
      std::array<double,3> fsample{data.fx, data.fy, data.fz};
      std::array<double,3> tsample{data.tx, data.ty, data.tz};

      fbuf.push_back(fsample);
      for (int i = 0; i < 3; ++i) fsum[i] += fsample[i];
      if (static_cast<int>(fbuf.size()) > ma_force_window_) {
        auto old = fbuf.front();
        for (int i = 0; i < 3; ++i) fsum[i] -= old[i];
        fbuf.pop_front();
      }

      tbuf.push_back(tsample);
      for (int i = 0; i < 3; ++i) tsum[i] += tsample[i];
      if (static_cast<int>(tbuf.size()) > ma_torque_window_) {
        auto old = tbuf.front();
        for (int i = 0; i < 3; ++i) tsum[i] -= old[i];
        tbuf.pop_front();
      }

      double fden = static_cast<double>(fbuf.size());
      double tden = static_cast<double>(tbuf.size());
      adj_fx = fsum[0]/fden; adj_fy = fsum[1]/fden; adj_fz = fsum[2]/fden;
      adj_tx = tsum[0]/tden; adj_ty = tsum[1]/tden; adj_tz = tsum[2]/tden;
    }

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = adj_fx;
    msg.wrench.force.y = adj_fy;
    msg.wrench.force.z = adj_fz;
    msg.wrench.torque.x = adj_tx;
    msg.wrench.torque.y = adj_ty;
    msg.wrench.torque.z = adj_tz;
    pub->publish(msg);
  }

  void perform_tare(int seconds) {
    is_calibrating_ = true;
    {
      std::lock_guard<std::mutex> s_lock(samples_mtx_);
      samples1_.clear();
      samples2_.clear();
    }
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    is_calibrating_ = false;

    auto calc_off = [](FTData & d, std::vector<std::vector<double>> & s) {
      if (s.empty()) return;
      double sum[6] = {0,0,0,0,0,0};
      for (auto & row : s) for (int i = 0; i < 6; ++i) sum[i] += row[i];
      std::lock_guard<std::mutex> d_lock(d.mtx);
      d.off_fx = sum[0]/s.size(); d.off_fy = sum[1]/s.size(); d.off_fz = sum[2]/s.size();
      d.off_tx = sum[3]/s.size(); d.off_ty = sum[4]/s.size(); d.off_tz = sum[5]/s.size();
    };

    std::lock_guard<std::mutex> s_lock(samples_mtx_);
    calc_off(data1_, samples1_);
    calc_off(data2_, samples2_);
  }

  void keyboard_loop() {
    // Set terminal to non-canonical
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_) {
      char c;
      if (read(STDIN_FILENO, &c, 1) == 1) {
        if (c == 't' || c == 'T') {
          std::cout << "\n[ft_sensor_bringup] Taring..." << std::endl;
          perform_tare(1);
          std::cout << "[ft_sensor_bringup] Tare done" << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  void print_loop() {
    auto copy = [](FTData & d) {
      std::lock_guard<std::mutex> lock(d.mtx);
      return std::array<double,6>{d.fx, d.fy, d.fz, d.tx, d.ty, d.tz};
    };

    auto s1 = copy(data1_);
    auto s2 = copy(data2_);

    auto print_row = [](const std::string & label, const std::array<double,6> & v) {
      double fmag = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
      double tmag = std::sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5]);
      std::cout << std::left << std::setw(8) << label << " "
                << std::fixed << std::setprecision(2)
                << std::right << std::setw(8) << v[0] << std::setw(8) << v[1] << std::setw(8) << v[2]
                << " | "
                << std::setw(8) << v[3] << std::setw(8) << v[4] << std::setw(8) << v[5]
                << " | "
                << std::setw(8) << fmag << std::setw(8) << tmag << std::endl;
    };

    std::cout << "\033[2J\033[H";
    std::cout << "--- FT Sensor Bringup (press 't' to tare) ---" << std::endl;
    std::cout << "         "
              << std::setw(8) << "Fx" << std::setw(8) << "Fy" << std::setw(8) << "Fz"
              << " | "
              << std::setw(8) << "Tx" << std::setw(8) << "Ty" << std::setw(8) << "Tz"
              << " | "
              << std::setw(8) << "|F|" << std::setw(8) << "|T|" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    print_row("S1", s1);
    print_row("S2", s2);
    std::cout.flush();
  }

  // Parameters
  std::string interface_name_;
  int force_id_1_;
  int torque_id_1_;
  int force_id_2_;
  int torque_id_2_;
  std::string frame_id_1_;
  std::string frame_id_2_;
  double print_rate_hz_;
  int ma_force_window_;
  int ma_torque_window_;

  // Sensors and data
  std::unique_ptr<FTSensorCAN> sensor1_;
  std::unique_ptr<FTSensorCAN> sensor2_;
  FTData data1_;
  FTData data2_;
  std::vector<std::vector<double>> samples1_;
  std::vector<std::vector<double>> samples2_;
  std::mutex samples_mtx_;
  std::atomic<bool> is_calibrating_{true};

  // smoothing buffers (force and torque separate)
  std::deque<std::array<double,3>> fbuf1_;
  std::deque<std::array<double,3>> fbuf2_;
  std::deque<std::array<double,3>> tbuf1_;
  std::deque<std::array<double,3>> tbuf2_;
  std::array<double,3> fsum1_{{0,0,0}};
  std::array<double,3> fsum2_{{0,0,0}};
  std::array<double,3> tsum1_{{0,0,0}};
  std::array<double,3> tsum2_{{0,0,0}};

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub1_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Threads
  std::thread keyboard_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FTSensorBringup>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
