#pragma once

namespace turntable_hardware_interface
{

class MinJerkTraj
{
public:
  struct Sample
  {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
  };

  MinJerkTraj() = default;
  MinJerkTraj(double start_position, double target_position, double duration_sec);

  void reset(double start_position, double target_position, double duration_sec);

  Sample sample(double elapsed_sec) const;
  double position(double elapsed_sec) const;
  double velocity(double elapsed_sec) const;
  double acceleration(double elapsed_sec) const;

  bool is_done(double elapsed_sec) const;

  double start_position() const {return start_position_;}
  double target_position() const {return target_position_;}
  double duration_sec() const {return duration_sec_;}

private:
  double clamp_time_(double elapsed_sec) const;
  double normalized_time_(double elapsed_sec) const;

  double start_position_ = 0.0;
  double target_position_ = 0.0;
  double duration_sec_ = 1.0;
};

}  // namespace turntable_hardware_interface
