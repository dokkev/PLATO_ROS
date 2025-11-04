#pragma once
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>


enum class ContactStatus {
    kNoContact = 0,
    kFewContact = 1,
    kEnoughContact = 2,
};  


// Lightweight geometric parallel-grasp mapper.
//
// The geometric parameters (L, d, h) and joint limits (qmin,qmax) are
// hardcoded here as constexpr values. Change them in this header if you
// want different geometry. The class precomputes feasible gap bounds in the
// constructor so `command()` calls are fast. Note: `command()` currently
// returns a std::vector<double> (4 elements) and therefore performs a small
// allocation on each call; if allocation-free operation is required, change
// the API to fill an output buffer or return a fixed-size aggregate type.
class ParallelGraspController {
public:
  // --- Hardcoded geometric and joint-limit constants ---
  static constexpr double L   = 0.06;   // tip radius (m)
  static constexpr double w   = -0.022;    // lateral offset / finger half-width (m)
  static constexpr double h   = -0.098;    // vertical offset (m)
  static constexpr double qmin = -1.0472; // joint lower limit (rad)
  static constexpr double qmax = 1.0472; // joint upper limit (rad)

  ParallelGraspController();

  // Map u in [0,1] -> joint commands vector (8 elements).
  // The returned std::vector<double> contains positions for joints 1..8 in
  // order. The grasp mapping sets joints 3..6 (indices 2..5) to the computed
  // values (q3,q4,q5,q6). Other joints are currently zero.
  // Map normalized command u_cmd (0..1) -> 8 joint angles. This method may
  // update internal bookkeeping (last commanded u and gap), so it is non-const.
  // Compute and return (by const-ref) the last full 8-element joint position
  // command vector. Returning by const-ref avoids an extra copy.
  const std::vector<double>& get_commands(double u_cmd);

private:

  // These are initialized at construction time using the constexpr L and w.
  const double Dy_max_ = std::sqrt(std::max(0.0, (2.0 * L)*(2.0 * L) - w*w));
  const double d_min_  = h - Dy_max_;
  const double d_max_  = h + Dy_max_;
  // Last commanded gap (meters). Updated by `command()`; stored here for inspection.
  double d_ = d_min_;

  // Accessor for the last commanded gap (meters).
  double last_gap() const { return d_; }
  // Last normalized command (0..1)
  double u_ = 0.5;

  // Accessor for last normalized command
  double get_gap() const { return d_; }
  double get_command() const { return u_; }

  // Last full 8-element joint position command (indices 0..7 -> joints 1..8).
  // Filled by get_commands()/commands().
  std::vector<double> joint_position_commands_ = std::vector<double>(8, 0.0);
};
