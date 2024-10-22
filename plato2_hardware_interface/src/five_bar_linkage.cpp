#include "plato2_hardware_interface/five_bar_linkage.hpp"


namespace FiveBarLinkage{

FiveBarLinkage::FiveBarLinkage(const FiveBarLinkageConfig &config) : config_(config){

}

/////////////////////////////////////////////////////////////////////////////////////

FiveBarLinkage::~FiveBarLinkage(){
}

/////////////////////////////////////////////////////////////////////////////////////

double FiveBarLinkage::get_pip_reduction_ratio(const double &mcp_motor_angle, const double &pip_motor_angle){
    // Calculate the reduction ratio of the PIP Motor
    const double L1 = config_.L1, L2 = config_.L2, L3 = config_.L3, L4 = config_.L4, L5 = config_.L5;
    const double theta1 = pip_motor_angle, theta2 = mcp_motor_angle;
    // Precompute the trigonometric functions once for reuse
    double cos_theta1 = std::cos(theta1);
    double sin_theta1 = std::sin(theta1);
    double cos_theta2 = std::cos(theta2);
    double sin_theta2 = std::sin(theta2);

    // Calculate positions for points based on link angles
    double x1 = L1 * cos_theta1, y1 = L1 * sin_theta1;
    double x4 = L5 + L4 * cos_theta2, y4 = L4 * sin_theta2;

    // Distance between (x1, y1) and (x4, y4)
    double dx = x4 - x1, dy = y4 - y1;
    double d = std::hypot(dx, dy);  // Efficient calculation of sqrt(x^2 + y^2)

    // Check for potential degenerate cases
    if (d == 0 || L2 + L3 < d || std::abs(L2 - L3) > d) {

        return 0.0;
    }

    // Intermediate calculations for forward kinematics
    double a = (L2 * L2 - L3 * L3 + d * d) / (2 * d);
    double h = std::sqrt(L2 * L2 - a * a);  // Only need sqrt once

    // Coordinates for the coupler point (L3)
    double x3 = x1 + a * dx / d - h * dy / d;
    double y3 = y1 + a * dy / d + h * dx / d;

    // Calculate L3 angle (relative to the x-axis)
    double l3_angle = std::atan2(y4 - y3, x4 - x3);

    // Relative angle between L3 and L4
    double l3_angle_relative = l3_angle - theta2;

    // Normalize the relative angle to the range [-pi, pi]
    l3_angle_relative = std::fmod(l3_angle_relative + M_PI, 2 * M_PI) - M_PI;

    // Return the ratio of relative L3 angle to L1 angle
    return l3_angle_relative / theta1;

}

} // namespace FiveBarLinkage