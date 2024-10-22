#ifndef PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_
#define PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_


#include "plato2_hardware_interface/utils/useful_functions.hpp"


namespace FiveBarLinkage{

struct FiveBarLinkageConfig{
    const double L1;
    const double L2;
    const double L3;
    const double L4;
    const double L5;

    const double L3_theta_offset;
};

class FiveBarLinkage{

public:
   /// @brief Constructor
    FiveBarLinkage(const FiveBarLinkageConfig &config);

    /// @brief Destructor
    ~FiveBarLinkage();

    void forward_kinematics(const double &mcp_motor_angle, const double &pip_motor_angle);
    void inverse_kinematics(const double &eef_x, const double &eef_y);

    // double L1 = config_.L1;
    // double L2 = config_.L2;
    // double L3 = config_.L3;
    // double L4 = config_.L4;
    // double L5 = config_.L5;
    // // Theta1 (L1) and Theta2 (L4) in radians
    // double theta1 = mcp_motor_angle;  // Equivalent to L1 in your diagram
    // double theta2 = pip_motor_angle;  // Equivalent to L4 in your diagram
    // // Calculate x and y positions for L1, L4
    // double x1 = L1 * cos(theta1);
    // double y1 = L1 * sin(theta1);
    // double x4 = L5 + L4 * cos(theta2);
    // double y4 = L4 * sin(theta2);
    // // Distance between (x1, y1) and (x4, y4)
    // double d = sqrt(pow(x4 - x1, 2) + pow(y4 - y1, 2));
    // // Intermediate calculations for forward kinematics
    // double a = (pow(L2, 2) - pow(L3, 2) + pow(d, 2)) / (2 * d);
    // double h = sqrt(pow(L2, 2) - pow(a, 2));
    // // Coordinates for the coupler point (L3)
    // double x3 = x1 + a * (x4 - x1) / d - h * (y4 - y1) / d;
    // double y3 = y1 + a * (y4 - y1) / d + h * (x4 - x1) / d;
    // // Calculate L3 angle (relative to x-axis)
    // double l3_angle = atan2(y4 - y3, x4 - x3);
    // // Calculate relative angle: L3 relative to L4
    // double l3_angle_relative = l3_angle - theta2;
    // // Normalize the relative angle to the range [-pi, pi]
    // l3_angle_relative = fmod(l3_angle_relative + M_PI, 2 * M_PI) - M_PI;
    // // Calculate the reduction ratio between L3 and L1 (PIP over MCP)
    // double reduction_ratio = l3_angle_relative / theta1;
    double get_pip_reduction_ratio(const double &mcp_motor_angle, const double &pip_motor_angle);


private:
    const FiveBarLinkageConfig config_;
     
};

}// namespace FiveBarLinkage




#endif // PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_