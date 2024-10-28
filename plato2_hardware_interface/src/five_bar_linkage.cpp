#include "plato2_hardware_interface/five_bar_linkage.hpp"


namespace FiveBarLinkage{

FiveBarLinkage::FiveBarLinkage(const FiveBarLinkageConfig &config) : config_(config){

}

/////////////////////////////////////////////////////////////////////////////////////

FiveBarLinkage::~FiveBarLinkage(){
}

/////////////////////////////////////////////////////////////////////////////////////

float FiveBarLinkage::update_kinematics(const float &mcp_motor_angle, const float &pip_motor_angle) {
    float theta1 = pip_motor_angle + 2.35619; // PIP Motor Position
    float theta4 = mcp_motor_angle; // MCP Motor Position

    // Corner points calculation
    // float Xb_x = config_.L1 * std::cos(theta1);
    // float Xb_y = config_.L1 * std::sin(theta1);
    // float Xd_x = config_.L5 + config_.L4 * std::cos(theta4);
    // float Xd_y = config_.L4 * std::sin(theta4);

    // Calculate theta3 and theta2 using linkage kinematics
    float A = 2 * config_.L3 * config_.L4 * std::sin(theta4) - 2 * config_.L3 * config_.L1 * std::sin(theta1);
    float B = 2 * config_.L3 * config_.L5 - 2 * config_.L1 * config_.L3 * std::cos(theta1) + 2 * config_.L3 * config_.L4 * std::cos(theta4);
    float C = (std::pow(config_.L1, 2) - std::pow(config_.L2, 2) + std::pow(config_.L3, 2) + std::pow(config_.L4, 2) + std::pow(config_.L5, 2)
              - 2 * config_.L1 * config_.L4 * std::sin(theta1) * std::sin(theta4)
              - 2 * config_.L1 * config_.L5 * std::cos(theta1)
              + 2 * config_.L4 * config_.L5 * std::cos(theta4)
              - 2 * config_.L1 * config_.L4 * std::cos(theta1) * std::cos(theta4));

    float tan_x = A + std::sqrt(std::pow(A, 2) + std::pow(B, 2) - std::pow(C, 2));
    float tan_y = B - C;
    float theta3 = M_PI - 2 * std::atan2(tan_y, tan_x);

    float theta2 = std::asin((config_.L3 * std::sin(theta3) + config_.L4 * std::sin(theta4) - config_.L1 * std::sin(theta1)) / config_.L2);

    float pip_joint_angle_reduction = (theta3 - config_.eef_angle_offset - theta4) /theta1; 


    return pip_joint_angle_reduction;
}

} // namespace FiveBarLinkage