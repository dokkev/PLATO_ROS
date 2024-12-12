#include "plato2_hardware_interface/five_bar_linkage.hpp"
#include "plato2_hardware_interface/hardware_config/linkage_config.hpp"
#include <iostream>

namespace FiveBarLinkage{

FiveBarLinkage::FiveBarLinkage(FiveBarLinkageConfig &config) : config_(config){

    // print the linkage configuration
    std::cout << "\nInitializing FiveBarLinkage with configuration:\n";
    std::cout << "Passed values:\n";
    std::cout << "L1: " << config_.L1 << "\n";
    std::cout << "L2: " << config_.L2 << "\n";
    std::cout << "L3: " << config_.L3 << "\n";
    std::cout << "L4: " << config_.L4 << "\n";
    std::cout << "L5: " << config_.L5 << "\n";
    std::cout << "eef_length: " << config.eef_length << "\n";
    std::cout << "eef_linkage_angle: " << config.eef_linkage_angle << "\n";


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

    float pip_joint_angle = theta3 - config_.eef_linkage_angle - theta4;

    float reduction_ratio = pip_joint_angle / pip_motor_angle;

    // added dummy_ration back for testing
    float dummy_ratio = (pip_motor_angle - mcp_motor_angle) / pip_motor_angle;
    return reduction_ratio;

    // return reduction_ratio;
}

} // namespace FiveBarLinkage