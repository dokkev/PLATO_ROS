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

void FiveBarLinkage::update_kinematics(const float &mcp_motor_angle, const float &pip_motor_angle) {
    theta1_ = pip_motor_angle + config_.eef_linkage_angle; // PIP Motor Position
    theta4_ = mcp_motor_angle; // MCP Motor Position

    // Pre-compute sine and cosine terms for efficiency
    float s_theta1 = std::sin(theta1_);
    float c_theta1 = std::cos(theta1_);
    float s_theta4 = std::sin(theta4_);
    float c_theta4 = std::cos(theta4_);

    // Compute A, B, and C terms
    float A = 2 * config_.L3 * config_.L4 * s_theta4 - 2 * config_.L3 * config_.L1 * s_theta1;
    float B = 2 * config_.L3 * config_.L5 - 2 * config_.L1 * config_.L3 * c_theta1 + 2 * config_.L3 * config_.L4 * c_theta4;
    float C = std::pow(config_.L1, 2) - std::pow(config_.L2, 2) + std::pow(config_.L3, 2) + std::pow(config_.L4, 2) + std::pow(config_.L5, 2) 
              - 2 * config_.L1 * config_.L4 * s_theta1 * s_theta4 
              - 2 * config_.L1 * config_.L5 * c_theta1 
              + 2 * config_.L4 * config_.L5 * c_theta4 
              - 2 * config_.L1 * config_.L4 * c_theta1 * c_theta4;

    // Check for invalid configuration
    if (std::pow(A, 2) + std::pow(B, 2) - std::pow(C, 2) < 0) {
        std::cerr << "Invalid configuration. A^2 + B^2 - C^2 < 0\n";
        position_amplification_ = 1.0;
        torque_amplification_ = 1.0;
    }

    // Calculate tan_x and tan_y
    float tan_x = A + std::sqrt(std::pow(A, 2) + std::pow(B, 2) - std::pow(C, 2));
    float tan_y = B - C;

    // Calculate theta3 using arctangent
    float theta3_ = M_PI - 2 * std::atan2(tan_y, tan_x);
    float s_theta3_ = std::sin(theta3_);

    // Calculate theta2 (PIP joint angle)
    float theta2_ = std::asin((config_.L3 * s_theta3_ + config_.L4 * s_theta4 - config_.L1 * s_theta1) / config_.L2);

    // Calculate PIP joint angle and reduction ratio
    position_amplification_ = (theta3_ - config_.eef_linkage_angle - theta4_) /  pip_motor_angle;
    // position_amplification_ = 1.0;

    torque_amplification_ = std::abs( (-config_.L3 * std::sin(theta2_ - theta3_) )  / (config_.L1 * std::sin(theta1_ - theta2_)) );
   

    

}

} // namespace FiveBarLinkage