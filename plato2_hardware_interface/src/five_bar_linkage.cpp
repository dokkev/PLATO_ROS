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
    float theta1 = (pip_motor_angle + config_.eef_linkage_angle); // PIP Motor Position
    float theta4 = mcp_motor_angle; // MCP Motor Position

    // print the motor angles
    // std::cout << "MCP Motor Angle: " << Plato::rad2deg(theta4) << "\n";
    // std::cout << "PIP Motor Angle: " << Plato::rad2deg(theta1) << "\n";

    // Calculate corner points
    float x1 = config_.L1 * std::cos(theta1);
    float y1 = config_.L1 * std::sin(theta1);
    float x4 = config_.L5 + config_.L4 * std::cos(theta4);
    float y4 = config_.L4 * std::sin(theta4);

    // Calculate distance between points
    float d = std::sqrt(std::pow(x4 - x1, 2) + std::pow(y4 - y1, 2));
    
    // Calculate intersection point using cosine law
    float a = (std::pow(config_.L2, 2) - std::pow(config_.L3, 2) + std::pow(d, 2)) / (2 * d);
    float h = std::sqrt(std::pow(config_.L2, 2) - std::pow(a, 2));
    
    // Calculate point 3 coordinates
    float x3 = x1 + a * (x4 - x1) / d - h * (y4 - y1) / d;
    float y3 = y1 + a * (y4 - y1) / d + h * (x4 - x1) / d;

    // Calculate theta3
    float theta3 = std::atan2(y3 - y4, x3 - x4);

    // Calculate PIP joint angle and reduction ratio
    float pip_joint_angle = theta3 - config_.eef_linkage_angle - theta4;
    float reduction_ratio = pip_joint_angle / pip_motor_angle;

    // print the calculated values
    // std::cout << "Output Linkage Angle: " << Plato::rad2deg(theta3) << "\n";
    // std::cout << "PIP Joint Angle: " << Plato::rad2deg(pip_joint_angle) << "\n";

    return reduction_ratio;
    // float dummy_ratio = (pip_motor_angle - mcp_motor_angle) / pip_motor_angle;
    // return dummy_ratio;
}

} // namespace FiveBarLinkage