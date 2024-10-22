#ifndef PLATO_HARDWARE_INTERFACE__LINKAGE_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__LINKAGE_CONFIG_HPP_


// TODO: Change this to yaml file

namespace PlatoLinkage{
    // Units: mm
    constexpr double L1 = 30.0f; // PIP Driving (Input) Linkage
    constexpr double L2 = 104.0f; // PIP Coupler Linkage
    constexpr double L3 = 30.0f; // PIP Joint (Output) Linkage
    constexpr double L4 = 60.0f; // MCP Linkage (Input=Output) 
    constexpr double L5 = 44.0f; // MCP motor to PIP motor Distance

    
    // Respective to the xy origin, the PIP Motor angle is at 135 deg when the PIP Joint and MCP Joint are at 0 deg.
    // However, PIP Motor angles are zeroed at the position where PIP Joint is at its 0 degree position (parallel proximal phalange)
    // making inconsistency in between MCP Motor Angles and PIP Motor Angle coordinate system. 
    // Therefore, we apply an offset to the PIP Motor angle to make it consistent  with the PIP Joint angle for Five Bar Linkage kinematics calculation.
    // TODO: Use coordinate transformation to avoid confusion
    constexpr double PIP_MOTOR_ZERO_ANGLE_OFFSET = 135.0f; // deg
}

#endif // PLATO_HARDWARE_INTERFACE__LINKAGE_CONFIG_HPP_