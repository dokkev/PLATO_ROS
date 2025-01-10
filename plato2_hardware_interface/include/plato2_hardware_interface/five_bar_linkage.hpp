#ifndef PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_
#define PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_


#include "plato2_hardware_interface/utils/useful_functions.hpp"
#include "plato2_hardware_interface/hardware_config/linkage_config.hpp"


namespace FiveBarLinkage{

struct FiveBarLinkageConfig {
    const float L1;
    const float L2;
    const float L3;
    const float L4;
    const float L5;
    const float eef_linkage_angle;
    const float eef_length;

    FiveBarLinkageConfig() 
        : L1(FingerLinkage::L1)
        , L2(FingerLinkage::L2)
        , L3(FingerLinkage::L3)
        , L4(FingerLinkage::L4)
        , L5(FingerLinkage::L5)
        , eef_linkage_angle(Plato::deg2rad(FingerLinkage::PIP_MOTOR_ZERO_ANGLE_OFFSET))
        , eef_length(FingerLinkage::EEF_LENGTH)
    {}
};

class FiveBarLinkage{

public:
   /// @brief Constructor
    FiveBarLinkage(FiveBarLinkageConfig &config);

    /// @brief Destructor
    ~FiveBarLinkage();

    void update_kinematics(const float &mcp_motor_angle, const float &pip_motor_angle);

    float get_position_amplification(){ return {position_amplification_};};

    float get_torque_amplification(){ return {torque_amplification_};};


private:
    /// @brief linkage lengths and end-effector (distal) link length with the angle offset
    const FiveBarLinkageConfig config_;

    /// 
    float theta1_, theta2_, theta3_, theta4_;
    float position_amplification_;

    /// @brief  Torque Amplification Factor (Mechanical Advantage)
    float torque_amplification_;
     
};

}// namespace FiveBarLinkage




#endif // PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_