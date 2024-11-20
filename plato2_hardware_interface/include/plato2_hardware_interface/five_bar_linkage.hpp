#ifndef PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_
#define PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_


#include "plato2_hardware_interface/utils/useful_functions.hpp"


namespace FiveBarLinkage{

struct FiveBarLinkageConfig{
    const float L1;
    const float L2;
    const float L3;
    const float L4;
    const float L5;

    const float eef_linkage_angle;
    const float eef_length;
};

class FiveBarLinkage{

public:
   /// @brief Constructor
    FiveBarLinkage(const FiveBarLinkageConfig &config);

    /// @brief Destructor
    ~FiveBarLinkage();

    float update_kinematics(const float &mcp_motor_angle, const float &pip_motor_angle);


private:
    const FiveBarLinkageConfig config_;
     
};

}// namespace FiveBarLinkage




#endif // PLATO_HARDWARE_INTERFACE__FIVE_BAR_LINKAGE_HPP_