#include <plato2_hardware_interface/plato2_hand.hpp>
#include <plato2_hardware_interface/hardware_config/actuator_config.hpp>

namespace plato2_hand{

Hand::Hand() : control_mode_(ControlMode::OFF){
    // Initialize the PCAN Interface
    // pcan_interface_ = pcan_interface::PCANInterface();

    // Initialize the Actuators
    init_actuators();
}

////////////////////////////////////////////////////////////////////////

Hand::~Hand(){
    // Stop the motion control
    stop();
    
}

////////////////////////////////////////////////////////////////////////

void Hand::init_actuators(){
    // Get actuator configs 
    auto actuator_configs = PlatoV2Config::init_actuator_configs();

    // Initialize the actuators
    for (size_t i = 0; i < actuator_configs.size(); ++i) {
        actuators_.emplace_back(pcan_interface_, actuator_configs[i]);
    }
}

////////////////////////////////////////////////////////////////////////

} // namespace plato2_hand