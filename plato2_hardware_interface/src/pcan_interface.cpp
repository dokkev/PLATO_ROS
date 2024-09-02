#include "plato2_hardware_interface/pcan_interface.hpp"

#include <iostream>

namespace pcan_interface {

PCANInterface::PCANInterface() {
    
    // Show PCAN Configuration
    std::cout << "Initiliazing CAN..." << std::endl;

    // Initialize PCAN
    TPCANStatus status_result;
    status_result = CAN_Initialize(pcan_handle, bitrate);
    

    if (status_result != PCAN_ERROR_OK) {
        std::cout << "Error initializing CAN" << std::endl;
        show_status(status_result);
        return;
    }
    std::cout << "...CAN Initialized" << std::endl;

}

PCANInterface::~PCANInterface() {
    // Shutdown PCAN
    std::cout << "Shutting Down CAN..." << std::endl;
    CAN_Uninitialize(PCAN_NONEBUS);
}   


TPCANStatus PCANInterface::write_message(TPCANMsg* msg) {

    return CAN_Write(pcan_handle, msg);
}




} // namespace pcan_interface