#ifndef PLATO2_HARDWARE_INTERFACE_FT_SENSOR_CAN_HPP_
#define PLATO2_HARDWARE_INTERFACE_FT_SENSOR_CAN_HPP_

#include <iostream>
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <functional>

#define FORCE_CAN_ID 0x01A
#define TORQUE_CAN_ID 0x01B

// Convert raw force output to Newtons (D15 Model)
inline double convert_force(uint16_t raw) {
    return (static_cast<double>(raw) / 1000.0) - 30.0;
}

// Convert raw torque output to Nm (D15 Model)
inline double convert_torque(uint16_t raw) {
    return (static_cast<double>(raw) / 100000.0) - 0.3;
}

class FTSensorCAN {
public:
    using Callback = std::function<void(double, double, double, double, double, double)>;

    explicit FTSensorCAN(const std::string& interface_name);
    ~FTSensorCAN();
    
    void set_callback(Callback cb);

private:
    int sock_;
    struct sockaddr_can addr_;
    struct ifreq ifr_;
    std::string if_name_;
    std::thread listener_thread_;
    Callback callback_;
    bool running_;

    void listen();
};

#endif // PLATO2_HARDWARE_INTERFACE_FT_SENSOR_CAN_HPP_
