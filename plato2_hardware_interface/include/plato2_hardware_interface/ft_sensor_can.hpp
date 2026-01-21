#ifndef PLATO2_HARDWARE_INTERFACE_FT_SENSOR_CAN_HPP_
#define PLATO2_HARDWARE_INTERFACE_FT_SENSOR_CAN_HPP_

#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

class FTSensorCAN {
public:
    // Callback passes: fx, fy, fz, tx, ty, tz
    using Callback = std::function<void(double, double, double, double, double, double)>;

    /**
     * @param interface_name e.g., "can0"
     * @param force_id The CAN ID for Force (e.g., 0x02A)
     * @param torque_id The CAN ID for Torque (e.g., 0x02B)
     */
    FTSensorCAN(const std::string& interface_name, uint32_t force_id, uint32_t torque_id);
    ~FTSensorCAN();

    void set_callback(Callback cb);

private:
    void listen();
    
    // Conversion logic for AIDIN AFT20-D15
    inline double convert_force(uint16_t raw) { 
        // (Raw / 1000) - 30.0
        // 30,000 raw = 0.0N
        return (static_cast<double>(raw) / 1000.0) - 30.0; 
    }

    inline double convert_torque(uint16_t raw) { 
        // (Raw / 100,000) - 0.3
        // 30,000 raw = 0.0Nm
        return (static_cast<double>(raw) / 100000.0) - 0.3; 
    }

    int sock_{-1};
    std::string if_name_;
    uint32_t force_id_;
    uint32_t torque_id_;

    std::thread listener_thread_;
    std::atomic<bool> running_{false};
    
    Callback callback_{nullptr};
    std::mutex callback_mutex_;
};

#endif