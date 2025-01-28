#ifndef FT_SENSOR_CAN_HPP_
#define FT_SENSOR_CAN_HPP_

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>  // Added for read() and close()
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <string>
#include <errno.h>  // Added for errno

class FTSensorCAN {
public:
    // Struct to hold 6-axis force/torque data
    struct Wrench {
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
        uint64_t timestamp;  // Microseconds since start
    };

    // CAN IDs for force and torque messages
    static constexpr canid_t FORCE_MSG_ID = 0x01A;
    static constexpr canid_t TORQUE_MSG_ID = 0x01B;

    FTSensorCAN(const std::string& interface = "can0") 
        : interface_(interface), running_(false) {}
    
    bool init() {
        // Create CAN socket
        socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_ < 0) {
            return false;
        }

        // Set socket to non-blocking mode
        int flags = fcntl(socket_, F_GETFL, 0);
        if (fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
            return false;
        }

        // Specify CAN interface
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_.c_str());
        if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
            return false;
        }

        // Bind to the CAN interface
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            return false;
        }

        // Set up filter for force and torque message IDs
        struct can_filter filter[2];
        filter[0].can_id = FORCE_MSG_ID;
        filter[0].can_mask = CAN_SFF_MASK;
        filter[1].can_id = TORQUE_MSG_ID;
        filter[1].can_mask = CAN_SFF_MASK;

        if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
            return false;
        }

        // Start the reading thread
        running_ = true;
        read_thread_ = std::thread(&FTSensorCAN::readLoop, this);

        return true;
    }

    // Get the latest wrench data without blocking
    Wrench getLatestWrench() {
        std::lock_guard<std::mutex> lock(wrench_mutex_);
        return latest_wrench_;
    }

    // Get the latest wrench data with timeout
    bool getLatestWrench(Wrench& wrench, uint64_t max_age_us = 10000) {
        std::lock_guard<std::mutex> lock(wrench_mutex_);
        
        // Get current timestamp
        auto now = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        // Check if data is fresh enough
        if (now - latest_wrench_.timestamp <= max_age_us) {
            wrench = latest_wrench_;
            return true;
        }
        return false;
    }

    ~FTSensorCAN() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (socket_ >= 0) {
            close(socket_);
        }
    }

private:
    int socket_ = -1;
    std::string interface_;
    std::thread read_thread_;
    std::atomic<bool> running_;
    std::mutex wrench_mutex_;
    Wrench latest_wrench_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};
    bool have_force_ = false;
    bool have_torque_ = false;

    void readLoop() {
        struct can_frame frame;
        Wrench current_wrench = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};

        while (running_) {
            // Try to read a CAN frame (non-blocking)
            int nbytes = read(socket_, &frame, sizeof(frame));
            
            if (nbytes == sizeof(frame)) {
                if (frame.can_id == FORCE_MSG_ID) {
                    // Decode forces according to AFT20-D15 model formula:
                    // Force [N] = Force output /1000 - 30
                    double force_scale = 1000.0;
                    double force_offset = 30.0;

                    uint16_t fx_raw = (frame.data[0] | (frame.data[1] << 8));
                    uint16_t fy_raw = (frame.data[2] | (frame.data[3] << 8));
                    uint16_t fz_raw = (frame.data[4] | (frame.data[5] << 8));

                    current_wrench.fx = (fx_raw / force_scale) - force_offset;
                    current_wrench.fy = (fy_raw / force_scale) - force_offset;
                    current_wrench.fz = (fz_raw / force_scale) - force_offset;
                    have_force_ = true;
                }
                else if (frame.can_id == TORQUE_MSG_ID) {
                    // Decode torques according to AFT20-D15 model formula:
                    // Torque [Nm] = Torque output /100,000 - 0.3
                    double torque_scale = 100000.0;
                    double torque_offset = 0.3;

                    uint16_t tx_raw = (frame.data[0] | (frame.data[1] << 8));
                    uint16_t ty_raw = (frame.data[2] | (frame.data[3] << 8));
                    uint16_t tz_raw = (frame.data[4] | (frame.data[5] << 8));

                    current_wrench.tx = (tx_raw / torque_scale) - torque_offset;
                    current_wrench.ty = (ty_raw / torque_scale) - torque_offset;
                    current_wrench.tz = (tz_raw / torque_scale) - torque_offset;
                    have_torque_ = true;
                }

                // If we have both force and torque data, update the latest wrench
                if (have_force_ && have_torque_) {
                    current_wrench.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    
                    {
                        std::lock_guard<std::mutex> lock(wrench_mutex_);
                        latest_wrench_ = current_wrench;
                    }
                    
                    have_force_ = false;
                    have_torque_ = false;
                }
            }
            else if (nbytes < 0 && errno != EAGAIN) {
                // Handle error (except would-block error)
                // You might want to add error handling here
            }

            // Small sleep to prevent busy-waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));  // 100us sleep
        }
    }
};

#endif // FT_SENSOR_CAN_HPP_