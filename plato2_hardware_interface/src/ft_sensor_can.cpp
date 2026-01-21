#include "plato2_hardware_interface/ft_sensor_can.hpp"
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>

FTSensorCAN::FTSensorCAN(const std::string& interface_name, uint32_t force_id, uint32_t torque_id)
    : if_name_(interface_name), force_id_(force_id), torque_id_(torque_id) {

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        throw std::runtime_error("Failed to create SocketCAN socket");
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        close(sock_);
        throw std::runtime_error("Interface not found: " + if_name_);
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Filter to only receive the two IDs this instance cares about
    struct can_filter rfilter[2];
    rfilter[0].can_id = force_id_;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = torque_id_;
    rfilter[1].can_mask = CAN_SFF_MASK;
    setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to bind SocketCAN");
    }

    running_.store(true);
    listener_thread_ = std::thread(&FTSensorCAN::listen, this);
}

FTSensorCAN::~FTSensorCAN() {
    running_.store(false);
    if (sock_ >= 0) {
        shutdown(sock_, SHUT_RDWR);
    }
    if (listener_thread_.joinable()) {
        listener_thread_.join();
    }
    if (sock_ >= 0) {
        close(sock_);
    }
}

void FTSensorCAN::set_callback(Callback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
}

void FTSensorCAN::listen() {
    struct can_frame frame;
    double fx = 0, fy = 0, fz = 0;
    double tx = 0, ty = 0, tz = 0;
    bool have_force = false;
    bool have_torque = false;

    while (running_.load()) {
        int nbytes = read(sock_, &frame, sizeof(struct can_frame));
        if (nbytes <= 0) {
            if (!running_.load()) break;
            continue;
        }

        uint16_t raw_x = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
        uint16_t raw_y = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
        uint16_t raw_z = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

        uint32_t id = frame.can_id & CAN_SFF_MASK;

        if (id == force_id_) {
            fx = convert_force(raw_x);
            fy = convert_force(raw_y);
            fz = convert_force(raw_z);
            have_force = true;
        } else if (id == torque_id_) {
            tx = convert_torque(raw_x);
            ty = convert_torque(raw_y);
            tz = convert_torque(raw_z);
            have_torque = true;
        }

        // Only trigger callback when we have a fresh set of both Force and Torque
        if (have_force && have_torque) {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (callback_) {
                callback_(fx, fy, fz, tx, ty, tz);
            }
            have_force = false;
            have_torque = false;
        }
    }
}