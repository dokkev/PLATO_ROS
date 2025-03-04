#include "plato2_hardware_interface/ft_sensor_can.hpp"
#include <cstring> 

FTSensorCAN::FTSensorCAN(const std::string& interface_name) : if_name_(interface_name), running_(true) {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        perror("Socket creation failed");
        exit(1);
    }

    std::strcpy(ifr_.ifr_name, if_name_.c_str());
    ioctl(sock_, SIOCGIFINDEX, &ifr_);

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        perror("Bind failed");
        exit(1);
    }

    listener_thread_ = std::thread(&FTSensorCAN::listen, this);
}

FTSensorCAN::~FTSensorCAN() {
    running_ = false;
    if (listener_thread_.joinable()) {
        listener_thread_.join();
    }
    close(sock_);
}

void FTSensorCAN::set_callback(Callback cb) {
    callback_ = std::move(cb);
}

void FTSensorCAN::listen() {
    struct can_frame frame;
    double fx = 0, fy = 0, fz = 0;
    double tx = 0, ty = 0, tz = 0;

    while (running_) {
        int nbytes = read(sock_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            uint16_t x = (frame.data[0] << 8) | frame.data[1];
            uint16_t y = (frame.data[2] << 8) | frame.data[3];
            uint16_t z = (frame.data[4] << 8) | frame.data[5];

            if (frame.can_id == FORCE_CAN_ID) {
                fx = convert_force(x);
                fy = convert_force(y);
                fz = convert_force(z);
            } else if (frame.can_id == TORQUE_CAN_ID) {
                tx = convert_torque(x);
                ty = convert_torque(y);
                tz = convert_torque(z);
            }

            // ✅ Always call callback with latest force & torque data
            callback_(fx, fy, fz, tx, ty, tz);
        }
    }
}
