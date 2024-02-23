#include <iostream>
#include <thread>
#include <mutex>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <iomanip> 
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/ioctl.h>
#include <chrono>

using namespace std;

    


std::mutex can_mutex;

void read_can(int s) {
    struct can_frame frame;
    while (true) {
        {
            std::lock_guard<std::mutex> lock(can_mutex);
            int nbytes = read(s, &frame, sizeof(struct can_frame));
            if (nbytes > 0){
                

                std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                std::cout << "RX CAN ID: 0x" << std::uppercase << std::setfill('0') << std::setw(3) << 
                std::hex <<frame.can_id  << " [" << static_cast<int16_t>(frame.can_dlc) << "] "<<std::flush;

                // Successfully read a frame with expected DLC
                int id = frame.can_id;
                double data;
                memcpy(&data, frame.data, sizeof(double));

                std::cout << "Received CAN ID " << id << " message: " << data << std::endl;

                // std:: cout << start.time_since_epoch().count() << std::endl;

                //     for (int i = 0; i < frame.can_dlc; i++) {
                //     std::cout << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
                //     << static_cast<int>(frame.data[i]) << ' ';
                // }
            }
            
            
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
}
}


void write_can(int s) {
    struct can_frame frame;
    frame.can_id = 0x123;
    frame.can_dlc = 2;
    frame.data[0] = 0x11;
    frame.data[1] = 0x22;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(can_mutex);
            int nbytes = write(s, &frame, sizeof(struct can_frame));
            if (nbytes > 0) {
                std::cout << "TX CAN ID: 0x" << std::uppercase << std::setfill('0') << std::setw(3) << 
                std::hex <<frame.can_id  << " [" << static_cast<int16_t>(frame.can_dlc) << "] "<<std::flush;

                for (int i = 0; i < frame.can_dlc; i++) {
                    std::cout << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
                    << static_cast<int>(frame.data[i]) << ' ';
                }
                std::cout << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }   

            
        }
        
    }
}


int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    std::thread read_thread(read_can, s);
    // std::thread write_thread(write_can, s);

    read_thread.join();
    // write_thread.join();

    close(s);
    return 0;
}

