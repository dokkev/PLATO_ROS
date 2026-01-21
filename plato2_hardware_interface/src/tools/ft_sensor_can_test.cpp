#include "plato2_hardware_interface/ft_sensor_can.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

struct FTData {
    double fx{0.0}, fy{0.0}, fz{0.0}, tx{0.0}, ty{0.0}, tz{0.0};
    double off_fx{0.0}, off_fy{0.0}, off_fz{0.0}, off_tx{0.0}, off_ty{0.0}, off_tz{0.0};
    std::mutex mtx;
};

// Non-blocking keyboard check
bool kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) {
        return (ch == 't');
    }
    return false;
}

void print_triplet(double x, double y, double z, int precision) {
    std::cout << std::fixed << std::setprecision(precision) 
              << std::setw(7) << x << "," 
              << std::setw(7) << y << "," 
              << std::setw(7) << z;
}

int main() {
    FTData d1, d2;
    std::atomic<bool> is_calibrating{true};
    std::vector<std::vector<double>> s1_samples, s2_samples;
    std::mutex samples_mtx;

    auto callback_factory = [&](FTData& data, std::vector<std::vector<double>>& samples) {
        return [&data, &samples, &is_calibrating, &samples_mtx](double fx, double fy, double fz, double tx, double ty, double tz) {
            if (is_calibrating) {
                std::lock_guard<std::mutex> s_lock(samples_mtx);
                samples.push_back({fx, fy, fz, tx, ty, tz});
            }
            std::lock_guard<std::mutex> d_lock(data.mtx);
            data.fx = fx - data.off_fx; data.fy = fy - data.off_fy; data.fz = fz - data.off_fz;
            data.tx = tx - data.off_tx; data.ty = ty - data.off_ty; data.tz = tz - data.off_tz;
        };
    };

    FTSensorCAN ft1("can0", 0x02A, 0x02B);
    ft1.set_callback(callback_factory(d1, s1_samples));
    FTSensorCAN ft2("can0", 0x03A, 0x03B);
    ft2.set_callback(callback_factory(d2, s2_samples));

    auto perform_tare = [&](int seconds) {
        is_calibrating = true;
        {
            std::lock_guard<std::mutex> s_lock(samples_mtx);
            s1_samples.clear(); s2_samples.clear();
        }
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        is_calibrating = false;

        std::lock_guard<std::mutex> s_lock(samples_mtx);
        auto calc_off = [](FTData& d, std::vector<std::vector<double>>& s) {
            if (s.empty()) return;
            double sum[6] = {0,0,0,0,0,0};
            for (auto& row : s) for (int i=0; i<6; ++i) sum[i] += row[i];
            std::lock_guard<std::mutex> d_lock(d.mtx);
            d.off_fx = sum[0]/s.size(); d.off_fy = sum[1]/s.size(); d.off_fz = sum[2]/s.size();
            d.off_tx = sum[3]/s.size(); d.off_ty = sum[4]/s.size(); d.off_tz = sum[5]/s.size();
        };
        calc_off(d1, s1_samples);
        calc_off(d2, s2_samples);
    };

    // Initial Tare
    std::cout << "Initial Calibration... Keep Still." << std::endl;
    perform_tare(2);

    std::cout << "\033[2J\033[H"; // Clear
    while (true) {
        if (kbhit()) {
            std::cout << "\r\033[K[ Taring... ]" << std::flush;
            perform_tare(1);
        }

        double s1[6], s2[6];
        { std::lock_guard<std::mutex> l1(d1.mtx); 
          s1[0]=d1.fx; s1[1]=d1.fy; s1[2]=d1.fz; s1[3]=d1.tx; s1[4]=d1.ty; s1[5]=d1.tz; }
        { std::lock_guard<std::mutex> l2(d2.mtx); 
          s2[0]=d2.fx; s2[1]=d2.fy; s2[2]=d2.fz; s2[3]=d2.tx; s2[4]=d2.ty; s2[5]=d2.tz; }

        std::cout << "\033[H"; // Move to top
        std::cout << "--- Dual FT Sensor Monitor ---" << std::endl;
        std::cout << "S1: \033[1;32mGreen\033[0m | S2: \033[1;34mBlue\033[0m | \033[1;33mPress 't' to Tare\033[0m" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        
        std::cout << "\r\033[K\033[1;32m";
        print_triplet(s1[0], s1[1], s1[2], 2); std::cout << " | "; print_triplet(s1[3], s1[4], s1[5], 3);
        std::cout << "\033[0m  ||  \033[1;34m";
        print_triplet(s2[0], s2[1], s2[2], 2); std::cout << " | "; print_triplet(s2[3], s2[4], s2[5], 3);
        std::cout << "\033[0m" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}