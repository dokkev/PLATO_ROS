// Simple CAN verification tool: requests MIT states (0xF1) and prints replies
#include <chrono>
#include <cstdint>
#include <functional>
#include <atomic>
#include <iostream>
#include <thread>

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/mit_can_protocol.hpp"
#include <PCANBasic.h>

using namespace std::chrono_literals;

static void usage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " <motor_tx_id_hex> [duration_sec] [--sweep] [--id-sweep]\n"
              << "  motor_tx_id_hex: e.g., 0x10\n"
              << "  duration_sec: seconds to attempt at a found bitrate (default 3)\n"
              << "  --sweep: try multiple bitrates until replies are observed\n"
              << "  --id-sweep: scan IDs 0..255 for 0xF1 replies (uses PCANBasic directly)\n"
              << "  --sweep + --id-sweep: try multiple bitrates and sweep IDs at each rate\n"
              << "Examples:\n"
              << "  " << argv0 << " 0x10 3 --sweep\n"
              << "  " << argv0 << " 0x00 2 --id-sweep\n"
              << "  " << argv0 << " 0x00 1 --sweep --id-sweep\n";
}

static const char* baud_to_str(TPCANBaudrate b) {
    switch (b) {
        case PCAN_BAUD_1M: return "1M"; case PCAN_BAUD_800K: return "800k"; case PCAN_BAUD_500K: return "500k";
        case PCAN_BAUD_250K: return "250k"; case PCAN_BAUD_125K: return "125k"; case PCAN_BAUD_100K: return "100k";
        case PCAN_BAUD_50K: return "50k"; case PCAN_BAUD_47K: return "47.6k"; case PCAN_BAUD_33K: return "33.3k";
        case PCAN_BAUD_20K: return "20k"; case PCAN_BAUD_10K: return "10k"; case PCAN_BAUD_5K: return "5k";
        default: return "unknown";
    }
}

struct SweepResult { bool success; TPCANBaudrate baud; };

static SweepResult try_sweep_bitrates(uint32_t id, double per_rate_sec) {
    // Candidate bitrates (most likely first)
    const TPCANBaudrate rates[] = {
        PCAN_BAUD_1M, PCAN_BAUD_500K, PCAN_BAUD_250K, PCAN_BAUD_125K,
        PCAN_BAUD_800K, PCAN_BAUD_100K, PCAN_BAUD_50K, PCAN_BAUD_33K, PCAN_BAUD_20K, PCAN_BAUD_10K
    };

    // Build request once
    TPCANMsg req{}; req.ID = id; req.LEN = 1; req.DATA[0] = 0xF1;

    for (auto b : rates) {
        // Reset and initialize PCAN
        CAN_Uninitialize(PCAN_NONEBUS);
        auto status = CAN_Initialize(PCAN_PCIBUS1, b);
        if (status != PCAN_ERROR_OK) {
            continue; // try next rate
        }

        std::cout << "Trying bitrate " << baud_to_str(b) << "...\n";
        const auto t0 = std::chrono::steady_clock::now();
        auto next_tx = t0;
        bool got_reply = false;
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < per_rate_sec) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_tx) {
                CAN_Write(PCAN_PCIBUS1, &req);
                next_tx = now + std::chrono::milliseconds(20);
            }
            // Drain RX
            TPCANMsg rx{}; TPCANTimestamp ts{};
            auto r = CAN_Read(PCAN_PCIBUS1, &rx, &ts);
            if (r == PCAN_ERROR_OK && rx.LEN >= 1 && rx.DATA[0] == 0xF1) {
                std::cout << "Observed 0xF1 reply at " << baud_to_str(b) << " (ID 0x" << std::hex << rx.ID << std::dec << ")\n";
                got_reply = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (got_reply) {
            return {true, b};
        }
    }
    return {false, PCAN_BAUD_1M};
}

struct FoundID { uint8_t requested; uint32_t replied; };

static std::vector<FoundID> sweep_ids_0_255(double per_id_ms) {
    std::vector<FoundID> found;

    // Drain any pending RX
    {
        TPCANMsg rx{}; TPCANTimestamp ts{};
        while (CAN_Read(PCAN_PCIBUS1, &rx, &ts) == PCAN_ERROR_OK) {}
    }

    for (int req_id = 0; req_id <= 255; ++req_id) {
        TPCANMsg req{}; req.ID = static_cast<uint32_t>(req_id); req.LEN = 1; req.DATA[0] = 0xF1;

        const auto t_start = std::chrono::steady_clock::now();
        auto next_tx = t_start; // send immediately
        bool got = false;
        while (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count() < per_id_ms) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_tx) {
                (void)CAN_Write(PCAN_PCIBUS1, &req);
                next_tx = now + std::chrono::milliseconds(10);
            }
            // Read all available frames quickly
            for (int i = 0; i < 4; ++i) {
                TPCANMsg rx{}; TPCANTimestamp ts{};
                auto r = CAN_Read(PCAN_PCIBUS1, &rx, &ts);
                if (r != PCAN_ERROR_OK) break;
                if (rx.LEN >= 1 && rx.DATA[0] == 0xF1) {
                    found.push_back(FoundID{static_cast<uint8_t>(req_id), rx.ID});
                    std::cout << "ID sweep: request 0x" << std::hex << req_id << ", reply ID 0x" << rx.ID << std::dec << "\n";
                    got = true;
                    break;
                }
            }
            if (got) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return found;
}

static bool sweep_all_rates_and_ids(double per_id_ms) {
    const TPCANBaudrate rates[] = {
        PCAN_BAUD_1M, PCAN_BAUD_500K, PCAN_BAUD_250K, PCAN_BAUD_125K,
        PCAN_BAUD_800K, PCAN_BAUD_100K, PCAN_BAUD_50K, PCAN_BAUD_33K, PCAN_BAUD_20K, PCAN_BAUD_10K
    };

    bool any_found = false;
    for (auto b : rates) {
        CAN_Uninitialize(PCAN_NONEBUS);
        auto st = CAN_Initialize(PCAN_PCIBUS1, b);
        if (st != PCAN_ERROR_OK) {
            continue;
        }
        std::cout << "Sweeping IDs at bitrate " << baud_to_str(b) << "...\n";
        auto found = sweep_ids_0_255(per_id_ms);
        if (!found.empty()) {
            any_found = true;
            std::cout << "  Found " << found.size() << " responders at " << baud_to_str(b) << ":\n";
            for (const auto& f : found) {
                std::cout << "    req 0x" << std::hex << int(f.requested) << ", reply 0x" << f.replied << std::dec << "\n";
            }
        }
    }
    return any_found;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    // Parse motor TX ID
    uint32_t id = 0;
    try {
        std::string s(argv[1]);
        id = static_cast<uint32_t>(std::stoul(s, nullptr, 0));
    } catch (...) {
        std::cerr << "Invalid motor_tx_id: " << argv[1] << "\n";
        return 1;
    }
    const double dur_sec = (argc >= 3) ? std::atof(argv[2]) : 3.0;
    bool sweep = false;
    bool id_sweep = false;
    for (int i = 3; i < argc; ++i) {
        if (std::string(argv[i]) == "--sweep") sweep = true;
        if (std::string(argv[i]) == "--id-sweep") id_sweep = true;
    }
    if (sweep && id_sweep) {
        // Full matrix: sweep all bitrates and sweep IDs at each
        if (sweep_all_rates_and_ids(15.0)) return 0;
        std::cerr << "No responders found across tested bitrates and IDs.\n";
        return 4;
    } else if (sweep) {
        // First, sweep to find a working bitrate quickly
        auto res = try_sweep_bitrates(id, std::min(1.0, dur_sec));
        if (!res.success) {
            std::cerr << "No replies observed at tested bitrates.\n";
            return 3;
        }
        // Keep the working bitrate initialized and run single-ID decode
            std::cout << "Using bitrate: " << baud_to_str(res.baud) << ". Listening for up to " << dur_sec << " s...\n";
            mit_can_protocol::MsgDecoder decoder(1.0f, 1.0f);
            const auto t0 = std::chrono::steady_clock::now();
            auto next_tx = t0;
            bool got = false;
            TPCANMsg req{}; req.ID = id; req.LEN = 1; req.DATA[0] = 0xF1;
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < dur_sec) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_tx) { CAN_Write(PCAN_PCIBUS1, &req); next_tx = now + 20ms; }
                TPCANMsg rx{}; TPCANTimestamp ts{}; auto r = CAN_Read(PCAN_PCIBUS1, &rx, &ts);
                if (r == PCAN_ERROR_OK && rx.LEN >= 7 && rx.DATA[0] == 0xF1) {
                    float p=0, v=0, kp=0, kd=0, tq=0; bool in_oc=false, fault=false;
                    decoder.get_states(rx, p, v, kp, kd, tq, in_oc, fault);
                    std::cout << "RX 0xF1 from ID 0x" << std::hex << rx.ID << std::dec
                              << " | pos=" << p << ", vel=" << v << ", tq=" << tq
                              << ", oc=" << in_oc << ", fault=" << fault << "\n";
                    got = true;
                }
                std::this_thread::sleep_for(1ms);
            }
            return got ? 0 : 2;
    }

    // Default path: use existing wrapper (1M default in PCANInterface)
    if (id_sweep) {
        // Use PCANBasic directly at 1M and sweep IDs
        CAN_Uninitialize(PCAN_NONEBUS);
        if (CAN_Initialize(PCAN_PCIBUS1, PCAN_BAUD_1M) != PCAN_ERROR_OK) {
            std::cerr << "Failed to initialize PCAN at 1M for ID sweep.\n";
            return 5;
        }
        auto found = sweep_ids_0_255(15.0);
        if (found.empty()) {
            std::cerr << "ID sweep: no 0xF1 replies observed at 1M.\n";
            // return 4;
        }
        std::cout << "ID sweep complete. Found " << found.size() << " responders.\n";
        return 0;
    }

    pcan_interface::PCANInterface can;

    // Decoder with generic constants (gear_ratio, torque_constant not needed for scaling here)
    mit_can_protocol::MsgDecoder decoder(1.0f, 1.0f);

    std::atomic<bool> got_reply{false};

    // Install a simple read callback to look for 0xF1 state frames
    can.set_read_callback([&](const TPCANMsg& msg) {
        if (msg.LEN >= 7 && msg.DATA[0] == 0xF1) {
            float p=0, v=0, kp=0, kd=0, tq=0; bool in_oc=false, fault=false;
            decoder.get_states(msg, p, v, kp, kd, tq, in_oc, fault);
            std::cout << "RX 0xF1 from ID 0x" << std::hex << msg.ID << std::dec
                      << " | pos=" << p << " rad, vel=" << v << " rad/s, tq=" << tq
                      << ", oc=" << in_oc << ", fault=" << fault << "\n";
            got_reply = true;
        }
    });

    // Build a READ_STATES request (protocol command 0xF1, std frame)
    TPCANMsg req{};
    req.ID = static_cast<uint32_t>(id);
    req.LEN = 1;
    req.DATA[0] = 0xF1; // CMD_READ_STATES per MIT mapping
    std::cout << "Sending 0xF1 requests to TX ID 0x" << std::hex << id << std::dec
              << " for " << dur_sec << " seconds... (no sweep)\n";

    // Send a few requests spaced out and poll for responses
    const auto t0 = std::chrono::steady_clock::now();
    auto next_tx = t0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < dur_sec) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_tx) {
            can.send_message(req);
            next_tx = now + 20ms; // 50 Hz queries
        }
        // Drain RX queue
        can.receive_message();
        std::this_thread::sleep_for(1ms);
    }

    if (!got_reply.load()) {
        std::cerr << "No 0xF1 state replies observed. Check motor power, CAN bitrate, IDs, and OC mode.\n";
        return 2;
    }

    return 0;
}
