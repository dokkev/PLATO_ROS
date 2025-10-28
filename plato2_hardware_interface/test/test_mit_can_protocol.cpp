#include <gtest/gtest.h>
#include "plato2_hardware_interface/mit_can_protocol.hpp"
#include <cmath>

namespace mit_can_protocol {

class MITCANProtocolTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use typical motor parameters
        gear_ratio = 6.0f;
        torque_constant = 0.091f;
        tx_id = 0x01;
        
        encoder = std::make_unique<MsgEncoder>(gear_ratio, torque_constant, tx_id);
        decoder = std::make_unique<MsgDecoder>(gear_ratio, torque_constant);
        
        // Zero out message
        std::memset(&msg, 0, sizeof(msg));
    }

    float gear_ratio;
    float torque_constant;
    uint8_t tx_id;
    TPCANMsg msg;
    std::unique_ptr<MsgEncoder> encoder;
    std::unique_ptr<MsgDecoder> decoder;
};

// Test 1: Command message IDs are set correctly
TEST_F(MITCANProtocolTest, CommandMessageIDs) {
    encoder->start_motor(msg);
    EXPECT_EQ(msg.ID, tx_id | 0x400) << "Start motor should set OC bit (0x400)";
    
    encoder->stop_motor(msg);
    EXPECT_EQ(msg.ID, tx_id) << "Stop motor should use base ID";
    EXPECT_EQ(msg.DATA[0], 0xCF) << "Stop motor command byte should be 0xCF";
    
    encoder->set_zero_position(msg);
    EXPECT_EQ(msg.ID, tx_id) << "Set zero should use base ID";
    EXPECT_EQ(msg.DATA[0], 0xB1) << "Set zero command byte should be 0xB1";
}

// Test 2: Impedance command encoding with zero values
TEST_F(MITCANProtocolTest, ImpedanceCommandZeros) {
    encoder->set_impedance(msg, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    EXPECT_EQ(msg.ID, tx_id | 0x400) << "Impedance command should set OC bit";
    EXPECT_EQ(msg.LEN, 8) << "Impedance command should be 8 bytes";
    
    // All zeros should result in all data bytes being zero
    for (int i = 0; i < 8; ++i) {
        EXPECT_EQ(msg.DATA[i], 0) << "Byte " << i << " should be 0 for zero command";
    }
}

// Test 3: Impedance command encoding with non-zero values
TEST_F(MITCANProtocolTest, ImpedanceCommandNonZero) {
    float pos = 1.0f;    // 1 rad
    float vel = 2.0f;    // 2 rad/s
    float kp = 100.0f;   // Stiffness
    float kd = 1.0f;     // Damping
    float torque = 5.0f; // 5 Nm
    
    encoder->set_impedance(msg, pos, vel, kp, kd, torque);
    
    EXPECT_EQ(msg.ID, tx_id | 0x400) << "Should have OC bit set";
    EXPECT_EQ(msg.LEN, 8) << "Should be 8 bytes";
    
    // At least some bytes should be non-zero
    bool has_nonzero = false;
    for (int i = 0; i < 8; ++i) {
        if (msg.DATA[i] != 0) {
            has_nonzero = true;
            break;
        }
    }
    EXPECT_TRUE(has_nonzero) << "Non-zero command should have non-zero data bytes";
}

// Test 4: Limits configuration encoding
TEST_F(MITCANProtocolTest, LimitsConfiguration) {
    float pos_max = 10.0f;  // 10 rad
    float vel_max = 20.0f;  // 20 rad/s
    float tq_max = 15.0f;   // 15 Nm
    
    encoder->set_limits(msg, pos_max, vel_max, tq_max, true, true, true);
    
    EXPECT_EQ(msg.ID, tx_id) << "Limits command should use base ID (no OC bit)";
    EXPECT_EQ(msg.LEN, 7) << "Limits command should be 7 bytes";
    EXPECT_EQ(msg.DATA[0], 0xF0) << "Limits command byte should be 0xF0";
    
    // Extract the encoded values (little-endian, 16-bit)
    uint16_t pos_u16 = msg.DATA[1] | (uint16_t(msg.DATA[2]) << 8);
    uint16_t vel_u16 = msg.DATA[3] | (uint16_t(msg.DATA[4]) << 8);
    uint16_t tq_u16 = msg.DATA[5] | (uint16_t(msg.DATA[6]) << 8);
    
    // Position LSB = 0.1 rad, so 10 rad -> 100
    EXPECT_EQ(pos_u16, 100) << "Position max encoding: 10 rad / 0.1 = 100";
    
    // Velocity LSB = 0.01 rad/s, so 20 rad/s -> 2000
    EXPECT_EQ(vel_u16, 2000) << "Velocity max encoding: 20 rad/s / 0.01 = 2000";
    
    // Torque LSB = 0.01 Nm, so 15 Nm -> 1500
    EXPECT_EQ(tq_u16, 1500) << "Torque max encoding: 15 Nm / 0.01 = 1500";
}

// Test 5: Protocol constants validation
TEST_F(MITCANProtocolTest, ProtocolConstants) {
    EXPECT_FLOAT_EQ(KP_MAX, 500.0f) << "KP max should be 500";
    EXPECT_FLOAT_EQ(KD_MAX, 5.0f) << "KD max should be 5";
    EXPECT_FLOAT_EQ(POS_MAX, 95.5f) << "Position max should be 95.5 rad";
    EXPECT_FLOAT_EQ(VEL_MAX, 45.0f) << "Velocity max should be 45 rad/s";
    EXPECT_FLOAT_EQ(T_MAX, 18.0f) << "Torque max should be 18 Nm";
}

// Test 6: Message structure size and alignment
TEST_F(MITCANProtocolTest, MessageStructure) {
    encoder->set_impedance(msg, 1.0f, 1.0f, 10.0f, 1.0f, 1.0f);
    
    EXPECT_LE(msg.LEN, 8) << "Message length should not exceed 8 bytes";
    EXPECT_GT(msg.LEN, 0) << "Message length should be positive";
}

// Test 7: Saturation behavior - values within limits
TEST_F(MITCANProtocolTest, SaturationWithinLimits) {
    // Command values well within protocol limits
    float pos = 10.0f;   // << POS_MAX (95.5)
    float vel = 10.0f;   // << VEL_MAX (45.0)
    float kp = 100.0f;   // << KP_MAX (500.0)
    float kd = 2.0f;     // << KD_MAX (5.0)
    float tq = 5.0f;     // << T_MAX (18.0)
    
    encoder->set_impedance(msg, pos, vel, kp, kd, tq);
    
    // Should encode without errors
    EXPECT_EQ(msg.LEN, 8);
    EXPECT_EQ(msg.ID, tx_id | 0x400);
}

// Test 8: Negative values handling
TEST_F(MITCANProtocolTest, NegativeValues) {
    // Test with negative position, velocity, and torque
    float pos = -5.0f;
    float vel = -10.0f;
    float tq = -3.0f;
    float kp = 50.0f;  // Gains are always positive
    float kd = 1.0f;
    
    encoder->set_impedance(msg, pos, vel, kp, kd, tq);
    
    EXPECT_EQ(msg.LEN, 8);
    EXPECT_EQ(msg.ID, tx_id | 0x400);
    
    // Message should be successfully encoded with non-zero data
    bool has_data = false;
    for (int i = 0; i < 8; ++i) {
        if (msg.DATA[i] != 0) {
            has_data = true;
            break;
        }
    }
    EXPECT_TRUE(has_data) << "Negative values should produce valid encoding";
}

// Test 9: Encoder/Decoder round-trip consistency
TEST_F(MITCANProtocolTest, RoundTripConsistency) {
    // Encode a command
    float pos_cmd = 1.0f;
    float vel_cmd = 2.0f;
    float kp_cmd = 100.0f;
    float kd_cmd = 1.0f;
    float tq_cmd = 3.0f;
    
    encoder->set_impedance(msg, pos_cmd, vel_cmd, kp_cmd, kd_cmd, tq_cmd);
    
    // Decode the response (simulating motor echo)
    float pos_fb, vel_fb, kp_fb, kd_fb, tq_fb;
    bool in_oc_mode, has_fault;
    
    decoder->get_states(msg, pos_fb, vel_fb, kp_fb, kd_fb, tq_fb, in_oc_mode, has_fault);
    
    // The decoded values should be close to commanded (within quantization error)
    // Using larger tolerance due to fixed-point quantization
    EXPECT_NEAR(pos_fb, pos_cmd, 0.01f) << "Position round-trip";
    EXPECT_NEAR(vel_fb, vel_cmd, 0.1f) << "Velocity round-trip";
    EXPECT_NEAR(kp_fb, kp_cmd, 1.0f) << "Kp round-trip";
    EXPECT_NEAR(kd_fb, kd_cmd, 0.01f) << "Kd round-trip";
    EXPECT_NEAR(tq_fb, tq_cmd, 0.1f) << "Torque round-trip";
}

// Test 10: Multiple TX IDs
TEST_F(MITCANProtocolTest, MultipleTXIDs) {
    uint8_t id1 = 0x01;
    uint8_t id2 = 0x0F;
    
    MsgEncoder enc1(gear_ratio, torque_constant, id1);
    MsgEncoder enc2(gear_ratio, torque_constant, id2);
    
    TPCANMsg msg1, msg2;
    enc1.set_impedance(msg1, 1.0f, 1.0f, 10.0f, 1.0f, 1.0f);
    enc2.set_impedance(msg2, 1.0f, 1.0f, 10.0f, 1.0f, 1.0f);
    
    EXPECT_EQ(msg1.ID, id1 | 0x400);
    EXPECT_EQ(msg2.ID, id2 | 0x400);
    EXPECT_NE(msg1.ID, msg2.ID) << "Different TX IDs should produce different CAN IDs";
}

} // namespace mit_can_protocol

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
