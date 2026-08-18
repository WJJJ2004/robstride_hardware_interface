#pragma once

#include <cstdint>
#include <vector>
#include <cmath>
#include <tuple>
#include <algorithm>

// 통신 타입 정의
namespace ProtocolCmd
{
    constexpr uint8_t MOTION_CONTROL = 0x01;
    constexpr uint8_t MOTOR_REQUEST  = 0x02;
    constexpr uint8_t MOTOR_ENABLE   = 0x03;
    constexpr uint8_t MOTOR_STOP     = 0x04;
}

class RobStrideProtocol {
public:
    // CAN ID 생성 (Type | MasterID | MotorID)
    static uint32_t generateCommandId(uint8_t type, uint8_t master_id, uint8_t motor_id);

    // 모터 ID 추출
    static uint8_t getMotorIdFromCanId(uint32_t can_id);

    // 통신 타입 추출
    static uint8_t getTypeFromCanId(uint32_t can_id);

    // Type-2 motor feedback status fields carried in the extended CAN ID.
    static uint8_t getRunStateFromCanId(uint32_t can_id);
    static uint8_t getFaultFlagsFromCanId(uint32_t can_id);

    // Float <-> Uint 변환 (SDK 핵심 로직)
    static uint16_t floatToUint(float x, float x_min, float x_max, int bits);

    // 명령 패킷 생성: Motion Control (MIT Mode)
    static std::vector<uint8_t> createMotionCommand(
        float p_des, float v_des, float kp, float kd, float t_ff,
        float p_min, float p_max, float v_min, float v_max, float kp_max, float kd_max, float t_max
    );

    // 명령 패킷 생성: Enable/Disable
    static std::vector<uint8_t> createEnableCommand();
    static std::vector<uint8_t> createDisableCommand();

    // 피드백 데이터 파싱
    // 리턴: <Position, Velocity, Torque, Temperature , Current>
    static std::tuple<float, float, float, float, float> parseFeedback(
        const std::vector<uint8_t>& data,
        float p_min, float p_max, float v_min, float v_max, float t_max
    );
};
