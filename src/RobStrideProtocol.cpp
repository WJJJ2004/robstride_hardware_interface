#include "robstride_rdk_ros2/RobStrideProtocol.hpp"

/*
    [Type 8bit] [Ext 0] [Master 8bit] [Motor 8bit]
    24~31       16~23   8~15          0~7
*/

uint32_t RobStrideProtocol::generateCommandId(uint8_t type, uint8_t master_id, uint8_t motor_id)
{
    return (static_cast<uint32_t>(type) << 24) |
           (static_cast<uint32_t>(master_id) << 8) |
           static_cast<uint32_t>(motor_id);
}

uint8_t RobStrideProtocol::getMotorIdFromCanId(uint32_t can_id)
{
    // Motor ID는 bits 8-15
    return static_cast<uint8_t>((can_id >> 8) & 0xFF);
}

uint8_t RobStrideProtocol::getTypeFromCanId(uint32_t can_id)
{
    // Type은 bits 24-31
    return static_cast<uint8_t>((can_id >> 24) & 0xFF);
}

uint16_t RobStrideProtocol::floatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;

    return static_cast<uint16_t>((x - offset) * ((static_cast<float>(1 << bits) - 1)) / span);
}

std::vector<uint8_t> RobStrideProtocol::createMotionCommand(
    float p_des, float v_des, float kp, float kd, float t_ff,
    float p_min, float p_max, float v_min, float v_max, float kp_max, float kd_max, float t_max)
{
    std::vector<uint8_t> data(8);

    uint16_t p_u = floatToUint(p_des, p_min, p_max, 16);
    uint16_t v_u = floatToUint(v_des, v_min, v_max, 16);
    uint16_t kp_u = floatToUint(kp, 0, kp_max, 16);
    uint16_t kd_u = floatToUint(kd, 0, kd_max, 16);

    data[0] = p_u >> 8;
    data[1] = p_u & 0xFF;
    data[2] = v_u >> 8;
    data[3] = v_u & 0xFF;
    data[4] = kp_u >> 8;
    data[5] = kp_u & 0xFF;
    data[6] = kd_u >> 8;
    data[7] = kd_u & 0xFF;

    return data;
}

std::vector<uint8_t> RobStrideProtocol::createEnableCommand()
{
    return std::vector<uint8_t>(8, 0); // 8 bytes of 0
}

std::vector<uint8_t> RobStrideProtocol::createDisableCommand()
{
    return std::vector<uint8_t>(8, 0); // 8 bytes of 0
}

std::tuple<float, float, float, float ,float> RobStrideProtocol::parseFeedback(
    const std::vector<uint8_t>& data,
    float p_min, float p_max, float v_min, float v_max, float t_max)
{
    if (data.size() < 8) return {0,0,0,0,0};
    uint16_t p_int = (data[0] << 8) | data[1];
    uint16_t v_int = (data[2] << 8) | data[3];
    uint16_t t_int = (data[4] << 8) | data[5];
    uint16_t temp_int = (data[6] << 8) | data[7];

    float p = -((static_cast<float>(p_int) / 32767.0f) - 1.0f) * p_max;
    float v = -((static_cast<float>(v_int) / 32767.0f) - 1.0f) * v_max;
    float t = ((static_cast<float>(t_int) / 32767.0f) - 1.0f) * t_max;
    float temp = static_cast<float>(temp_int) * 0.1f;
    // float c = t / 1.09f;

    float torque_nm = (static_cast<float>(t_int) / 65535.0f) * 72.0f - 36.0f;
    float c = torque_nm / 1.09f;


    // Torque(Nm) = (RawData / 65535.0) * 72.0 - 36.0
    // Current(A) = Torque(Nm) / 1.09

    return {p, v, t, temp, c};
}
