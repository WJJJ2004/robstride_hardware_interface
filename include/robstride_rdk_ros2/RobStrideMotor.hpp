#pragma once
#include <stdexcept>
#include <string>
#include <cmath>

#include "CanTransport.hpp"
#include "RobStrideProtocol.hpp"
#include <memory>
#include <cstdint>
#include <vector>

enum class ActuatorType {
    ROBSTRIDE_00, ROBSTRIDE_01, ROBSTRIDE_02,
    ROBSTRIDE_03, ROBSTRIDE_04, ROBSTRIDE_05, ROBSTRIDE_06,
    CUSTOM
};

struct MotorLimits {
    float pos_limit;
    float vel_limit;
    float torque_limit;
    float kp_max;
    float kd_max;
};

class RobStrideMotor {
public:
    RobStrideMotor(std::shared_ptr<CanTransport> transport, uint8_t motor_id, ActuatorType type);

    bool enable();
    bool disable();

    bool sendMotionCommand(float torque, float position, float velocity, float kp, float kd);
    void processPacket(uint32_t rx_id, const std::vector<uint8_t>& rx_data);

    // Getters
    uint8_t getMotorId() const { return motor_id_; }
    float getPosition() const { return position_; }
    float getVelocity() const { return velocity_; }
    float getTorque() const { return torque_; }
    float getTemperature() const { return temperature_; }
    float getCurrent() const { return current_; }

private:
    std::shared_ptr<CanTransport> transport_;
    uint8_t motor_id_;
    uint8_t master_id_ = 0xFD;
    ActuatorType type_;
    MotorLimits limits_;

    // State
    float position_ = 0.0f;
    float velocity_ = 0.0f;
    float torque_ = 0.0f;
    float temperature_ = 0.0f;
    float current_ = 0.0f;

    void loadLimits();
    float getVelocityFeedbackScale() const;
};

// struct CanBusGroup
// {
//     std::string interface_name;
//     std::shared_ptr<CanTransport> transport;
//     std::vector<std::shared_ptr<RobStrideMotor>> motors;
// };
