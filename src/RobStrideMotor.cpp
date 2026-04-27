#include "robstride_rdk_ros2/RobStrideMotor.hpp"
#include <rclcpp/rclcpp.hpp>

RobStrideMotor::RobStrideMotor(std::shared_ptr<CanTransport> transport, uint8_t motor_id, ActuatorType type)
    : transport_(transport), motor_id_(motor_id), type_(type)
{
    loadLimits();
}

void RobStrideMotor::loadLimits()
{
    switch(type_)
    {
        case ActuatorType::ROBSTRIDE_00:
            limits_ = { 4 * M_PI, 50.0f, 17.0f, 500.0f, 5.0f };
            break;
        case ActuatorType::ROBSTRIDE_01:
            limits_ = { 4 * M_PI, 50.0f, 17.0f, 1500.0f, 20.0f };
            break;
        case ActuatorType::ROBSTRIDE_02:
            limits_ = { 4 * M_PI, 40.0f, 34.0f, 2000.0f, 20.0f };
            break;
        case ActuatorType::ROBSTRIDE_03:
            limits_ = { 4 * M_PI, 30.0f, 34.0f, 3000.0f, 50.0f };
            break;
        case ActuatorType::ROBSTRIDE_04:
            limits_ = { 4 * M_PI, 25.0f, 50.0f, 4000.0f, 80.0f };
            break;
        case ActuatorType::ROBSTRIDE_05:
            limits_ = { 4 * M_PI, 20.0f, 60.0f, 5000.0f, 100.0f };
            break;
        case ActuatorType::ROBSTRIDE_06:
            limits_ = { 4 * M_PI, 20.0f, 60.0f, 5000.0f, 100.0f };
            break;
        case ActuatorType::CUSTOM:
        default:
            limits_ = { 4 * M_PI, 50.0f, 17.0f, 1500.0f, 20.0f };
            break;
    }
}

bool RobStrideMotor::enable()
{
    uint32_t id = RobStrideProtocol::generateCommandId(
        ProtocolCmd::MOTOR_ENABLE, master_id_, motor_id_);

    auto data = RobStrideProtocol::createEnableCommand();

    if (transport_->send(id, data))
    {
        RCLCPP_INFO(
            rclcpp::get_logger("robstride_motor"),
            "Motor %u Enable Command Sent.",
            static_cast<unsigned>(motor_id_));
        return true;
    }
    return false;
}

bool RobStrideMotor::disable()
{
    uint32_t id = RobStrideProtocol::generateCommandId(
        ProtocolCmd::MOTOR_STOP, master_id_, motor_id_
    );
    auto data = RobStrideProtocol::createDisableCommand();

    if (transport_->send(id, data))
    {
        RCLCPP_INFO(
            rclcpp::get_logger("robstride_motor"),
            "Motor %u Disable Command Sent.",
            static_cast<unsigned>(motor_id_));
        return true;
    }
    return false;
}

bool RobStrideMotor::sendMotionCommand(float torque, float position, float velocity, float kp, float kd)
{
    // 부호 반전
    float hw_position = -position;
    float hw_velocity = -velocity;

    uint16_t t_uint = RobStrideProtocol::floatToUint(torque, -limits_.torque_limit, limits_.torque_limit, 16);

    // ID 생성 (수동 조작 필요, 왜냐하면 표준 포맷과 약간 다름)
    uint32_t id = (ProtocolCmd::MOTION_CONTROL << 24) | (t_uint << 8) | motor_id_;

    auto data = RobStrideProtocol::createMotionCommand(
        hw_position, hw_velocity, kp, kd, 0.0f, // t_ff is in ID
        -limits_.pos_limit, limits_.pos_limit,
        -limits_.vel_limit, limits_.vel_limit,
        limits_.kp_max, limits_.kd_max, 0.0f
    );

    return transport_->send(id, data);
}

bool RobStrideMotor::processPacket(uint32_t rx_id, const std::vector<uint8_t>& rx_data)
{
    uint8_t received_motor_id = RobStrideProtocol::getMotorIdFromCanId(rx_id);
    uint8_t type = RobStrideProtocol::getTypeFromCanId(rx_id);

    // 모터 ID 확인
    if (received_motor_id != motor_id_)
    {
        return false; // 내 데이터 아님
    }

    // MOTOR_REQUEST 또는 MOTION_CONTROL 응답 모두 피드백으로 처리
    if (type == ProtocolCmd::MOTOR_REQUEST || type == ProtocolCmd::MOTION_CONTROL)
    {
        auto [p, v, t, temp, c] = RobStrideProtocol::parseFeedback(
            rx_data,
            limits_.pos_limit, limits_.pos_limit,
            limits_.vel_limit, limits_.vel_limit,
            limits_.torque_limit
        );

        position_ = p;
        velocity_ = v;
        torque_ = t;
        temperature_ = temp;
        current_ = c;
        return true;
    }
    return false;
}
