#include "robstride_rdk_ros2/RobStrideMotor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>

RobStrideMotor::RobStrideMotor(std::shared_ptr<CanTransport> transport, uint8_t motor_id, ActuatorType type)
    : transport_(transport), motor_id_(motor_id), type_(type)
{
    loadLimits();
}

void RobStrideMotor::loadLimits()
{
    // RobStride private-protocol Type-1/Type-2 quantization ranges.
    // Source: RobStride motor communication protocol summary, 2025-12-29.
    constexpr float POSITION_LIMIT = 12.57f;
    switch(type_)
    {
        case ActuatorType::ROBSTRIDE_00:
            limits_ = { POSITION_LIMIT, 33.0f, 14.0f, 500.0f, 5.0f };
            break;
        case ActuatorType::ROBSTRIDE_01:
            limits_ = { POSITION_LIMIT, 44.0f, 17.0f, 500.0f, 5.0f };
            break;
        case ActuatorType::ROBSTRIDE_02:
            limits_ = { POSITION_LIMIT, 44.0f, 17.0f, 500.0f, 5.0f };
            break;
        case ActuatorType::ROBSTRIDE_03:
            limits_ = { POSITION_LIMIT, 20.0f, 60.0f, 5000.0f, 100.0f };
            break;
        case ActuatorType::ROBSTRIDE_04:
            limits_ = { POSITION_LIMIT, 15.0f, 120.0f, 5000.0f, 100.0f };
            break;
        case ActuatorType::ROBSTRIDE_05:
            limits_ = { POSITION_LIMIT, 50.0f, 5.5f, 500.0f, 5.0f };
            break;
        case ActuatorType::ROBSTRIDE_06:
            limits_ = { POSITION_LIMIT, 50.0f, 36.0f, 5000.0f, 100.0f };
            break;
        case ActuatorType::CUSTOM:
        default:
            limits_ = { POSITION_LIMIT, 50.0f, 17.0f, 1500.0f, 20.0f };
            break;
    }
}

bool RobStrideMotor::enable()
{
    const auto frame = createEnableCommand();

    if (transport_->send(frame))
    {
        RCLCPP_INFO(
            rclcpp::get_logger("robstride_motor"),
            "Enable command sent: motor_id=%u bus=%s",
            static_cast<unsigned>(motor_id_),
            transport_->getInterfaceName().c_str());
        return true;
    }
    return false;
}

bool RobStrideMotor::disable()
{
    const auto frame = createDisableCommand();

    if (transport_->send(frame))
    {
        RCLCPP_INFO(
            rclcpp::get_logger("robstride_motor"),
            "Disable command sent: motor_id=%u bus=%s",
            static_cast<unsigned>(motor_id_),
            transport_->getInterfaceName().c_str());
        return true;
    }
    return false;
}

bool RobStrideMotor::sendMotionCommand(float torque, float position, float velocity, float kp, float kd)
{
    const auto frame = createMotionCommand(torque, position, velocity, kp, kd);
    return transport_->send(frame);
}

CanTxFrameData RobStrideMotor::createMotionCommand(
    float torque, float position, float velocity, float kp, float kd) const
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

    return CanTxFrameData{
        id, std::move(data), motor_id_, "MOTION_CONTROL"};
}

CanTxFrameData RobStrideMotor::createEnableCommand() const
{
    return CanTxFrameData{
        RobStrideProtocol::generateCommandId(
            ProtocolCmd::MOTOR_ENABLE, master_id_, motor_id_),
        RobStrideProtocol::createEnableCommand(),
        motor_id_,
        "ENABLE"};
}

CanTxFrameData RobStrideMotor::createDisableCommand() const
{
    return CanTxFrameData{
        RobStrideProtocol::generateCommandId(
            ProtocolCmd::MOTOR_STOP, master_id_, motor_id_),
        RobStrideProtocol::createDisableCommand(),
        motor_id_,
        "DISABLE"};
}

CanTxFrameData RobStrideMotor::createCanTimeoutCommand(
    uint32_t timeout_raw) const
{
    std::vector<uint8_t> data(8, 0);
    constexpr uint16_t CAN_TIMEOUT_INDEX = 0x7028;
    data[0] = static_cast<uint8_t>(CAN_TIMEOUT_INDEX & 0xFF);
    data[1] = static_cast<uint8_t>(CAN_TIMEOUT_INDEX >> 8);
    std::memcpy(&data[4], &timeout_raw, sizeof(timeout_raw));
    constexpr uint8_t PARAM_WRITE_TYPE = 0x12;
    return CanTxFrameData{
        RobStrideProtocol::generateCommandId(
            PARAM_WRITE_TYPE, master_id_, motor_id_),
        std::move(data),
        motor_id_,
        "CAN_TIMEOUT_WRITE"};
}

void RobStrideMotor::processPacket(uint32_t rx_id, const std::vector<uint8_t>& rx_data)
{
    uint8_t received_motor_id = RobStrideProtocol::getMotorIdFromCanId(rx_id);
    uint8_t type = RobStrideProtocol::getTypeFromCanId(rx_id);

    // 모터 ID 확인
    if (received_motor_id != motor_id_)
    {
        throw std::invalid_argument(
            "Packet motor ID mismatch: received=" +
            std::to_string(received_motor_id) +
            ", expected=" +
            std::to_string(motor_id_));
    }

    if (type != ProtocolCmd::MOTOR_REQUEST &&
        type != ProtocolCmd::MOTION_CONTROL)
    {
        throw std::runtime_error(
            "Received packet type is not a valid feedback type for this motor: " +
            std::to_string(type));
        // return false;
    }

    if (rx_data.size() != 8)
    {
        throw std::runtime_error(
            "Received packet data length is invalid: " +
            std::to_string(rx_data.size()));

        // return false;
    }

    auto [p, v, t, temp, c] = RobStrideProtocol::parseFeedback(
        rx_data,
        -limits_.pos_limit, limits_.pos_limit,
        -limits_.vel_limit, limits_.vel_limit,
        limits_.torque_limit
    );

    if (!std::isfinite(p) ||
        !std::isfinite(v) ||
        !std::isfinite(t) ||
        !std::isfinite(temp) ||
        !std::isfinite(c))
    {
        throw std::runtime_error(
            "Received packet contains non-finite values: " +
            std::to_string(p) + ", " +
            std::to_string(v) + ", " +
            std::to_string(t) + ", " +
            std::to_string(temp) + ", " +
            std::to_string(c));
        // return false;
    }

    position_ = p;
    velocity_ = v;
    torque_ = t;
    temperature_ = temp;
    current_ = c;

    // Only Type-2 feedback defines run-state/fault fields in the extended
    // identifier. Type-1 frames may carry motion feedback but must never be
    // accepted as an Enable confirmation.
    if (type == ProtocolCmd::MOTOR_REQUEST)
    {
        fault_flags_.store(
            RobStrideProtocol::getFaultFlagsFromCanId(rx_id),
            std::memory_order_release);
        run_state_.store(
            RobStrideProtocol::getRunStateFromCanId(rx_id),
            std::memory_order_release);
        feedback_sequence_.fetch_add(1, std::memory_order_acq_rel);
    }

    // return true;
}
