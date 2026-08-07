#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

enum class WorkerOperation { None, Read, Write };

enum class WorkerResult
{
    Idle, Success, PartialSuccess, Timeout,
    BusUnavailable, InvalidCommand, IoError
};

enum class WriteResult
{
    Ok, TryAgain, NoBuffer, BusDown, IoError, InvalidArg
};

struct MotorCommandData
{
    uint16_t motor_id{0};
    float torque{0.0f};
    float position{0.0f};
    float velocity{0.0f};
    float kp{0.0f};
    float kd{0.0f};
};

struct MotorStateData
{
    uint16_t motor_id{0};
    std::size_t global_packet_index{0};
    float position{0.0f};
    float velocity{0.0f};
    float current{0.0f};
    bool updated{false};
    bool valid{false};
    std::chrono::steady_clock::time_point feedback_time{};
};

struct BusCommandData { std::vector<MotorCommandData> commands; };

struct BusStateData
{
    std::vector<MotorStateData> states;
    std::size_t received_frame_count{0};
    std::size_t valid_frame_count{0};
    std::size_t invalid_frame_count{0};
    std::size_t updated_motor_count{0};
    bool all_motors_updated{false};
    bool timed_out{false};
    WorkerResult result{WorkerResult::Idle};
    std::chrono::microseconds elapsed{0};
};

struct BusWriteStats
{
    uint64_t ok_writes{0};
    uint64_t fail_writes{0};
    uint64_t eagain_count{0};
    uint64_t enobufs_count{0};
    uint32_t consecutive_eagain_cycles{0};
    uint32_t consecutive_enobufs_cycles{0};
    bool eagain_active{false};
    std::chrono::steady_clock::time_point first_eagain_time{};
};

struct MotorWriteStats
{
    WriteResult last_result{WriteResult::Ok};
    uint32_t consecutive_failures{0};
};

struct BusWorkerStatus
{
    WorkerOperation last_operation{WorkerOperation::None};
    WorkerResult last_result{WorkerResult::Idle};
    bool busy{false};
    bool running{false};
    uint64_t requested_generation{0};
    uint64_t completed_generation{0};
};
