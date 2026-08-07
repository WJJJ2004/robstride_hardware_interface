#pragma once

#include "robstride_rdk_ros2/CanTransport.hpp"
#include "robstride_rdk_ros2/HardwareData.hpp"
#include "robstride_rdk_ros2/RobStrideMotor.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

class CanBusWorker
{
public:
    CanBusWorker(
        std::string interface_name,
        std::shared_ptr<CanTransport> transport,
        std::vector<std::shared_ptr<RobStrideMotor>> motors,
        std::vector<std::size_t> global_packet_indices);
    ~CanBusWorker();

    CanBusWorker(const CanBusWorker&) = delete;
    CanBusWorker& operator=(const CanBusWorker&) = delete;
    CanBusWorker(CanBusWorker&&) = delete;
    CanBusWorker& operator=(CanBusWorker&&) = delete;

    bool start();
    void stop();
    bool isRunning() const;
    bool isBusy() const;
    const std::string& getInterfaceName() const;
    std::size_t getMotorCount() const;

    void setCommandData(const BusCommandData& command_data);
    uint64_t requestRead();
    uint64_t requestWrite();
    bool waitUntilCompleted(
        uint64_t generation,
        std::chrono::microseconds timeout);

    BusStateData getStateSnapshot() const;
    BusWriteStats getWriteStatsSnapshot() const;
    std::vector<MotorWriteStats> getMotorWriteStatsSnapshot() const;
    BusWorkerStatus getStatusSnapshot() const;
    void resetStatistics();

private:
    uint64_t requestOperation(WorkerOperation operation);
    void workerLoop();
    BusStateData performRead();
    WorkerResult performWrite();
    WriteResult safeSendCommand(
        RobStrideMotor& motor,
        const MotorCommandData& command);
    std::size_t findLocalMotorIndex(uint8_t motor_id) const;
    void initializeStateBuffer();

    static constexpr std::size_t MAX_RX_PACKETS_PER_CYCLE = 50;
    static constexpr auto READ_TIMEOUT = std::chrono::microseconds(1000);

    std::string interface_name_;
    std::shared_ptr<CanTransport> transport_;
    std::vector<std::shared_ptr<RobStrideMotor>> motors_;
    std::vector<std::size_t> global_packet_indices_;
    std::unordered_map<uint8_t, std::size_t> motor_id_to_local_index_;

    std::thread worker_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> busy_{false};

    mutable std::mutex work_mutex_;
    std::condition_variable work_cv_;
    WorkerOperation requested_operation_{WorkerOperation::None};
    uint64_t requested_generation_{0};
    uint64_t completed_generation_{0};

    std::condition_variable completion_cv_;

    mutable std::mutex command_mutex_;
    BusCommandData command_data_;

    mutable std::mutex state_mutex_;
    BusStateData state_data_;

    mutable std::mutex stats_mutex_;
    BusWriteStats write_stats_;
    std::vector<MotorWriteStats> motor_write_stats_;

    std::atomic<WorkerOperation> last_operation_{WorkerOperation::None};
    std::atomic<WorkerResult> last_result_{WorkerResult::Idle};
};
