#include "robstride_rdk_ros2/CanBusWorker.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <exception>
#include <utility>

CanBusWorker::CanBusWorker(
    std::string interface_name,
    std::shared_ptr<CanTransport> transport,
    std::vector<std::shared_ptr<RobStrideMotor>> motors,
    std::vector<std::size_t> global_packet_indices)
: interface_name_(std::move(interface_name)),
  transport_(std::move(transport)),
  motors_(std::move(motors)),
  global_packet_indices_(std::move(global_packet_indices))
{
    if (motors_.size() != global_packet_indices_.size()) {
        throw std::invalid_argument(
            "CanBusWorker: motors and global_packet_indices size mismatch");
    }
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        if (!motors_[i]) {
            throw std::invalid_argument("CanBusWorker: null motor pointer");
        }
        if (!motor_id_to_local_index_
                .emplace(motors_[i]->getMotorId(), i).second) {
            throw std::invalid_argument(
                "CanBusWorker: duplicate motor ID on bus " + interface_name_);
        }
    }
    motor_write_stats_.resize(motors_.size());
    initializeStateBuffer();
}

CanBusWorker::~CanBusWorker()
{
    stop();
}

void CanBusWorker::initializeStateBuffer()
{
    BusStateData initial;
    initial.states.resize(motors_.size());
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        initial.states[i].motor_id = motors_[i]->getMotorId();
        initial.states[i].global_packet_index = global_packet_indices_[i];
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_data_ = std::move(initial);
}

bool CanBusWorker::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }
    if (!transport_ || !transport_->isOpen()) {
        running_.store(false);
        last_result_.store(WorkerResult::BusUnavailable);
        return false;
    }
    worker_thread_ = std::thread(&CanBusWorker::workerLoop, this);
    return true;
}

void CanBusWorker::stop()
{
    running_.store(false, std::memory_order_release);
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    busy_.store(false, std::memory_order_release);
}

bool CanBusWorker::isRunning() const
{
    return running_.load(std::memory_order_acquire);
}

bool CanBusWorker::isBusy() const
{
    return busy_.load(std::memory_order_acquire);
}

const std::string& CanBusWorker::getInterfaceName() const
{
    return interface_name_;
}

std::size_t CanBusWorker::getMotorCount() const
{
    return motors_.size();
}

void CanBusWorker::workerLoop()
{
    while (running_.load(std::memory_order_acquire)) {
        uint32_t can_id = 0;
        std::vector<uint8_t> data;
        bool received = false;
        try {
            received = transport_->receive(
                can_id, data, RX_POLL_TIMEOUT_MS);
        } catch (const std::exception&) {
            last_result_.store(WorkerResult::IoError);
            continue;
        }
        if (!received) {
            continue;
        }
        processReceivedFrame(can_id, data);

        for (std::size_t i = 1; i < MAX_RX_BATCH; ++i) {
            try {
                if (!transport_->receive(can_id, data, 0)) {
                    break;
                }
                processReceivedFrame(can_id, data);
            } catch (const std::exception&) {
                last_result_.store(WorkerResult::IoError);
                break;
            }
        }
    }
}

void CanBusWorker::processReceivedFrame(
    uint32_t can_id,
    const std::vector<uint8_t>& data)
{
    const uint8_t motor_id =
        RobStrideProtocol::getMotorIdFromCanId(can_id);
    const std::size_t local_index = findLocalMotorIndex(motor_id);
    if (local_index == motors_.size()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++state_data_.received_frame_count;
        ++state_data_.invalid_frame_count;
        return;
    }

    auto& motor = motors_[local_index];
    try {
        motor->processPacket(can_id, data);
    } catch (const std::exception&) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++state_data_.received_frame_count;
        ++state_data_.invalid_frame_count;
        return;
    }

    MotorStateData state;
    state.motor_id = motor->getMotorId();
    state.global_packet_index = global_packet_indices_[local_index];
    state.position = motor->getPosition();
    state.velocity = motor->getVelocity();
    state.current = motor->getCurrent();
    state.run_state = motor->getRunState();
    state.fault_flags = motor->getFaultFlags();
    state.feedback_sequence = motor->getFeedbackSequence();
    state.valid = std::isfinite(state.position) &&
        std::isfinite(state.velocity) && std::isfinite(state.current);
    state.updated = true;
    state.feedback_time = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_data_.states[local_index] = state;
        ++state_data_.received_frame_count;
        if (state.valid) {
            ++state_data_.valid_frame_count;
        } else {
            ++state_data_.invalid_frame_count;
        }
        state_data_.result = state.valid
            ? WorkerResult::Success : WorkerResult::PartialSuccess;
    }
    last_operation_.store(WorkerOperation::Read);
    last_result_.store(state.valid
        ? WorkerResult::Success : WorkerResult::PartialSuccess);
}

BusSendReport CanBusWorker::sendCommandData(
    const BusCommandData& command_data)
{
    if (command_data.commands.size() != motors_.size()) {
        BusSendReport report;
        report.result = WorkerResult::InvalidCommand;
        report.requested = motors_.size();
        report.error_number = EINVAL;
        for (const auto& motor : motors_) {
            report.failed_motor_ids.push_back(motor->getMotorId());
        }
        return report;
    }
    std::vector<CanTxFrameData> frames;
    frames.reserve(motors_.size());
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const auto& command = command_data.commands[i];
        if (command.motor_id != motors_[i]->getMotorId()) {
            BusSendReport report;
            report.result = WorkerResult::InvalidCommand;
            report.requested = motors_.size();
            report.error_number = EINVAL;
            for (const auto& motor : motors_) {
                report.failed_motor_ids.push_back(motor->getMotorId());
            }
            return report;
        }
        frames.push_back(motors_[i]->createMotionCommand(
            command.torque, command.position, command.velocity,
            command.kp, command.kd));
    }
    return sendFrames(frames);
}

BusSendReport CanBusWorker::sendDisableAll()
{
    std::vector<CanTxFrameData> frames;
    frames.reserve(motors_.size());
    for (const auto& motor : motors_) {
        frames.push_back(motor->createDisableCommand());
    }
    return sendFrames(frames);
}

BusSendReport CanBusWorker::configureCanWatchdog(uint32_t timeout_raw)
{
    std::vector<CanTxFrameData> frames;
    frames.reserve(motors_.size());
    for (const auto& motor : motors_) {
        frames.push_back(motor->createCanTimeoutCommand(timeout_raw));
    }
    return sendFrames(frames);
}

BusSendReport CanBusWorker::sendFrames(
    const std::vector<CanTxFrameData>& frames)
{
    BusSendReport report;
    report.requested = frames.size();
    if (!transport_ || !transport_->isOpen()) {
        last_result_.store(WorkerResult::BusUnavailable);
        report.result = WorkerResult::BusUnavailable;
        report.error_number = ENOTCONN;
        for (const auto& frame : frames) {
            report.failed_motor_ids.push_back(frame.motor_id);
        }
        return report;
    }
    busy_.store(true);
    const CanBatchSendResult result = transport_->sendBatch(frames);
    busy_.store(false);

    report.queued = result.queued;
    report.error_number = result.error_number;
    for (std::size_t i = result.queued; i < frames.size(); ++i) {
        report.failed_motor_ids.push_back(frames[i].motor_id);
    }

    const std::size_t failed = result.requested - result.queued;
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        write_stats_.ok_writes += result.queued;
        write_stats_.fail_writes += failed;

        const bool eagain = result.error_number == EAGAIN ||
            result.error_number == EWOULDBLOCK;
        const bool enobufs = result.error_number == ENOBUFS;
        if (eagain) {
            write_stats_.eagain_count += failed;
            ++write_stats_.consecutive_eagain_cycles;
            if (!write_stats_.eagain_active) {
                write_stats_.eagain_active = true;
                write_stats_.first_eagain_time =
                    std::chrono::steady_clock::now();
            }
        } else {
            write_stats_.consecutive_eagain_cycles = 0;
            write_stats_.eagain_active = false;
        }
        if (enobufs) {
            write_stats_.enobufs_count += failed;
            ++write_stats_.consecutive_enobufs_cycles;
        } else {
            write_stats_.consecutive_enobufs_cycles = 0;
        }
        for (std::size_t i = 0; i < motor_write_stats_.size(); ++i) {
            const bool queued = i < result.queued;
            motor_write_stats_[i].last_result = queued
                ? WriteResult::Ok
                : (enobufs ? WriteResult::NoBuffer
                    : (eagain ? WriteResult::TryAgain : WriteResult::IoError));
            motor_write_stats_[i].consecutive_failures = queued
                ? 0 : motor_write_stats_[i].consecutive_failures + 1;
        }
    }

    last_operation_.store(WorkerOperation::Write);
    const WorkerResult worker_result = failed == 0
        ? WorkerResult::Success : WorkerResult::PartialSuccess;
    last_result_.store(worker_result);
    report.result = worker_result;
    return report;
}

std::size_t CanBusWorker::findLocalMotorIndex(uint8_t motor_id) const
{
    const auto it = motor_id_to_local_index_.find(motor_id);
    return it == motor_id_to_local_index_.end()
        ? motors_.size() : it->second;
}

BusStateData CanBusWorker::getStateSnapshot() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_data_;
}

BusWriteStats CanBusWorker::getWriteStatsSnapshot() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return write_stats_;
}

std::vector<MotorWriteStats>
CanBusWorker::getMotorWriteStatsSnapshot() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return motor_write_stats_;
}

BusWorkerStatus CanBusWorker::getStatusSnapshot() const
{
    BusWorkerStatus status;
    status.last_operation = last_operation_.load();
    status.last_result = last_result_.load();
    status.busy = busy_.load();
    status.running = running_.load();
    return status;
}

void CanBusWorker::resetStatistics()
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    write_stats_ = BusWriteStats{};
    std::fill(
        motor_write_stats_.begin(), motor_write_stats_.end(),
        MotorWriteStats{});
}
