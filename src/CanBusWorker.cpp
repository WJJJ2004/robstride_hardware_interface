#include "robstride_rdk_ros2/CanBusWorker.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <exception>
#include <limits>
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

    for (std::size_t local_index = 0;
         local_index < motors_.size();
         ++local_index)
    {
        if (!motors_[local_index]) {
            throw std::invalid_argument(
                "CanBusWorker: null motor pointer");
        }

        const uint8_t motor_id =
            motors_[local_index]->getMotorId();

        const auto result =
            motor_id_to_local_index_.emplace(
                motor_id,
                local_index);

        if (!result.second) {
            throw std::invalid_argument(
                "CanBusWorker: duplicate motor ID on bus " +
                interface_name_);
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
    BusStateData initial_state;
    initial_state.states.resize(motors_.size());

    for (std::size_t i = 0; i < motors_.size(); ++i) {
        initial_state.states[i].motor_id =
            motors_[i]->getMotorId();

        initial_state.states[i].global_packet_index =
            global_packet_indices_[i];
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_data_ = std::move(initial_state);
}

bool CanBusWorker::start()
{
    bool expected = false;

    if (!running_.compare_exchange_strong(
            expected,
            true,
            std::memory_order_acq_rel))
    {
        // 이미 실행 중
        return true;
    }

    if (!transport_ || !transport_->isOpen()) {
        running_.store(false, std::memory_order_release);
        last_result_.store(
            WorkerResult::BusUnavailable,
            std::memory_order_release);

        return false;
    }

    try {
        worker_thread_ =
            std::thread(&CanBusWorker::workerLoop, this);
    }
    catch (...) {
        running_.store(false, std::memory_order_release);
        throw;
    }

    return true;
}

void CanBusWorker::stop()
{
    const bool was_running =
        running_.exchange(false, std::memory_order_acq_rel);

    if (!was_running && !worker_thread_.joinable()) {
        return;
    }

    /*
     * work_cv_에서 기다리고 있는 worker를 깨운다.
     */
    work_cv_.notify_all();

    /*
     * waitUntilCompleted()에서 기다리고 있을 수 있는
     * 메인 스레드도 깨운다.
     */
    completion_cv_.notify_all();

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

void CanBusWorker::setCommandData(
    const BusCommandData& command_data)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_data_ = command_data;
}

uint64_t CanBusWorker::requestRead()
{
    return requestOperation(WorkerOperation::Read);
}

uint64_t CanBusWorker::requestWrite()
{
    return requestOperation(WorkerOperation::Write);
}

uint64_t CanBusWorker::requestOperation(
    WorkerOperation operation)
{
    if (operation == WorkerOperation::None) {
        return 0;
    }

    if (!running_.load(std::memory_order_acquire)) {
        return 0;
    }

    uint64_t generation = 0;

    {
        std::lock_guard<std::mutex> lock(work_mutex_);

        /*
         * 아직 이전 작업을 처리 중인데 새 작업을 덮어쓰는 상황을
         * 막기 위해 busy 여부를 검사한다.
         */
        if (busy_.load(std::memory_order_acquire) ||
            requested_generation_ != completed_generation_)
        {
            return 0;
        }

        requested_operation_ = operation;

        ++requested_generation_;
        generation = requested_generation_;
    }

    work_cv_.notify_one();

    return generation;
}

bool CanBusWorker::waitUntilCompleted(
    uint64_t generation,
    std::chrono::microseconds timeout)
{
    if (generation == 0) {
        return false;
    }

    std::unique_lock<std::mutex> lock(work_mutex_);

    return completion_cv_.wait_for(
        lock,
        timeout,
        [this, generation]()
        {
            if (!running_.load(std::memory_order_acquire)) {
                return true;
            }

            return completed_generation_ >= generation;
        });
}

void CanBusWorker::workerLoop()
{
    while (running_.load(std::memory_order_acquire)) {
        WorkerOperation operation = WorkerOperation::None;
        uint64_t generation = 0;

        {
            std::unique_lock<std::mutex> lock(work_mutex_);

            work_cv_.wait(
                lock,
                [this]()
                {
                    return
                        !running_.load(std::memory_order_acquire) ||
                        requested_generation_ >
                            completed_generation_;
                });

            if (!running_.load(std::memory_order_acquire)) {
                break;
            }

            operation = requested_operation_;
            generation = requested_generation_;

            busy_.store(true, std::memory_order_release);
        }

        last_operation_.store(
            operation,
            std::memory_order_release);

        WorkerResult operation_result =
            WorkerResult::IoError;

        try {
            switch (operation) {
                case WorkerOperation::Read:
                {
                    BusStateData local_state =
                        performRead();

                    operation_result =
                        local_state.result;

                    /*
                     * 실제 CAN receive가 끝난 뒤에만 mutex를 잡고
                     * 결과를 공유 버퍼에 반영한다.
                     */
                    {
                        std::lock_guard<std::mutex>
                            lock(state_mutex_);

                        state_data_ =
                            std::move(local_state);
                    }

                    break;
                }

                case WorkerOperation::Write:
                    operation_result =
                        performWrite();
                    break;

                case WorkerOperation::None:
                default:
                    operation_result =
                        WorkerResult::IoError;
                    break;
            }
        }
        catch (...) {
            operation_result =
                WorkerResult::IoError;
        }

        last_result_.store(
            operation_result,
            std::memory_order_release);

        {
            std::lock_guard<std::mutex> lock(work_mutex_);

            /*
             * 현재 처리한 요청의 generation까지 완료됐음을 표시한다.
             */
            completed_generation_ = generation;
            requested_operation_ = WorkerOperation::None;
        }

        busy_.store(false, std::memory_order_release);

        completion_cv_.notify_all();
    }
}

BusStateData CanBusWorker::performRead()
{
    BusStateData local_state;
    local_state.states.resize(motors_.size());

    for (std::size_t i = 0; i < motors_.size(); ++i) {
        local_state.states[i].motor_id =
            motors_[i]->getMotorId();

        local_state.states[i].global_packet_index =
            global_packet_indices_[i];

        /*
         * 이번 read cycle에서 새 프레임을 받기 전까지는
         * updated=false 상태로 시작한다.
         */
        local_state.states[i].updated = false;
        local_state.states[i].valid = false;
    }

    if (!transport_ || !transport_->isOpen()) {
        local_state.result =
            WorkerResult::BusUnavailable;

        return local_state;
    }

    const auto start_time =
        std::chrono::steady_clock::now();

    const auto deadline =
        start_time + READ_TIMEOUT;

    uint32_t rx_id = 0;
    std::vector<uint8_t> rx_data;

    while (
        local_state.received_frame_count <
            MAX_RX_PACKETS_PER_CYCLE)
    {
        if (std::chrono::steady_clock::now() >= deadline) {
            local_state.timed_out = true;
            break;
        }

        /*
         * receive timeout은 0으로 두고 non-blocking으로 읽는다.
         * 외부 deadline이 전체 read 시간을 제한한다.
         */
        const bool received =
            transport_->receive(
                rx_id,
                rx_data,
                0);

        if (!received) {
            break;
        }

        ++local_state.received_frame_count;

        const uint8_t motor_id =
            RobStrideProtocol::getMotorIdFromCanId(
                rx_id);

        const std::size_t local_index =
            findLocalMotorIndex(motor_id);

        if (local_index == motors_.size()) {
            ++local_state.invalid_frame_count;
            continue;
        }

        auto& motor = motors_[local_index];

        try {
            motor->processPacket(
                rx_id,
                rx_data);
        }
        catch (const std::exception&) {
            ++local_state.invalid_frame_count;
            continue;
        }

        const float position =
            static_cast<float>(
                motor->getPosition());

        const float velocity =
            static_cast<float>(
                motor->getVelocity());

        const float current =
            static_cast<float>(
                motor->getCurrent());

        const bool valid =
            std::isfinite(position) &&
            std::isfinite(velocity) &&
            std::isfinite(current);

        auto& state =
            local_state.states[local_index];

        state.position = position;
        state.velocity = velocity;
        state.current = current;
        state.run_state = motor->getRunState();
        state.fault_flags = motor->getFaultFlags();
        state.feedback_sequence = motor->getFeedbackSequence();

        state.updated = true;
        state.valid = valid;

        state.feedback_time =
            std::chrono::steady_clock::now();

        if (valid) {
            ++local_state.valid_frame_count;
        }
        else {
            ++local_state.invalid_frame_count;
        }
    }

    local_state.updated_motor_count =
        static_cast<std::size_t>(std::count_if(
            local_state.states.begin(),
            local_state.states.end(),
            [](const MotorStateData& state)
            {
                return state.updated && state.valid;
            }));

    local_state.all_motors_updated =
        std::all_of(
            local_state.states.begin(),
            local_state.states.end(),
            [](const MotorStateData& state)
            {
                return state.updated &&
                       state.valid;
            });

    const auto end_time =
        std::chrono::steady_clock::now();

    local_state.elapsed =
        std::chrono::duration_cast<
            std::chrono::microseconds>(
                end_time - start_time);

    if (local_state.timed_out) {
        local_state.result =
            WorkerResult::Timeout;
    }
    else if (local_state.all_motors_updated) {
        local_state.result =
            WorkerResult::Success;
    }
    else if (local_state.valid_frame_count > 0) {
        local_state.result =
            WorkerResult::PartialSuccess;
    }
    else {
        local_state.result =
            WorkerResult::PartialSuccess;
    }

    return local_state;
}

WorkerResult CanBusWorker::performWrite()
{
    if (!transport_ || !transport_->isOpen()) {
        return WorkerResult::BusUnavailable;
    }

    BusCommandData command_snapshot;

    /*
     * command mutex를 짧게 잡고 로컬로 복사한다.
     * 실제 CAN 송신 중에는 mutex를 잡지 않는다.
     */
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_snapshot = command_data_;
    }

    if (command_snapshot.commands.size() != motors_.size()) {
        return WorkerResult::InvalidCommand;
    }

    bool any_failure = false;
    bool bus_down = false;

    bool eagain_this_cycle = false;
    bool enobufs_this_cycle = false;

    for (std::size_t local_index = 0;
         local_index < motors_.size();
         ++local_index)
    {
        auto& motor = motors_[local_index];
        const auto& command =
            command_snapshot.commands[local_index];

        if (command.motor_id != motor->getMotorId()) {
            any_failure = true;

            std::lock_guard<std::mutex> lock(stats_mutex_);

            ++write_stats_.fail_writes;

            motor_write_stats_[local_index].last_result =
                WriteResult::InvalidArg;

            ++motor_write_stats_[local_index]
                .consecutive_failures;

            continue;
        }

        const WriteResult result =
            safeSendCommand(
                *motor,
                command);

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);

            motor_write_stats_[local_index].last_result =
                result;

            if (result == WriteResult::Ok) {
                ++write_stats_.ok_writes;

                motor_write_stats_[local_index]
                    .consecutive_failures = 0;
            }
            else {
                ++write_stats_.fail_writes;

                ++motor_write_stats_[local_index]
                    .consecutive_failures;

                any_failure = true;

                switch (result) {
                    case WriteResult::TryAgain:
                        ++write_stats_.eagain_count;
                        eagain_this_cycle = true;
                        break;

                    case WriteResult::NoBuffer:
                        ++write_stats_.enobufs_count;
                        enobufs_this_cycle = true;
                        break;

                    case WriteResult::BusDown:
                        bus_down = true;
                        break;

                    default:
                        break;
                }
            }
        }

        /*
         * EAGAIN이나 ENOBUFS라면 같은 버스의 TX queue가
         * 막혀 있을 가능성이 크므로 이번 버스의 나머지 송신을
         * 중단한다.
         */
        if (result == WriteResult::TryAgain ||
            result == WriteResult::NoBuffer ||
            result == WriteResult::BusDown)
        {
            break;
        }
    }

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        if (eagain_this_cycle) {
            ++write_stats_.consecutive_eagain_cycles;

            if (!write_stats_.eagain_active) {
                write_stats_.eagain_active = true;
                write_stats_.first_eagain_time =
                    std::chrono::steady_clock::now();
            }
        }
        else {
            write_stats_.consecutive_eagain_cycles = 0;
            write_stats_.eagain_active = false;
        }

        if (enobufs_this_cycle) {
            ++write_stats_.consecutive_enobufs_cycles;
        }
        else {
            write_stats_.consecutive_enobufs_cycles = 0;
        }
    }

    if (bus_down) {
        return WorkerResult::BusUnavailable;
    }

    if (any_failure) {
        return WorkerResult::PartialSuccess;
    }

    return WorkerResult::Success;
}

WriteResult CanBusWorker::safeSendCommand(
    RobStrideMotor& motor,
    const MotorCommandData& command)
{
    errno = 0;

    const bool success =
        motor.sendMotionCommand(
            command.torque,
            command.position,
            command.velocity,
            command.kp,
            command.kd);

    if (success) {
        return WriteResult::Ok;
    }

    const int saved_errno = errno;

    if (saved_errno == EAGAIN ||
        saved_errno == EWOULDBLOCK)
    {
        return WriteResult::TryAgain;
    }

    if (saved_errno == ENOBUFS) {
        return WriteResult::NoBuffer;
    }

    if (saved_errno == ENETDOWN ||
        saved_errno == ENODEV)
    {
        return WriteResult::BusDown;
    }

    if (saved_errno == EINVAL) {
        return WriteResult::InvalidArg;
    }

    return WriteResult::IoError;
}

std::size_t CanBusWorker::findLocalMotorIndex(
    uint8_t motor_id) const
{
    const auto iterator =
        motor_id_to_local_index_.find(motor_id);

    if (iterator ==
        motor_id_to_local_index_.end())
    {
        return motors_.size();
    }

    return iterator->second;
}

BusStateData CanBusWorker::getStateSnapshot() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_data_;
}

BusWriteStats
CanBusWorker::getWriteStatsSnapshot() const
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

BusWorkerStatus
CanBusWorker::getStatusSnapshot() const
{
    BusWorkerStatus status;

    status.last_operation =
        last_operation_.load(
            std::memory_order_acquire);

    status.last_result =
        last_result_.load(
            std::memory_order_acquire);

    status.busy =
        busy_.load(
            std::memory_order_acquire);

    status.running =
        running_.load(
            std::memory_order_acquire);

    {
        std::lock_guard<std::mutex> lock(work_mutex_);

        status.requested_generation =
            requested_generation_;

        status.completed_generation =
            completed_generation_;
    }

    return status;
}

void CanBusWorker::resetStatistics()
{
    std::lock_guard<std::mutex> lock(stats_mutex_);

    write_stats_ = BusWriteStats{};

    std::fill(
        motor_write_stats_.begin(),
        motor_write_stats_.end(),
        MotorWriteStats{});
}
