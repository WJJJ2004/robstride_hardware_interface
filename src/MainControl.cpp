#include "robstride_rdk_ros2/MainControl.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>

static float wrapToPi(float angle)
{
    angle = std::fmod(angle, 2.0f * static_cast<float>(M_PI));
    if (angle > static_cast<float>(M_PI))
        angle -= 2.0f * static_cast<float>(M_PI);
    else if (angle < -static_cast<float>(M_PI))
        angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

MainControlNode::MainControlNode(const rclcpp::NodeOptions & options)
    : LifecycleNode("main_control_node", options)
{
    RCLCPP_INFO(this->get_logger(), "MainControlNode created");
}

MainControlNode::~MainControlNode()
{
    RCLCPP_INFO(this->get_logger(), "MainControlNode destroying...");

    timer_.reset();

    stopWorkers();

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        if (all_motors_[i])
        {
            all_motors_[i]->disable();
        }
    }

    flushCanRxQueues("destructor");

    RCLCPP_INFO(this->get_logger(), "MainControlNode destroyed");
}

void MainControlNode::initParameters()
{
    declare_parameter("baud_rate", 1000000);
    declare_parameter("control_frequency_hz", 200.0);
    declare_parameter("initial_interpolation_duration_sec", 1.0);
    declare_parameter<std::vector<std::string>>("can_interfaces", {"can0"});

    const auto can_interfaces = get_parameter("can_interfaces").as_string_array();

    for (const auto& can_name : can_interfaces)
    {
        declare_parameter<std::vector<int64_t>>(can_name + ".motor_ids", std::vector<int64_t>{});
        declare_parameter<std::vector<int64_t>>(can_name + ".motor_type", std::vector<int64_t>{});
    }

    RCLCPP_INFO(this->get_logger(), "[Configure] Parameters initialized");
    RCLCPP_INFO(this->get_logger(), "[Configure] baud_rate: %ld", get_parameter("baud_rate").as_int());
    RCLCPP_INFO(this->get_logger(), "[Configure] control_frequency_hz: %.3f",
        get_parameter("control_frequency_hz").as_double());
    RCLCPP_INFO(this->get_logger(),
        "[Configure] initial_interpolation_duration_sec: %.3f",
        get_parameter("initial_interpolation_duration_sec").as_double());
    RCLCPP_INFO(this->get_logger(), "[Configure] can_interfaces count: %zu", can_interfaces.size());

    for (const auto& can_name : can_interfaces)
    {
        auto ids = get_parameter(can_name + ".motor_ids").as_integer_array();
        RCLCPP_INFO(this->get_logger(), "[Configure] %s: %zu motors", can_name.c_str(), ids.size());
    }
}

bool MainControlNode::updateControlLoopPeriod()
{
    control_frequency_hz_ = get_parameter("control_frequency_hz").as_double();
    if (!std::isfinite(control_frequency_hz_) || control_frequency_hz_ <= 0.0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[Configure] control_frequency_hz must be finite and greater than 0 (received: %.3f)",
            control_frequency_hz_);
        return false;
    }

    initial_interpolation_duration_sec_ =
        get_parameter("initial_interpolation_duration_sec").as_double();
    if (!std::isfinite(initial_interpolation_duration_sec_) ||
        initial_interpolation_duration_sec_ <= 0.0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[Configure] initial_interpolation_duration_sec must be finite "
            "and greater than 0 (received: %.3f)",
            initial_interpolation_duration_sec_);
        return false;
    }

    // RX is handled continuously by each bus worker. One timer callback now
    // represents one complete control cycle.
    const double callback_period_ns =
        1.0e9 / control_frequency_hz_;
    if (callback_period_ns < 1.0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[Configure] control_frequency_hz is too high: %.3f",
            control_frequency_hz_);
        return false;
    }

    control_loop_period_ = std::chrono::nanoseconds(
        static_cast<int64_t>(std::llround(callback_period_ns)));

    RCLCPP_INFO(
        this->get_logger(),
        "[Configure] Control frequency: %.3f Hz, callback frequency: %.3f Hz, timer period: %.3f us",
        control_frequency_hz_,
        control_frequency_hz_,
        static_cast<double>(control_loop_period_.count()) / 1000.0);
    return true;
}

void MainControlNode::startControlLoopTimer()
{
    timer_ = this->create_wall_timer(
        control_loop_period_,
        std::bind(&MainControlNode::control_loop, this));
}

std::string MainControlNode::execute_command(const std::string& cmd)
{
    std::array<char, 256> buffer{};
    std::string result;

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        RCLCPP_ERROR(this->get_logger(), "popen() failed for command: %s", cmd.c_str());
        return "";
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }

    return result;
}

// ------------------------------- Motor Feedback Handling For Initial -------------------------------

void MainControlNode::disableAllMotors()
{
    const std::string motor_ids = formatMotorIdList(motor_ids_);
    RCLCPP_WARN(
        this->get_logger(),
        "[Safety] Sending disable command: motor_ids=%s",
        motor_ids.c_str());

    for (size_t i = 0;
         i < all_motors_.size();
         ++i)
    {
        if (!all_motors_[i])
        {
            continue;
        }

        if (!all_motors_[i]->disable())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[Safety] Disable command TX failed: "
                "motor_id=%u bus=%s",
                static_cast<unsigned>(motor_ids_[i]),
                i < packet_index_to_bus_.size()
                    ? packet_index_to_bus_[i].c_str()
                    : "unknown");
        }
    }
}

float MainControlNode::computeMedian(
    const std::deque<float>& samples) const
{
    if (samples.empty())
    {
        return 0.0f;
    }

    std::vector<float> sorted(
        samples.begin(),
        samples.end());

    std::sort(
        sorted.begin(),
        sorted.end());

    const size_t n = sorted.size();

    if ((n % 2) == 1)
    {
        return sorted[n / 2];
    }

    return 0.5f *
        (sorted[n / 2 - 1] +
         sorted[n / 2]);
}

std::string MainControlNode::formatMotorContext(
    std::size_t motor_index) const
{
    if (motor_index >= motor_ids_.size() ||
        motor_index >= packet_index_to_bus_.size())
    {
        return "motor_id=unknown bus=unknown "
            "internal_packet_index=" + std::to_string(motor_index);
    }
    return "motor_id=" + std::to_string(motor_ids_[motor_index]) +
        " bus=" + packet_index_to_bus_[motor_index];
}

std::string MainControlNode::formatMotorIdList(
    const std::vector<uint16_t>& motor_ids) const
{
    std::ostringstream stream;
    stream << '[';
    for (std::size_t i = 0; i < motor_ids.size(); ++i)
    {
        if (i != 0) stream << ',';
        stream << motor_ids[i];
    }
    stream << ']';
    return stream.str();
}

std::vector<uint16_t> MainControlNode::getGroupMotorIds(
    const CanBusGroup& group) const
{
    std::vector<uint16_t> ids;
    ids.reserve(group.motors.size());
    for (const auto& motor : group.motors)
    {
        if (motor) ids.push_back(motor->getMotorId());
    }
    return ids;
}

std::string MainControlNode::formatInitSampleReport(
    const InitSampleCheckReport& report) const
{
    if (!report.motor_id)
    {
        return report.detail;
    }
    return "motor_id=" + std::to_string(*report.motor_id) +
        " bus=" + report.bus + " " + report.detail;
}

bool MainControlNode::shouldEmitLog(
    const std::string& key,
    std::chrono::milliseconds interval)
{
    const auto now = std::chrono::steady_clock::now();
    const auto it = log_throttle_times_.find(key);
    if (it != log_throttle_times_.end() && now - it->second < interval)
    {
        return false;
    }
    log_throttle_times_[key] = now;
    return true;
}

InitSampleCheckReport MainControlNode::checkInitialSamples() const
{
    if (init_position_samples_.size() != all_motors_.size() ||
        init_velocity_samples_.size() != all_motors_.size() ||
        last_feedback_time_.size() != all_motors_.size() ||
        motor_ids_.size() != all_motors_.size() ||
        packet_index_to_bus_.size() != all_motors_.size())
    {
        return {InitSampleCheckResult::Fatal, std::nullopt, {},
            "[InternalInvariant] initial sample or motor metadata size mismatch"};
    }

    const auto now = std::chrono::steady_clock::now();

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        const auto& q_samples =
            init_position_samples_[i];

        const auto& qdot_samples =
            init_velocity_samples_[i];

        // 아직 필요한 개수만큼 수집되지 않은 경우
        if (q_samples.size() < INIT_REQUIRED_SAMPLES ||
            qdot_samples.size() < INIT_REQUIRED_SAMPLES)
        {
            return {
                InitSampleCheckResult::Collecting,
                motor_ids_[i],
                packet_index_to_bus_[i],
                "q_samples=" + std::to_string(q_samples.size()) +
                    "/" + std::to_string(INIT_REQUIRED_SAMPLES) +
                    " qdot_samples=" + std::to_string(qdot_samples.size()) +
                    "/" + std::to_string(INIT_REQUIRED_SAMPLES)};
        }

        const auto feedback_age =
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                now - last_feedback_time_[i]);

        if (feedback_age.count() >
            INIT_FEEDBACK_STALE_MS)
        {
            return {InitSampleCheckResult::Fatal, motor_ids_[i],
                packet_index_to_bus_[i],
                "stale_feedback age_ms=" +
                    std::to_string(feedback_age.count())};
        }

        const auto minmax =
            std::minmax_element(
                q_samples.begin(),
                q_samples.end());

        const float q_min = *minmax.first;
        const float q_max = *minmax.second;
        const float position_spread = q_max - q_min;

        if (!std::isfinite(position_spread))
        {
            return {InitSampleCheckResult::Fatal, motor_ids_[i],
                packet_index_to_bus_[i],
                "non-finite position spread"};
        }

        if (position_spread >
            INIT_POSITION_SPREAD_LIMIT)
        {
            return {InitSampleCheckResult::Fatal, motor_ids_[i],
                packet_index_to_bus_[i],
                "position_spread_too_large min=" + std::to_string(q_min) +
                    " max=" + std::to_string(q_max) +
                    " spread=" + std::to_string(position_spread) +
                    " limit=" + std::to_string(INIT_POSITION_SPREAD_LIMIT)};
        }

        for (const float qdot : qdot_samples)
        {
            if (!std::isfinite(qdot))
            {
                return {InitSampleCheckResult::Fatal, motor_ids_[i],
                    packet_index_to_bus_[i],
                    "non-finite velocity sample"};
            }

            if (std::fabs(qdot) >
                INIT_VELOCITY_LIMIT)
            {
                return {InitSampleCheckResult::Fatal, motor_ids_[i],
                    packet_index_to_bus_[i],
                    "initial_velocity_too_large qdot=" +
                        std::to_string(qdot) +
                        " limit=" + std::to_string(INIT_VELOCITY_LIMIT)};
            }
        }
    }

    return {InitSampleCheckResult::Ready, std::nullopt, {}, "ready"};
}

// ------------------------ Lifecycle Callbacks ------------------------

void MainControlNode::resetRuntimeStates()
{
    walk_initialized_ = false;
    start_positions_captured_ = false;
    start_position_init_attempted_ = false;

    interpolation_cycle_count_ = 0;
    interpolation_start_time_ = {};
    start_positions_.clear();
    last_read_cycle_all_updated_ = false;

    packet_initialized_ = false;
    initial_raw_position_printed_ = false;

    motor_feedback_seen_.assign(
        all_motors_.size(), false);

    last_valid_motor_pos_.assign(
        all_motors_.size(), 0.0f);

    init_position_samples_.clear();
    init_position_samples_.resize(all_motors_.size());

    init_velocity_samples_.clear();
    init_velocity_samples_.resize(all_motors_.size());

    last_feedback_time_.assign(
        all_motors_.size(),
        std::chrono::steady_clock::time_point{});

    init_phase_ = InitPhase::COLLECT_FEEDBACK;
    init_phase_start_time_ =
        std::chrono::steady_clock::now();

    latest_motor_states_.assign(all_motors_.size(), MotorStateData{});
    for (size_t i = 0; i < latest_motor_states_.size(); ++i)
    {
        latest_motor_states_[i].motor_id = motor_ids_[i];
        latest_motor_states_[i].global_packet_index = i;
    }

    for (auto& group : can_groups_)
    {
        group.last_write_summary_log = {};
        if (group.worker)
        {
            group.worker->resetStatistics();
        }
    }
}

CallbackReturn MainControlNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[Configure] Configuring...");

    initParameters();

    if (!updateControlLoopPeriod())
    {
        return CallbackReturn::FAILURE;
    }

    // if (!canSetup())
    // {
    //     RCLCPP_ERROR(this->get_logger(), "[Configure] CAN setup failed");
    //     return CallbackReturn::FAILURE;
    // }

    // Use best_effort so we're compatible with both best_effort and reliable publishers.
    // If we require reliable here and the publisher is best_effort, we will receive nothing.
    rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));
    cmd_qos.reliable();

    rclcpp::QoS motor_status_qos(rclcpp::KeepLast(1));
    motor_status_qos.best_effort();

    state_pub = this->create_publisher<roa_interfaces::msg::MotorStateArray>(
        "/hardware_interface/state", motor_status_qos);

    initial_pub = this->create_publisher<std_msgs::msg::Bool>(
        "walk_initialized", cmd_qos);

    walk_sub = this->create_subscription<roa_interfaces::msg::MotorCommandArray>(
        "/hardware_interface/command", cmd_qos,
        std::bind(&MainControlNode::walkCallback, this, std::placeholders::_1));

    torque_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/hardware_interface/etop", 10,
        std::bind(&MainControlNode::torqueCallback, this, std::placeholders::_1));

    const auto can_interfaces = get_parameter("can_interfaces").as_string_array();

    can_groups_.clear();
    all_motors_.clear();
    motor_ids_.clear();
    motor_id_to_index_.clear();
    packet_index_to_bus_.clear();

    for (const auto& can_name : can_interfaces)
    {
        CanBusGroup group;
        group.interface_name = can_name;
        group.transport = std::make_shared<CanTransport>();

        if (!group.transport->open(can_name))
        {
            requestFatalShutdown(
                "[Configure] Failed to open CAN interface '" +
                can_name + "'"
            );
            // RCLCPP_ERROR(this->get_logger(),
            //     "[Configure] Failed to open CAN interface '%s'",
            //     can_name.c_str());
            return CallbackReturn::FAILURE;
        }

        const auto motor_ids = get_parameter(can_name + ".motor_ids").as_integer_array();
        const auto motor_types = get_parameter(can_name + ".motor_type").as_integer_array();

        if (motor_ids.size() != motor_types.size())
        {
            requestFatalShutdown(
                "[Configure] motor_ids and motor_type size mismatch: bus=" +
                can_name + " motor_ids_count=" +
                std::to_string(motor_ids.size()) + " motor_type_count=" +
                std::to_string(motor_types.size()));
            // RCLCPP_ERROR(this->get_logger(),
            //     "[Configure] %s: motor_ids and motor_type size mismatch",
            //     can_name.c_str());
            return CallbackReturn::FAILURE;
        }

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            if (motor_types[i] < 0 || motor_types[i] > 6)
            {
                requestFatalShutdown(
                    "[Configure] Unsupported motor_type=" +
                    std::to_string(motor_types[i]) + " bus=" + can_name +
                    " motor_id=" + std::to_string(motor_ids[i]) +
                    "; expected a RobStride model number from 0 through 6");
                return CallbackReturn::FAILURE;
            }
            const auto type = static_cast<ActuatorType>(motor_types[i]);
            const auto id = static_cast<uint8_t>(motor_ids[i]);

            auto motor = std::make_shared<RobStrideMotor>(group.transport, id, type);
            group.motors.push_back(motor);
            all_motors_.push_back(motor);
            motor_ids_.push_back(id);

            const size_t packet_index = all_motors_.size() - 1;
            group.global_packet_indices.push_back(packet_index);
            packet_index_to_bus_.push_back(can_name);

            RCLCPP_INFO(this->get_logger(),
                "[Configure] Motor initialized: motor_id=%u bus=%s type=%ld",
                id, can_name.c_str(), motor_types[i]);
        }

        group.worker = std::make_shared<CanBusWorker>(
            group.interface_name,
            group.transport,
            group.motors,
            group.global_packet_indices);

        can_groups_.push_back(std::move(group));
    }

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        const uint16_t id = motor_ids_[i];

        if (motor_id_to_index_.find(id) != motor_id_to_index_.end())
        {
            const size_t previous_index = motor_id_to_index_.at(id);
            requestFatalShutdown(
                "[Configure] Duplicate motor_id detected: motor_id=" +
                std::to_string(id) + " first_bus=" +
                packet_index_to_bus_[previous_index] + " duplicate_bus=" +
                packet_index_to_bus_[i]);
            // RCLCPP_ERROR(this->get_logger(),
            //     "[Configure] Duplicate motor_id detected: %u", id);
            return CallbackReturn::FAILURE;
        }

        motor_id_to_index_[id] = i;

        RCLCPP_INFO(this->get_logger(),
            "[Configure] Motor mapping registered: motor_id=%u bus=%s",
            id, packet_index_to_bus_[i].c_str());
    }

    packet_commands_.commands.resize(all_motors_.size());
    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        const uint16_t id = motor_ids_[i];
        packet_commands_.commands[i].motor_id = id;
        packet_commands_.commands[i].torque   = 0.0f;
        packet_commands_.commands[i].position = 0.0f;
        packet_commands_.commands[i].velocity = 0.0f;
        packet_commands_.commands[i].kp       = 0.0f;
        packet_commands_.commands[i].kd       = 0.0f;
    }

    resetRuntimeStates();
    policy_state_ = SystemPolicyState::ACTIVE;
    last_safety_disable_time_ = {};
    tx_degraded_since_.clear();
    log_throttle_times_.clear();

    const std::string configured_motor_ids = formatMotorIdList(motor_ids_);
    RCLCPP_INFO(this->get_logger(),
        "[Configure] Configured successfully: motor_ids=%s "
        "can_interfaces=%zu total_motors=%zu",
        configured_motor_ids.c_str(), can_groups_.size(), all_motors_.size());

    velocity_filters_.clear();
    velocity_filters_.reserve(all_motors_.size());
    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        velocity_filters_.emplace_back(3.0f); // 23 hz -> 10 hz로 낮춰서 더 부드럽게
    }
    last_velocity_filter_time_ = std::chrono::steady_clock::now();
    velocity_filter_time_initialized_ = false;

    return CallbackReturn::SUCCESS;
}

CallbackReturn MainControlNode::on_activate(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(
        this->get_logger(),
        "[Activate] Activating...");

    resetRuntimeStates();
    policy_state_ = SystemPolicyState::ACTIVE;
    last_safety_disable_time_ = {};
    tx_degraded_since_.clear();
    log_throttle_times_.clear();
    flushCanRxQueues("before_enable");

    // Start RX workers before Enable so a fast Type-2 confirmation cannot be
    // left unread or mistaken for an old feedback frame.
    if (!startWorkers())
    {
        disableAllMotors();
        return CallbackReturn::FAILURE;
    }

    if (!collectInitialFeedbackDisabled())
    {
        RCLCPP_ERROR(this->get_logger(),
            "[Activate] Disabled-state feedback validation failed");
        sendSafetyDisableBatches();
        stopWorkers();
        return CallbackReturn::FAILURE;
    }

    for (auto& group : can_groups_)
    {
        if (!group.worker)
        {
            const std::string motor_ids =
                formatMotorIdList(getGroupMotorIds(group));
            RCLCPP_ERROR(this->get_logger(),
                "[Activate] CAN watchdog configuration failed: "
                "motor_ids=%s bus=%s reason=missing_worker",
                motor_ids.c_str(), group.interface_name.c_str());
            sendSafetyDisableBatches();
            stopWorkers();
            return CallbackReturn::FAILURE;
        }
        const BusSendReport report =
            group.worker->configureCanWatchdog(MOTOR_CAN_TIMEOUT_RAW);
        if (report.result != WorkerResult::Success)
        {
            const std::string failed_ids =
                formatMotorIdList(report.failed_motor_ids);
            RCLCPP_ERROR(this->get_logger(),
                "[Activate] CAN watchdog configuration TX failed: "
                "failed_motor_ids=%s bus=%s queued=%zu/%zu errno=%d (%s)",
                failed_ids.c_str(), group.interface_name.c_str(),
                report.queued, report.requested, report.error_number,
                report.error_number != 0
                    ? std::strerror(report.error_number) : "none");
            sendSafetyDisableBatches();
            stopWorkers();
            return CallbackReturn::FAILURE;
        }
    }

    std::vector<uint64_t> enable_baselines;
    enable_baselines.reserve(all_motors_.size());
    for (const auto& motor : all_motors_)
    {
        enable_baselines.push_back(
            motor ? motor->getFeedbackSequence() : 0);
    }

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        if (!all_motors_[i]->enable())
        {
            RCLCPP_ERROR(this->get_logger(),
                "[Activate] Enable command TX failed: motor_id=%u bus=%s",
                static_cast<unsigned>(motor_ids_[i]),
                packet_index_to_bus_[i].c_str());
            sendSafetyDisableBatches();
            stopWorkers();
            return CallbackReturn::FAILURE;
        }
    }

    if (!confirmMotorEnableStates(enable_baselines))
    {
        RCLCPP_ERROR(this->get_logger(),
            "[Activate] One or more motors did not confirm enabled state");
        sendSafetyDisableBatches();
        stopWorkers();
        return CallbackReturn::FAILURE;
    }
    torque_enabled_ = true;

    velocity_filters_.clear();
    velocity_filters_.reserve(all_motors_.size());

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        velocity_filters_.emplace_back(3.0f);
    }

    last_velocity_filter_time_ =
        std::chrono::steady_clock::now();

    velocity_filter_time_initialized_ = false;

    startControlLoopTimer();

    const std::string active_motor_ids = formatMotorIdList(motor_ids_);
    RCLCPP_INFO(
        this->get_logger(),
        "[Activate] Disabled feedback validated and motors enabled: "
        "motor_ids=%s; waiting for upper command",
        active_motor_ids.c_str());

    return CallbackReturn::SUCCESS;
}

CallbackReturn MainControlNode::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    timer_.reset();
    sendSafetyDisableBatches();
    stopWorkers();
    torque_enabled_ = false;
    return CallbackReturn::SUCCESS;
}

CallbackReturn MainControlNode::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    timer_.reset();
    stopWorkers();
    disableAllMotors();
    can_groups_.clear();
    all_motors_.clear();
    motor_ids_.clear();
    latest_motor_states_.clear();
    return CallbackReturn::SUCCESS;
}

CallbackReturn MainControlNode::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    timer_.reset();
    stopWorkers();
    disableAllMotors();
    return CallbackReturn::SUCCESS;
}

bool MainControlNode::startWorkers()
{
    for (auto& group : can_groups_)
    {
        if (!group.worker || !group.worker->start())
        {
            const std::string motor_ids =
                formatMotorIdList(getGroupMotorIds(group));
            RCLCPP_ERROR(this->get_logger(),
                "[Worker] Failed to start: motor_ids=%s bus=%s",
                motor_ids.c_str(), group.interface_name.c_str());
            stopWorkers();
            return false;
        }
    }
    return true;
}

void MainControlNode::stopWorkers()
{
    for (auto& group : can_groups_)
    {
        if (group.worker)
        {
            group.worker->stop();
        }
    }
}

bool MainControlNode::collectInitialFeedbackDisabled()
{
    std::vector<uint64_t> last_sequences(all_motors_.size(), 0);
    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        last_sequences[i] = all_motors_[i]->getFeedbackSequence();
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(INIT_COLLECTION_TIMEOUT_MS);
    while (std::chrono::steady_clock::now() < deadline)
    {
        for (auto& group : can_groups_)
        {
            if (!group.worker)
            {
                policy_state_ = SystemPolicyState::DEGRADED;
                continue;
            }
            const BusSendReport report = group.worker->sendDisableAll();
            if (report.result != WorkerResult::Success)
            {
                policy_state_ = SystemPolicyState::DEGRADED;
                const std::string failed_ids =
                    formatMotorIdList(report.failed_motor_ids);
                if (shouldEmitLog(
                    "disabled_probe:" + group.interface_name,
                    std::chrono::milliseconds(1000)))
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[CAN TX] DISABLED_PROBE batch incomplete: "
                        "failed_motor_ids=%s bus=%s queued=%zu/%zu "
                        "errno=%d (%s)",
                        failed_ids.c_str(), group.interface_name.c_str(),
                        report.queued, report.requested, report.error_number,
                        report.error_number != 0
                            ? std::strerror(report.error_number) : "none");
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        for (auto& group : can_groups_)
        {
            if (!group.worker)
            {
                continue;
            }
            const BusStateData snapshot = group.worker->getStateSnapshot();
            for (const auto& state : snapshot.states)
            {
                const size_t index = state.global_packet_index;
                if (index >= all_motors_.size() || !state.updated ||
                    !state.valid ||
                    state.feedback_sequence <= last_sequences[index])
                {
                    continue;
                }
                last_sequences[index] = state.feedback_sequence;
                if (state.fault_flags != 0)
                {
                    RCLCPP_ERROR(this->get_logger(),
                        "[Activate] Fault during disabled probe: "
                        "motor_id=%u bus=%s fault=0x%02X",
                        static_cast<unsigned>(motor_ids_[index]),
                        packet_index_to_bus_[index].c_str(),
                        static_cast<unsigned>(state.fault_flags));
                    return false;
                }

                const bool valid_feedback =
                    std::isfinite(state.position) &&
                    std::isfinite(state.velocity) &&
                    std::isfinite(state.current) &&
                    std::fabs(state.position) <= INIT_Q_ABS_LIMIT &&
                    std::fabs(state.velocity) <= INIT_VELOCITY_LIMIT;
                if (!valid_feedback)
                {
                    init_position_samples_[index].clear();
                    init_velocity_samples_[index].clear();
                    continue;
                }

                auto& q = init_position_samples_[index];
                auto& qdot = init_velocity_samples_[index];
                q.push_back(state.position);
                qdot.push_back(state.velocity);
                if (q.size() > INIT_REQUIRED_SAMPLES) q.pop_front();
                if (qdot.size() > INIT_REQUIRED_SAMPLES) qdot.pop_front();
                last_feedback_time_[index] = state.feedback_time;
            }
        }

        const InitSampleCheckReport sample_report = checkInitialSamples();
        if (sample_report.result == InitSampleCheckResult::Fatal)
        {
            const std::string detail = formatInitSampleReport(sample_report);
            RCLCPP_ERROR(this->get_logger(),
                "[Activate] Disabled feedback invalid: %s", detail.c_str());
            return false;
        }
        if (sample_report.result == InitSampleCheckResult::Ready)
        {
            start_positions_.resize(all_motors_.size());
            for (size_t i = 0; i < all_motors_.size(); ++i)
            {
                start_positions_[i] = computeMedian(init_position_samples_[i]);
                latest_motor_states_[i].position = start_positions_[i];
                latest_motor_states_[i].feedback_sequence = last_sequences[i];
                latest_motor_states_[i].feedback_time = last_feedback_time_[i];
                RCLCPP_INFO(this->get_logger(),
                    "[Activate] Disabled position validated: "
                    "motor_id=%u bus=%s median=%.6f samples=%zu",
                    static_cast<unsigned>(motor_ids_[i]),
                    packet_index_to_bus_[i].c_str(),
                    start_positions_[i], init_position_samples_[i].size());
            }
            start_positions_captured_ = true;
            init_phase_ = InitPhase::WAIT_COMMAND;
            interpolation_cycle_count_ = 0;
            interpolation_start_time_ = {};
            return true;
        }
    }

    const InitSampleCheckReport sample_report = checkInitialSamples();
    const std::string detail = formatInitSampleReport(sample_report);
    RCLCPP_ERROR(this->get_logger(),
        "[Activate] Disabled feedback collection timeout: %s",
        detail.c_str());
    return false;
}

bool MainControlNode::confirmMotorEnableStates(
    const std::vector<uint64_t>& baseline_sequences)
{
    if (baseline_sequences.size() != all_motors_.size())
    {
        RCLCPP_ERROR(this->get_logger(),
            "[Activate] Enable baseline size mismatch");
        return false;
    }

    std::vector<bool> confirmed(all_motors_.size(), false);
    std::vector<size_t> attempts(all_motors_.size(), 1);
    size_t confirmed_count = 0;

    const auto deadline =
        std::chrono::steady_clock::now() + ENABLE_CONFIRMATION_TIMEOUT;
    auto next_resend =
        std::chrono::steady_clock::now() + ENABLE_RESEND_INTERVAL;
    auto next_keepalive = std::chrono::steady_clock::now();

    while (confirmed_count < all_motors_.size() &&
           std::chrono::steady_clock::now() < deadline)
    {
        for (size_t i = 0; i < all_motors_.size(); ++i)
        {
            if (confirmed[i] || !all_motors_[i])
            {
                continue;
            }

            const uint64_t sequence =
                all_motors_[i]->getFeedbackSequence();
            if (sequence <= baseline_sequences[i])
            {
                continue;
            }

            const uint8_t fault_flags =
                all_motors_[i]->getFaultFlags();
            const uint8_t run_state =
                all_motors_[i]->getRunState();

            if (fault_flags != 0)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "[Activate] Motor fault while enabling: "
                    "motor_id=%u bus=%s fault=0x%02X run_state=%u",
                    static_cast<unsigned>(motor_ids_[i]),
                    packet_index_to_bus_[i].c_str(),
                    static_cast<unsigned>(fault_flags),
                    static_cast<unsigned>(run_state));
                return false;
            }

            if (run_state == MOTOR_RUN_STATE)
            {
                confirmed[i] = true;
                ++confirmed_count;
                RCLCPP_INFO(this->get_logger(),
                    "[Activate] Enable confirmed: "
                    "motor_id=%u bus=%s sequence=%lu attempts=%zu",
                    static_cast<unsigned>(motor_ids_[i]),
                    packet_index_to_bus_[i].c_str(),
                    static_cast<unsigned long>(sequence),
                    attempts[i]);
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_keepalive)
        {
            // CAN_TIMEOUT is already armed. A zero-gain motion frame keeps
            // confirmed motors alive without producing torque while slower
            // motors are still completing their Enable handshake.
            sendZeroCommands();
            next_keepalive = now + std::chrono::milliseconds(10);
        }
        if (confirmed_count < all_motors_.size() && now >= next_resend)
        {
            for (size_t i = 0; i < all_motors_.size(); ++i)
            {
                if (!confirmed[i] && all_motors_[i])
                {
                    ++attempts[i];
                    if (!all_motors_[i]->enable())
                    {
                        RCLCPP_WARN(this->get_logger(),
                            "[Activate] Enable resend TX failed: "
                            "motor_id=%u bus=%s attempt=%zu",
                            static_cast<unsigned>(motor_ids_[i]),
                            packet_index_to_bus_[i].c_str(),
                            attempts[i]);
                    }
                }
            }
            next_resend += ENABLE_RESEND_INTERVAL;
        }

        if (confirmed_count < all_motors_.size())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    if (confirmed_count == all_motors_.size())
    {
        return true;
    }

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        if (confirmed[i] || !all_motors_[i])
        {
            continue;
        }
        RCLCPP_ERROR(this->get_logger(),
            "[Activate] Enable confirmation timeout: "
            "motor_id=%u bus=%s baseline=%lu sequence=%lu "
            "run_state=%u fault=0x%02X attempts=%zu",
            static_cast<unsigned>(motor_ids_[i]),
            packet_index_to_bus_[i].c_str(),
            static_cast<unsigned long>(baseline_sequences[i]),
            static_cast<unsigned long>(all_motors_[i]->getFeedbackSequence()),
            static_cast<unsigned>(all_motors_[i]->getRunState()),
            static_cast<unsigned>(all_motors_[i]->getFaultFlags()),
            attempts[i]);
    }
    return false;
}

bool MainControlNode::sendZeroCommands()
{
    for (auto& group : can_groups_)
    {
        BusCommandData command;
        command.commands.reserve(group.motors.size());
        for (size_t local_idx = 0; local_idx < group.motors.size(); ++local_idx)
        {
            MotorCommandData motor_command;
            motor_command.motor_id =
                motor_ids_[group.global_packet_indices[local_idx]];
            command.commands.push_back(motor_command);
        }
        BusSendReport report;
        if (group.worker)
        {
            report = group.worker->sendCommandData(command);
        }
        else
        {
            report.result = WorkerResult::BusUnavailable;
            report.requested = group.motors.size();
            report.error_number = ENOTCONN;
            for (const auto& motor : group.motors)
            {
                if (motor) report.failed_motor_ids.push_back(motor->getMotorId());
            }
        }
        handleBusSendResult(group.interface_name, report, "ZERO");
    }
    return true;
}

void MainControlNode::control_loop()
{
    handle_read_packet();
    handle_write_packet();
}
// bool MainControlNode::isStartPositionReady(std::string* reason)
// {
//     if (all_motors_.empty())
//     {
//         if (reason) *reason = "all_motors_ is empty";
//         return false;
//     }

//     if (motor_feedback_seen_.size() != all_motors_.size() ||
//         last_valid_motor_pos_.size() != all_motors_.size())
//     {
//         if (reason) *reason = "init validation buffers size mismatch";
//         return false;
//     }

//     for (size_t i = 0; i < all_motors_.size(); ++i)
//     {
//         if (!motor_feedback_seen_[i])
//         {
//             if (reason)
//             {
//                 *reason = "motor " + std::to_string(i) +
//                           " valid feedback not seen yet";
//             }
//             return false;
//         }

//         const float q = last_valid_motor_pos_[i];

//         if (!std::isfinite(q))
//         {
//             if (reason)
//             {
//                 *reason = "motor " + std::to_string(i) +
//                           " q is not finite";
//             }
//             return false;
//         }

//         if (std::fabs(q) > INIT_Q_ABS_LIMIT)
//         {
//             if (reason)
//             {
//                 *reason = "motor " + std::to_string(i) +
//                           " q out of range: " + std::to_string(q);
//             }
//             return false;
//         }
//     }

//     if (reason) *reason = "ready";
//     return true;
// }

float MainControlNode::computeWrappedCommand(float current_raw_pos, float target_wrapped_pos) const
{
    float diff = target_wrapped_pos - wrapToPi(current_raw_pos);

    if (diff > static_cast<float>(M_PI))
        diff -= 2.0f * static_cast<float>(M_PI);
    if (diff < -static_cast<float>(M_PI))
        diff += 2.0f * static_cast<float>(M_PI);

    return current_raw_pos + diff;
}

void MainControlNode::handle_read_packet()
{
    static int fail_count = 0;
    static int success_count = 0;

    int cycle_fail_count = 0;
    int cycle_success_count = 0;

    std::vector<bool> current_cycle_updated(all_motors_.size(), false);

    for (auto& group : can_groups_)
    {
        if (!group.worker)
        {
            continue;
        }

        const BusStateData bus_state = group.worker->getStateSnapshot();
        for (const auto& state : bus_state.states)
        {
            const size_t index = state.global_packet_index;
            if (index >= latest_motor_states_.size())
            {
                continue;
            }

            if (state.updated && state.valid)
            {
                const bool new_feedback =
                    state.feedback_time >
                    latest_motor_states_[index].feedback_time;
                const bool new_status_feedback =
                    state.feedback_sequence >
                    latest_motor_states_[index].feedback_sequence;
                if (new_feedback)
                {
                    latest_motor_states_[index] = state;
                    current_cycle_updated[index] = true;
                }

                if (torque_enabled_ && new_status_feedback &&
                    state.fault_flags != 0)
                {
                    enterFrozen(
                        "[Safety] Motor fault feedback: motor_id=" +
                        std::to_string(state.motor_id) +
                        " bus=" + packet_index_to_bus_[index] +
                        " fault=" + std::to_string(state.fault_flags) +
                        " run_state=" + std::to_string(state.run_state));
                    return;
                }

                if (torque_enabled_ && new_status_feedback &&
                    state.run_state != MOTOR_RUN_STATE)
                {
                    enterFrozen(
                        "[Safety] Motor left enabled state: motor_id=" +
                        std::to_string(state.motor_id) +
                        " bus=" + packet_index_to_bus_[index] +
                        " run_state=" + std::to_string(state.run_state));
                    return;
                }
            }
        }
    }

    if (torque_enabled_ && init_phase_ != InitPhase::COLLECT_FEEDBACK &&
        policy_state_ != SystemPolicyState::FROZEN &&
        policy_state_ != SystemPolicyState::ESTOPPED)
    {
        const auto now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < latest_motor_states_.size(); ++i)
        {
            const auto stamp = latest_motor_states_[i].feedback_time;
            if (stamp == std::chrono::steady_clock::time_point{})
            {
                continue;
            }
            const auto age = now - stamp;
            if (age > FEEDBACK_FREEZE_TIMEOUT)
            {
                enterFrozen(
                    "[Safety] Feedback stale: motor_id=" +
                    std::to_string(motor_ids_[i]) +
                    " bus=" + packet_index_to_bus_[i] +
                    " age_ms=" + std::to_string(
                        std::chrono::duration_cast<std::chrono::milliseconds>(age).count()));
                break;
            }
            if (age > FEEDBACK_WARN_TIMEOUT)
            {
                policy_state_ = SystemPolicyState::DEGRADED;
                if (shouldEmitLog(
                    "feedback_delayed:" + std::to_string(motor_ids_[i]),
                    std::chrono::milliseconds(1000)))
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[Safety] Feedback delayed: motor_id=%u bus=%s age_ms=%ld",
                        static_cast<unsigned>(motor_ids_[i]),
                        packet_index_to_bus_[i].c_str(),
                        static_cast<long>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(age).count()));
                }
            }
        }
    }
    auto msg = roa_interfaces::msg::MotorStateArray();
    msg.states.resize(all_motors_.size());
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "motor_states";

    auto current_time = std::chrono::steady_clock::now();
    const float nominal_dt_sec = 1.0f / static_cast<float>(control_frequency_hz_);
    float dt_sec = nominal_dt_sec;
    if (velocity_filter_time_initialized_)
    {
        dt_sec = std::chrono::duration<float>(current_time - last_velocity_filter_time_).count();
    }
    if (dt_sec <= 0.0f || dt_sec > 0.1f)
    {
        dt_sec = nominal_dt_sec;
    }
    last_velocity_filter_time_ = current_time;
    velocity_filter_time_initialized_ = true;

    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        msg.states[i].motor_id = motor_ids_[i];

        if (current_cycle_updated[i])
        {
            ++success_count;
            ++cycle_success_count;

            const float raw_pos = latest_motor_states_[i].position;
            const float wrapped_pos = wrapToPi(raw_pos);
            const float raw_velocity = latest_motor_states_[i].velocity;
            const float current = latest_motor_states_[i].current;

            const bool valid_feedback =
                std::isfinite(raw_pos) &&
                std::isfinite(raw_velocity) &&
                std::isfinite(current) &&
                std::fabs(raw_pos) <= INIT_Q_ABS_LIMIT &&
                std::fabs(raw_velocity) <= INIT_VELOCITY_LIMIT;

            // ===== Init last valid q update =====
            // if (std::isfinite(raw_pos) && std::fabs(raw_pos) <= INIT_Q_ABS_LIMIT)
            // {
            //     if (i < motor_feedback_seen_.size())
            //     {
            //         motor_feedback_seen_[i] = true;
            //     }

            //     if (i < last_valid_motor_pos_.size())
            //     {
            //         last_valid_motor_pos_[i] = raw_pos;
            //     }
            // }
            // else
            // {
            //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //         "[InitValidation] Invalid q feedback ignored. motor_id=%u bus=%s raw_pos=%.6f",
            //         static_cast<unsigned>(motor_ids_[i]), packet_index_to_bus_[i].c_str(),
            //         raw_pos);
            // }
            if (init_phase_ == InitPhase::COLLECT_FEEDBACK)
            {
                if (valid_feedback)
                {
                    motor_feedback_seen_[i] = true;
                    last_valid_motor_pos_[i] = raw_pos;
                    last_feedback_time_[i] =
                        std::chrono::steady_clock::now();

                    auto& q_samples =
                        init_position_samples_[i];

                    auto& qdot_samples =
                        init_velocity_samples_[i];

                    q_samples.push_back(raw_pos);
                    qdot_samples.push_back(raw_velocity);

                    if (q_samples.size() >
                        INIT_REQUIRED_SAMPLES)
                    {
                        q_samples.pop_front();
                    }

                    if (qdot_samples.size() >
                        INIT_REQUIRED_SAMPLES)
                    {
                        qdot_samples.pop_front();
                    }
                }
                else
                {
                    if (shouldEmitLog(
                        "invalid_feedback:" + std::to_string(motor_ids_[i]),
                        std::chrono::milliseconds(1000)))
                    {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "[InitCollect] Invalid feedback: "
                            "motor_id=%u bus=%s q=%.6f qdot=%.6f current=%.6f",
                            static_cast<unsigned>(motor_ids_[i]),
                            packet_index_to_bus_[i].c_str(),
                            raw_pos,
                            raw_velocity,
                            current);
                    }

                    // 연속 정상 샘플을 요구한다면 비정상값 발생 시 초기화
                    init_position_samples_[i].clear();
                    init_velocity_samples_[i].clear();
                    motor_feedback_seen_[i] = false;
                }
            }

            float velocity = raw_velocity;
            if (i < velocity_filters_.size())
            {
                velocity = velocity_filters_[i].filter(raw_velocity, dt_sec);
            }

            msg.states[i].position = wrapped_pos;
            msg.states[i].velocity = velocity;
            msg.states[i].current  = current;
        }
        else
        {
            ++fail_count;
            ++cycle_fail_count;

            // 마지막 정상값 유지
            msg.states[i].position = wrapToPi(latest_motor_states_[i].position);
            {
                const float raw_velocity = latest_motor_states_[i].velocity;
                float velocity = raw_velocity;
                if (i < velocity_filters_.size())
                {
                    velocity = velocity_filters_[i].filter(raw_velocity, dt_sec);
                }
                msg.states[i].velocity = velocity;
            }
            msg.states[i].current = latest_motor_states_[i].current;

            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //     "[Read] motor_id=%u bus=%s update failed (success=%d fail=%d)",
            //     msg.states[i].motor_id,
            //     packet_index_to_bus_[i].c_str(),
            //     success_count,
            //     fail_count);
        }
    }

    if(cycle_fail_count != 0)
    {
        last_read_cycle_all_updated_ = false;
    }
    else
    {
        last_read_cycle_all_updated_ = true;
    }

    if (init_phase_ == InitPhase::COLLECT_FEEDBACK)
    {
        const InitSampleCheckReport sample_report = checkInitialSamples();
        const std::string detail = formatInitSampleReport(sample_report);

        if (sample_report.result == InitSampleCheckResult::Fatal)
        {
            enterFrozen(
                "[Init] Initial sample validation failed: " +
                detail);

            return;
        }

        if (sample_report.result == InitSampleCheckResult::Ready)
        {
            start_positions_.resize(all_motors_.size());

            for (size_t i = 0;
                i < all_motors_.size();
                ++i)
            {
                const float median_position =
                    computeMedian(
                        init_position_samples_[i]);

                // 중앙값 자체도 마지막으로 다시 검사
                if (!std::isfinite(median_position) ||
                    std::fabs(median_position) >
                        INIT_Q_ABS_LIMIT)
                {
                    enterFrozen(
                        "[Init] Invalid median position: " +
                        formatMotorContext(i) +
                        " median=" +
                        std::to_string(median_position));

                    return;
                }

                start_positions_[i] =
                    median_position;

                RCLCPP_INFO(
                    this->get_logger(),
                    "[Init] Start position validated: "
                    "motor_id=%u bus=%s "
                    "median=%.6f samples=%zu",
                    static_cast<unsigned>(motor_ids_[i]),
                    packet_index_to_bus_[i].c_str(),
                    median_position,
                    init_position_samples_[i].size());
            }

            start_positions_captured_ = true;
            init_phase_ = InitPhase::WAIT_COMMAND;
            interpolation_cycle_count_ = 0;
            interpolation_start_time_ = {};

            RCLCPP_INFO(
                this->get_logger(),
                "[Init] All initial positions validated: motor_ids=%s; "
                "waiting for upper command.",
                formatMotorIdList(motor_ids_).c_str());
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[Init] Collecting feedback: %s",
                detail.c_str());
        }

        const auto elapsed =
            std::chrono::steady_clock::now() -
            init_phase_start_time_;

        if (elapsed >
            std::chrono::milliseconds(
                INIT_COLLECTION_TIMEOUT_MS))
        {
            enterFrozen(
                "[Init] Initial feedback collection timeout: " +
                detail);

            return;
        }
    }

    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "stabilzation_ : %s, cycle_fali_count : %d", last_read_cycle_all_updated_ ? "true" : "false", cycle_fail_count);
    state_pub->publish(msg);
}

void MainControlNode::logWriteSummaryThrottle()
{
    const auto now = std::chrono::steady_clock::now();
    for (auto& group : can_groups_)
    {
        if (!group.worker)
        {
            continue;
        }
        if (group.last_write_summary_log !=
                std::chrono::steady_clock::time_point{} &&
            now - group.last_write_summary_log < std::chrono::seconds(5))
        {
            continue;
        }
        group.last_write_summary_log = now;
        const BusWriteStats stats =
            group.worker->getWriteStatsSnapshot();
        const std::string motor_ids =
            formatMotorIdList(getGroupMotorIds(group));
        RCLCPP_INFO(
            this->get_logger(),
            "[WriteSummary] "
            "motor_ids=%s bus=%s "
            "ok=%lu fail=%lu "
            "eagain_total=%lu "
            "eagain_cycles=%u "
            "enobufs_total=%lu "
            "enobufs_cycles=%u",
            motor_ids.c_str(), group.interface_name.c_str(),
            static_cast<unsigned long>(stats.ok_writes),
            static_cast<unsigned long>(stats.fail_writes),
            static_cast<unsigned long>(stats.eagain_count),
            stats.consecutive_eagain_cycles,
            static_cast<unsigned long>(stats.enobufs_count),
            stats.consecutive_enobufs_cycles);
    }
}

void MainControlNode::handle_write_packet()
{
    if (policy_state_ == SystemPolicyState::FROZEN ||
        policy_state_ == SystemPolicyState::ESTOPPED ||
        !torque_enabled_)
    {
        sendSafetyDisableBatches();
        return;
    }

    if (init_phase_ == InitPhase::COLLECT_FEEDBACK)
    {
        sendZeroCommands();
        return;
    }

    if (init_phase_ == InitPhase::WAIT_COMMAND)
    {
        bool command_ready = false;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_ready = packet_initialized_;
        }

        if (!command_ready)
        {
            sendZeroCommands();
            return;
        }

        interpolation_cycle_count_ = 0;
        interpolation_start_time_ = std::chrono::steady_clock::now();
        init_phase_ = InitPhase::INTERPOLATING;
        RCLCPP_INFO(this->get_logger(),
            "[Init] Upper command available. Starting %.3f s interpolation.",
            initial_interpolation_duration_sec_);
    }

    roa_interfaces::msg::MotorCommandArray command_snapshot;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!packet_initialized_)
        {
            return;
        }
        command_snapshot = packet_commands_;
    }

    float alpha = 0.0f;
    if (init_phase_ == InitPhase::INTERPOLATING)
    {
        ++interpolation_cycle_count_;
        const std::chrono::duration<double> elapsed =
            std::chrono::steady_clock::now() - interpolation_start_time_;
        alpha = static_cast<float>(std::clamp(
            elapsed.count() / initial_interpolation_duration_sec_,
            0.0,
            1.0));
    }

    for (auto& group : can_groups_)
    {
        BusCommandData bus_command;
        bus_command.commands.reserve(group.global_packet_indices.size());

        for (const size_t packet_index : group.global_packet_indices)
        {
            if (packet_index >= command_snapshot.commands.size() ||
                packet_index >= latest_motor_states_.size())
            {
                requestFatalShutdown(
                    "[InternalInvariant] Invalid packet_index=" +
                    std::to_string(packet_index) +
                    " command_size=" +
                    std::to_string(command_snapshot.commands.size()) +
                    " state_size=" +
                    std::to_string(latest_motor_states_.size()));
                return;
            }

            const auto& cmd = command_snapshot.commands[packet_index];
            float command_pos = wrapToPi(cmd.position);

            if (init_phase_ == InitPhase::INTERPOLATING)
            {
                const float start_raw = start_positions_[packet_index];
                float diff = command_pos - wrapToPi(start_raw);
                if (diff > static_cast<float>(M_PI))
                {
                    diff -= 2.0f * static_cast<float>(M_PI);
                }
                if (diff < -static_cast<float>(M_PI))
                {
                    diff += 2.0f * static_cast<float>(M_PI);
                }
                command_pos = start_raw + alpha * diff;
            }
            else if (init_phase_ == InitPhase::RUNNING)
            {
                command_pos = computeWrappedCommand(
                    latest_motor_states_[packet_index].position,
                    command_pos);
            }

            MotorCommandData worker_command;
            worker_command.motor_id = cmd.motor_id;
            worker_command.torque = cmd.torque;
            worker_command.position = command_pos;
            worker_command.velocity = cmd.velocity;
            worker_command.kp = cmd.kp;
            worker_command.kd = cmd.kd;
            bus_command.commands.push_back(worker_command);
        }

        if (!group.worker)
        {
            const std::string motor_ids =
                formatMotorIdList(getGroupMotorIds(group));
            requestFatalShutdown(
                "[Write] Missing CAN worker: motor_ids=" + motor_ids +
                " bus=" + group.interface_name);
            return;
        }
        const BusSendReport report =
            group.worker->sendCommandData(bus_command);
        handleBusSendResult(group.interface_name, report, "COMMAND");
    }

    if (init_phase_ == InitPhase::INTERPOLATING && alpha >= 1.0f)
    {
        const std::chrono::duration<double> actual_elapsed =
            std::chrono::steady_clock::now() - interpolation_start_time_;
        init_phase_ = InitPhase::RUNNING;
        walk_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[Init] initialization interpolation completed after %.3f s "
            "(%lu control cycles)",
            actual_elapsed.count(),
            static_cast<unsigned long>(interpolation_cycle_count_));
    }

    logWriteSummaryThrottle();
}
void MainControlNode::publishWalkInitialized(bool initialized)
{
    std_msgs::msg::Bool msg;
    msg.data = initialized;

    initial_pub->publish(msg);
}

void MainControlNode::walkCallback(
    const roa_interfaces::msg::MotorCommandArray::SharedPtr msg)
{

    publishWalkInitialized(walk_initialized_);
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        packet_commands_.header = msg->header;

        for (const auto& cmd : msg->commands)
        {
            const auto it =
                motor_id_to_index_.find(
                    cmd.motor_id);

            if (it ==
                motor_id_to_index_.end())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[ros_sub] Unknown motor_id: %u",
                    cmd.motor_id);

                continue;
            }

            const size_t packet_index =
                it->second;

            packet_commands_.commands[
                packet_index] = cmd;
        }

        packet_initialized_ = true;
    }

    if (init_phase_ == InitPhase::WAIT_COMMAND &&
        start_positions_captured_)
    {
        interpolation_cycle_count_ = 0;
        interpolation_start_time_ = std::chrono::steady_clock::now();
        init_phase_ = InitPhase::INTERPOLATING;

        RCLCPP_INFO(
            this->get_logger(),
            "[Init] First upper command received. "
            "Starting %.3f s interpolation.",
            initial_interpolation_duration_sec_);
    }
}

void MainControlNode::torqueCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
    {
        torque_enabled_ = false;
        policy_state_ = SystemPolicyState::ESTOPPED;
        walk_initialized_ = false;
        sendSafetyDisableBatches();
        const std::string motor_ids = formatMotorIdList(motor_ids_);
        RCLCPP_ERROR(this->get_logger(),
            "[EStop] Entered ESTOPPED state: motor_ids=%s. Process remains alive; "
            "deactivate/activate lifecycle transition is required to resume.",
            motor_ids.c_str());
        return;
    }

    RCLCPP_WARN(this->get_logger(),
        "[EStop] Enable request ignored while safety state is latched. "
        "Use a fresh lifecycle deactivate/activate transition.");
}

void MainControlNode::enterFrozen(const std::string& reason)
{
    if (policy_state_ == SystemPolicyState::FROZEN ||
        policy_state_ == SystemPolicyState::ESTOPPED)
    {
        return;
    }
    policy_state_ = SystemPolicyState::FROZEN;
    torque_enabled_ = false;
    walk_initialized_ = false;
    RCLCPP_ERROR(this->get_logger(),
        "[Frozen] %s. Upper commands are ignored; lifecycle reactivation is required.",
        reason.c_str());
    sendSafetyDisableBatches();
}

void MainControlNode::handleBusSendResult(
    const std::string& bus,
    const BusSendReport& report,
    const char* operation)
{
    if (report.result == WorkerResult::Success)
    {
        tx_degraded_since_.erase(bus);
        if (tx_degraded_since_.empty() &&
            policy_state_ == SystemPolicyState::DEGRADED)
        {
            policy_state_ = SystemPolicyState::ACTIVE;
        }
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto inserted = tx_degraded_since_.emplace(bus, now);
    policy_state_ = SystemPolicyState::DEGRADED;
    const std::string failed_ids =
        formatMotorIdList(report.failed_motor_ids);
    if (shouldEmitLog(
        "tx:" + std::string(operation) + ':' + bus,
        std::chrono::milliseconds(1000)))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[CAN TX] %s batch incomplete: failed_motor_ids=%s bus=%s "
            "queued=%zu/%zu errno=%d (%s)",
            operation, failed_ids.c_str(), bus.c_str(),
            report.queued, report.requested, report.error_number,
            report.error_number != 0
                ? std::strerror(report.error_number) : "none");
    }

    if (!inserted.second &&
        now - inserted.first->second > TX_DEGRADED_FREEZE_TIMEOUT)
    {
        enterFrozen(
            "[Safety] Persistent CAN TX degradation: failed_motor_ids=" +
            failed_ids + " bus=" + bus +
            " operation=" + operation);
    }
}

void MainControlNode::sendSafetyDisableBatches()
{
    const auto now = std::chrono::steady_clock::now();
    if (last_safety_disable_time_ !=
            std::chrono::steady_clock::time_point{} &&
        now - last_safety_disable_time_ < SAFETY_DISABLE_INTERVAL)
    {
        return;
    }
    last_safety_disable_time_ = now;
    for (auto& group : can_groups_)
    {
        if (!group.worker)
        {
            const std::string motor_ids =
                formatMotorIdList(getGroupMotorIds(group));
            if (shouldEmitLog(
                "safety_disable:" + group.interface_name,
                std::chrono::milliseconds(1000)))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[Safety] Disable batch failed: "
                    "motor_ids=%s bus=%s reason=missing_worker",
                    motor_ids.c_str(), group.interface_name.c_str());
            }
            continue;
        }
        const BusSendReport report = group.worker->sendDisableAll();
        if (report.result != WorkerResult::Success)
        {
            const std::string failed_ids =
                formatMotorIdList(report.failed_motor_ids);
            if (shouldEmitLog(
                "safety_disable:" + group.interface_name,
                std::chrono::milliseconds(1000)))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[Safety] Disable batch incomplete: failed_motor_ids=%s "
                    "bus=%s queued=%zu/%zu errno=%d (%s)",
                    failed_ids.c_str(), group.interface_name.c_str(),
                    report.queued, report.requested, report.error_number,
                    report.error_number != 0
                        ? std::strerror(report.error_number) : "none");
            }
        }
    }
}

// 디버그용: CAN 수신 큐 플러시
void MainControlNode::flushCanRxQueues(const char* tag)
{
    uint32_t rx_id = 0;
    std::vector<uint8_t> rx_data;

    for (auto& group : can_groups_)
    {
        const std::string motor_ids =
            formatMotorIdList(getGroupMotorIds(group));
        int flushed = 0;

        while (group.transport && group.transport->receive(rx_id, rx_data, 0))
        {
            flushed++;
        }

        RCLCPP_WARN(this->get_logger(),
            "[CAN Flush:%s] motor_ids=%s bus=%s flushed=%d pending_rx_frames",
            tag,
            motor_ids.c_str(),
            group.interface_name.c_str(),
            flushed);
    }
}

void MainControlNode::requestFatalShutdown(
    const std::string& reason)
{
    RCLCPP_FATAL(
        this->get_logger(),
        "[FatalShutdown] %s",
        reason.c_str());

    // 제어 루프가 이미 실행 중인 경우 중지
    timer_.reset();

    stopWorkers();

    // 생성된 모터가 있다면 가능한 범위에서 Disable
    disableAllMotors();

    // executor의 spin()을 종료시킴
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

// 메인 함수
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MainControlNode>();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
