#include "robstride_rdk_ros2/MainControl.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>

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
    declare_parameter<std::vector<std::string>>("can_interfaces", {"can0"});

    const auto can_interfaces = get_parameter("can_interfaces").as_string_array();

    for (const auto& can_name : can_interfaces)
    {
        declare_parameter<std::vector<int64_t>>(can_name + ".motor_ids", std::vector<int64_t>{});
        declare_parameter<std::vector<int64_t>>(can_name + ".motor_type", std::vector<int64_t>{});
    }

    RCLCPP_INFO(this->get_logger(), "[Configure] Parameters initialized");
    RCLCPP_INFO(this->get_logger(), "[Configure] baud_rate: %ld", get_parameter("baud_rate").as_int());
    RCLCPP_INFO(this->get_logger(), "[Configure] can_interfaces count: %zu", can_interfaces.size());

    for (const auto& can_name : can_interfaces)
    {
        auto ids = get_parameter(can_name + ".motor_ids").as_integer_array();
        RCLCPP_INFO(this->get_logger(), "[Configure] %s: %zu motors", can_name.c_str(), ids.size());
    }
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

bool MainControlNode::processReceivedFrame(
    CanBusGroup& group,
    uint32_t rx_id,
    const std::vector<uint8_t>& rx_data,
    const char* phase)
{
    const uint8_t motor_id =
        RobStrideProtocol::getMotorIdFromCanId(rx_id);

    const uint8_t type =
        RobStrideProtocol::getTypeFromCanId(rx_id);

    for (size_t local_idx = 0;
         local_idx < group.motors.size();
         ++local_idx)
    {
        auto& motor = group.motors[local_idx];

        if (motor->getMotorId() != motor_id)
        {
            continue;
        }

        if (local_idx >= group.global_packet_indices.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] Invalid local index mapping: "
                "bus=%s local_idx=%zu",
                phase,
                group.interface_name.c_str(),
                local_idx);

            return false;
        }

        const size_t packet_index =
            group.global_packet_indices[local_idx];

        if (packet_index >= all_motors_.size() ||
            packet_index >= motor_feedback_seen_.size() ||
            packet_index >= last_valid_motor_pos_.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] Invalid global packet index: "
                "bus=%s packet_index=%zu",
                phase,
                group.interface_name.c_str(),
                packet_index);

            return false;
        }

        try
        {
            motor->processPacket(rx_id, rx_data);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] Invalid motor frame: "
                "bus=%s motor_id=%u type=%u "
                "can_id=0x%08X size=%zu reason=%s",
                phase,
                group.interface_name.c_str(),
                static_cast<unsigned>(motor_id),
                static_cast<unsigned>(type),
                rx_id,
                rx_data.size(),
                e.what());

            return false;
        }

        const float q =
            static_cast<float>(motor->getPosition());

        const float qdot =
            static_cast<float>(motor->getVelocity());

        const float current =
            static_cast<float>(motor->getCurrent());

        if (!std::isfinite(q) ||
            !std::isfinite(qdot) ||
            !std::isfinite(current))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] Non-finite motor feedback: "
                "bus=%s motor_id=%u "
                "q=%.6f qdot=%.6f current=%.6f",
                phase,
                group.interface_name.c_str(),
                static_cast<unsigned>(motor_id),
                q,
                qdot,
                current);

            return false;
        }

        if (std::fabs(q) > INIT_Q_ABS_LIMIT)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] Position out of range: "
                "bus=%s motor_id=%u q=%.6f limit=%.6f",
                phase,
                group.interface_name.c_str(),
                static_cast<unsigned>(motor_id),
                q,
                INIT_Q_ABS_LIMIT);

            return false;
        }

        const bool first_feedback =
            !motor_feedback_seen_[packet_index];

        motor_feedback_seen_[packet_index] = true;
        last_valid_motor_pos_[packet_index] = q;

        if (first_feedback)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] Motor feedback confirmed: "
                "bus=%s motor_id=%u packet_index=%zu "
                "q=%.6f qdot=%.6f current=%.6f",
                phase,
                group.interface_name.c_str(),
                static_cast<unsigned>(motor_id),
                packet_index,
                q,
                qdot,
                current);
        }

        return true;
    }

    RCLCPP_WARN(
        this->get_logger(),
        "[%s] Frame from unknown motor: "
        "bus=%s motor_id=%u type=%u can_id=0x%08X",
        phase,
        group.interface_name.c_str(),
        static_cast<unsigned>(motor_id),
        static_cast<unsigned>(type),
        rx_id);

    return false;
}

bool MainControlNode::verifyInitialMotorFeedback(
    std::chrono::milliseconds timeout)
{
    if (all_motors_.empty())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[ActivateVerify] No motors configured");

        return false;
    }

    if (motor_feedback_seen_.size() != all_motors_.size() ||
        last_valid_motor_pos_.size() != all_motors_.size())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[ActivateVerify] Validation buffer size mismatch: "
            "motors=%zu seen=%zu positions=%zu",
            all_motors_.size(),
            motor_feedback_seen_.size(),
            last_valid_motor_pos_.size());

        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline)
    {
        bool received_any_frame = false;

        for (auto& group : can_groups_)
        {
            if (!group.transport ||
                !group.transport->isOpen())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[ActivateVerify] CAN transport unavailable: "
                    "bus=%s",
                    group.interface_name.c_str());

                return false;
            }

            uint32_t rx_id = 0;
            std::vector<uint8_t> rx_data;

            constexpr int MAX_FRAMES_PER_BUS_PER_PASS = 100;
            int received_count = 0;

            while (received_count <
                       MAX_FRAMES_PER_BUS_PER_PASS &&
                   group.transport->receive(
                       rx_id,
                       rx_data,
                       0))
            {
                ++received_count;
                received_any_frame = true;

                processReceivedFrame(
                    group,
                    rx_id,
                    rx_data,
                    "ActivateVerify");
            }
        }

        bool all_confirmed = true;

        for (size_t i = 0;
             i < motor_feedback_seen_.size();
             ++i)
        {
            if (!motor_feedback_seen_[i])
            {
                all_confirmed = false;
                break;
            }
        }

        if (all_confirmed)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[ActivateVerify] All %zu motors "
                "provided valid feedback",
                all_motors_.size());

            return true;
        }

        if (!received_any_frame)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
        }
    }

    RCLCPP_ERROR(
        this->get_logger(),
        "[ActivateVerify] Feedback verification timeout: "
        "%ld ms",
        static_cast<long>(timeout.count()));

    for (size_t i = 0;
         i < all_motors_.size();
         ++i)
    {
        if (motor_feedback_seen_[i])
        {
            continue;
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "[ActivateVerify] Missing feedback: "
            "packet_index=%zu bus=%s motor_id=%u",
            i,
            i < packet_index_to_bus_.size()
                ? packet_index_to_bus_[i].c_str()
                : "unknown",
            static_cast<unsigned>(
                all_motors_[i]->getMotorId()));
    }

    return false;
}

void MainControlNode::disableAllMotors()
{
    RCLCPP_WARN(
        this->get_logger(),
        "[Safety] Sending disable command to all motors");

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
                "packet_index=%zu bus=%s motor_id=%u",
                i,
                i < packet_index_to_bus_.size()
                    ? packet_index_to_bus_[i].c_str()
                    : "unknown",
                static_cast<unsigned>(
                    all_motors_[i]->getMotorId()));
        }
    }
}

// ------------------------ Lifecycle Callbacks ------------------------

void MainControlNode::resetRuntimeStates()
{
    walk_initialized_ = false;
    start_positions_captured_ = false;
    start_position_init_attempted_ = false;

    init_tick_count_ = 0;
    start_positions_.clear();
    last_read_cycle_all_updated_ = false;

    packet_initialized_ = false;
    initial_raw_position_printed_ = false;

    motor_feedback_seen_.assign(all_motors_.size(), false);
    last_valid_motor_pos_.assign(all_motors_.size(), 0.0f);

    for (auto& group : can_groups_)
    {
        group.write_stats = BusWriteStats{};
    }

    for (auto& motor_stat : motor_write_stats_)
    {
        motor_stat = MotorWriteStats{};
    }
}

CallbackReturn MainControlNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[Configure] Configuring...");

    initParameters();

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
                "[Configure] %s: motor_ids and motor_type size mismatch" +
                can_name + "'"
            );
            // RCLCPP_ERROR(this->get_logger(),
            //     "[Configure] %s: motor_ids and motor_type size mismatch",
            //     can_name.c_str());
            return CallbackReturn::FAILURE;
        }

        for (size_t i = 0; i < motor_ids.size(); ++i)
        {
            const auto type = static_cast<ActuatorType>(motor_types[i]);
            const auto id = static_cast<uint8_t>(motor_ids[i]);

            auto motor = std::make_shared<RobStrideMotor>(group.transport, id, type);
            group.motors.push_back(motor);
            all_motors_.push_back(motor);

            const size_t packet_index = all_motors_.size() - 1;
            group.global_packet_indices.push_back(packet_index);
            packet_index_to_bus_.push_back(can_name);

            RCLCPP_INFO(this->get_logger(),
                "[Configure] %s Motor[%zu] initialized - ID: %u, Type: %ld",
                can_name.c_str(), i, id, motor_types[i]);
        }

        can_groups_.push_back(std::move(group));
    }

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        const uint16_t id = static_cast<uint16_t>(all_motors_[i]->getMotorId());

        if (motor_id_to_index_.find(id) != motor_id_to_index_.end())
        {
            requestFatalShutdown(
                "[Configure] Duplicate motor_id detected: " +
                std::to_string(id));
            // RCLCPP_ERROR(this->get_logger(),
            //     "[Configure] Duplicate motor_id detected: %u", id);
            return CallbackReturn::FAILURE;
        }

        motor_id_to_index_[id] = i;

        RCLCPP_INFO(this->get_logger(),
            "[Configure] Packet mapper: motor_id=%u -> packet_index=%zu (bus=%s)",
            id, i, packet_index_to_bus_[i].c_str());
    }

    packet_commands_.commands.resize(all_motors_.size());
    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        const uint16_t id = static_cast<uint16_t>(all_motors_[i]->getMotorId());
        packet_commands_.commands[i].motor_id = id;
        packet_commands_.commands[i].torque   = 0.0f;
        packet_commands_.commands[i].position = 0.0f;
        packet_commands_.commands[i].velocity = 0.0f;
        packet_commands_.commands[i].kp       = 0.0f;
        packet_commands_.commands[i].kd       = 0.0f;
    }

    motor_write_stats_.assign(all_motors_.size(), MotorWriteStats{});
    resetRuntimeStates();

    RCLCPP_INFO(this->get_logger(),
        "[Configure] Configured successfully with %zu CAN interfaces, %zu total motors",
        can_groups_.size(), all_motors_.size());

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

CallbackReturn MainControlNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[Activate] Activating...");

    resetRuntimeStates();
    flushCanRxQueues("before_enable");

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        if (!all_motors_[i]->enable())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[Activate] Enable command TX failed: "
                "packet_index=%zu bus=%s motor_id=%u",
                i,
                i < packet_index_to_bus_.size()
                    ? packet_index_to_bus_[i].c_str()
                    : "unknown",
                static_cast<unsigned>(
                    all_motors_[i]->getMotorId()));
            // disableAllMotors();
            requestFatalShutdown(
                "[Activate] Enable command TX failed: "
                "packet_index=" + std::to_string(i) +
                " bus=" + (i < packet_index_to_bus_.size() ? packet_index_to_bus_[i] : "unknown") +
                " motor_id=" + std::to_string(all_motors_[i]->getMotorId()));

            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[Activate] Enable command sent: "
            "packet_index=%zu bus=%s motor_id=%u",
            i,
            i < packet_index_to_bus_.size()
                ? packet_index_to_bus_[i].c_str()
                : "unknown",
            static_cast<unsigned>(
                all_motors_[i]->getMotorId()));        
    }

    // Verify Initial Motor Feedback 
    constexpr auto activation_timeout =
        std::chrono::milliseconds(300);

    if (!verifyInitialMotorFeedback(
            activation_timeout))
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "[Activate] Motor activation verification failed. "
            "Control loop will not start.");

        disableAllMotors();

        // Disable 응답은 현재 해석하지 않으므로
        // 잠시 후 남은 응답만 정리
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));

        flushCanRxQueues(
            "activate_failure_after_disable");
        requestFatalShutdown(
            "[Activate] Motor activation verification failed. "
            "Control loop will not start.");

        return CallbackReturn::FAILURE;
    }

    // initial filter 
    velocity_filters_.clear();
    velocity_filters_.reserve(all_motors_.size());
    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        velocity_filters_.emplace_back(3.0f); // 23 hz -> 10 hz로 낮춰서 더 부드럽게
    }
    last_velocity_filter_time_ = std::chrono::steady_clock::now();
    velocity_filter_time_initialized_ = false;


    current_state = ControlState::READ_PACKET;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5), // 50 hz timer
        std::bind(&MainControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
        "[Activate] Activated successfully with %zu motors",
        all_motors_.size());

    return CallbackReturn::SUCCESS;
}

void MainControlNode::control_loop()
{
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    // "[Loop] current_state=%s",
    // (current_state == ControlState::READ_PACKET ? "READ" : "WRITE"));

    static auto last_print_time = std::chrono::steady_clock::now();
    static int write_count = 0;

    if (current_state == ControlState::WRITE_PACKET)
    {
        ++write_count;

        const auto current_time = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = current_time - last_print_time;

        if (elapsed.count() >= 1.0)
        {
            // const double hz = write_count / elapsed.count();
            // RCLCPP_INFO(this->get_logger(), "Motor Control Rate: %.2f Hz", hz);
            last_print_time = current_time;
            write_count = 0;
        }
    }

    switch (current_state)
    {
        case ControlState::WRITE_PACKET:
            handle_write_packet();
            break;
        case ControlState::READ_PACKET:
            handle_read_packet();
            break;
    }
}
bool MainControlNode::isStartPositionReady(std::string* reason)
{
    if (all_motors_.empty())
    {
        if (reason) *reason = "all_motors_ is empty";
        return false;
    }

    if (motor_feedback_seen_.size() != all_motors_.size() ||
        last_valid_motor_pos_.size() != all_motors_.size())
    {
        if (reason) *reason = "init validation buffers size mismatch";
        return false;
    }

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        if (!motor_feedback_seen_[i])
        {
            if (reason)
            {
                *reason = "motor " + std::to_string(i) +
                          " valid feedback not seen yet";
            }
            return false;
        }

        const float q = last_valid_motor_pos_[i];

        if (!std::isfinite(q))
        {
            if (reason)
            {
                *reason = "motor " + std::to_string(i) +
                          " q is not finite";
            }
            return false;
        }

        if (std::fabs(q) > INIT_Q_ABS_LIMIT)
        {
            if (reason)
            {
                *reason = "motor " + std::to_string(i) +
                          " q out of range: " + std::to_string(q);
            }
            return false;
        }
    }

    if (reason) *reason = "ready";
    return true;
}

void MainControlNode::transition_to(ControlState new_state)
{
    current_state = new_state;
}

const char* MainControlNode::toString(
    WriteResult result) const
{
    switch (result)
    {
        case WriteResult::Ok:
            return "Ok";

        case WriteResult::TryAgain:
            return "TryAgain";

        case WriteResult::NoBuffer:
            return "NoBuffer";

        case WriteResult::BusDown:
            return "BusDown";

        case WriteResult::IoError:
            return "IoError";

        case WriteResult::InvalidArg:
            return "InvalidArg";

        default:
            return "Unknown";
    }
}

// const char* MainControlNode::toString(WriteResult result) const
// {
//     switch (result)
//     {
//         case WriteResult::Ok:         return "Ok";
//         case WriteResult::WouldBlock: return "WouldBlock";
//         case WriteResult::BusDown:    return "BusDown";
//         case WriteResult::IoError:    return "IoError";
//         case WriteResult::InvalidArg: return "InvalidArg";
//         default:                      return "Unknown";
//     }
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

WriteResult MainControlNode::safeSendCommand(
    RobStrideMotor& motor,
    float torque,
    float position,
    float velocity,
    float kp,
    float kd)
{
    errno = 0;

    const bool ok = motor.sendMotionCommand(
        torque,
        position,
        velocity,
        kp,
        kd);

    if (ok)
    {
        return WriteResult::Ok;
    }

    const int saved_errno = errno;

    if (saved_errno == EAGAIN ||
        saved_errno == EWOULDBLOCK)
    {
        return WriteResult::TryAgain;
    }

    if (saved_errno == ENOBUFS)
    {
        return WriteResult::NoBuffer;
    }

    if (saved_errno == ENETDOWN ||
        saved_errno == ENODEV)
    {
        return WriteResult::BusDown;
    }

    if (saved_errno == EINVAL)
    {
        return WriteResult::InvalidArg;
    }

    return WriteResult::IoError;
}

void MainControlNode::printInitialRawPositionsOnce(const char* tag)
{
    if (initial_raw_position_printed_)
    {
        return;
    }

    initial_raw_position_printed_ = true;

    RCLCPP_WARN(this->get_logger(),
        "==================== [Initial Raw Position Debug: %s] ====================",
        tag);

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        const float raw_q = static_cast<float>(all_motors_[i]->getPosition());
        const float wrapped_q = wrapToPi(raw_q);
        const float turns = raw_q / (2.0f * static_cast<float>(M_PI));

        const bool seen =
            (i < motor_feedback_seen_.size()) ? motor_feedback_seen_[i] : false;

        const float last_valid_q =
            (i < last_valid_motor_pos_.size()) ? last_valid_motor_pos_[i] : 0.0f;

        const float last_valid_wrapped = wrapToPi(last_valid_q);
        const float last_valid_turns =
            last_valid_q / (2.0f * static_cast<float>(M_PI));

        RCLCPP_WARN(this->get_logger(),
            "[InitialRawQ] packet_index=%zu motor_id=%u bus=%s seen=%s "
            "raw_q=%.6f wrapped_q=%.6f turns=%.3f "
            "last_valid_q=%.6f last_valid_wrapped=%.6f last_valid_turns=%.3f",
            i,
            all_motors_[i]->getMotorId(),
            (i < packet_index_to_bus_.size() ? packet_index_to_bus_[i].c_str() : "unknown"),
            seen ? "true" : "false",
            raw_q,
            wrapped_q,
            turns,
            last_valid_q,
            last_valid_wrapped,
            last_valid_turns);
    }

    RCLCPP_WARN(this->get_logger(),
        "==========================================================================");
}

void MainControlNode::handle_read_packet()
{
    static int fail_count = 0;
    static int success_count = 0;

    int cycle_fail_count = 0;
    int cycle_success_count = 0;

    std::vector<bool> current_cycle_updated(all_motors_.size(), false);

    uint32_t rx_id = 0;
    std::vector<uint8_t> rx_data;

    constexpr int MAX_RX_PACKETS_PER_GROUP = 50;
    constexpr auto MAX_RX_TIME = std::chrono::microseconds(1000);

    auto rx_start_time = std::chrono::steady_clock::now();

    for (auto& group : can_groups_)
    {
        int recv_count = 0;

        while (recv_count < MAX_RX_PACKETS_PER_GROUP &&
            group.transport->receive(rx_id, rx_data, 0))
        {
            recv_count++;

            if (std::chrono::steady_clock::now() - rx_start_time > MAX_RX_TIME)
            {
                RCLCPP_WARN(this->get_logger(),
                    "CAN RX time limit reached. recv_count=%d", recv_count);
                break;
            }

            const uint8_t motor_id = RobStrideProtocol::getMotorIdFromCanId(rx_id);

            for (size_t local_idx = 0; local_idx < group.motors.size(); ++local_idx)
            {
                auto& motor = group.motors[local_idx];

                if (motor->getMotorId() != motor_id)
                {
                    continue;
                }

                // if (motor->processPacket(rx_id, rx_data))
                // {
                //     if (local_idx < group.global_packet_indices.size())
                //     {
                //         const size_t packet_index = group.global_packet_indices[local_idx];

                //         if (packet_index < current_cycle_updated.size())
                //         {
                //             current_cycle_updated[packet_index] = true;
                //         }
                //     }
                // }
                try
                {
                    motor->processPacket(rx_id, rx_data);

                    if (local_idx < group.global_packet_indices.size())
                    {
                        const size_t packet_index = group.global_packet_indices[local_idx];

                        if (packet_index < current_cycle_updated.size())
                        {
                            current_cycle_updated[packet_index] = true;
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "[Read] Error processing CAN packet: "
                        "bus=%s motor_id=%u can_id=0x%08X reason=%s",
                        group.interface_name.c_str(),
                        static_cast<unsigned>(motor_id),
                        rx_id,
                        e.what());
                }
                break;
            }
        }

        if (recv_count >= MAX_RX_PACKETS_PER_GROUP)
        {
            RCLCPP_WARN(this->get_logger(),
                "CAN RX packet limit reached. recv_count=%d", recv_count);
        }
    }
    auto msg = roa_interfaces::msg::MotorStateArray();
    msg.states.resize(all_motors_.size());
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "motor_states";

    auto current_time = std::chrono::steady_clock::now();
    float dt_sec = 0.01f;
    if (velocity_filter_time_initialized_)
    {
        dt_sec = std::chrono::duration<float>(current_time - last_velocity_filter_time_).count();
    }
    if (dt_sec <= 0.0f || dt_sec > 0.1f)
    {
        dt_sec = 0.01f;
    }
    last_velocity_filter_time_ = current_time;
    velocity_filter_time_initialized_ = true;

    for (size_t i = 0; i < all_motors_.size(); i++)
    {
        msg.states[i].motor_id = all_motors_[i]->getMotorId();

        if (current_cycle_updated[i])
        {
            ++success_count;
            ++cycle_success_count;

            const float raw_pos = static_cast<float>(all_motors_[i]->getPosition());
            const float wrapped_pos = wrapToPi(raw_pos);
            const float raw_velocity = static_cast<float>(all_motors_[i]->getVelocity());
            const float current  = static_cast<float>(all_motors_[i]->getCurrent());

            // ===== Init last valid q update =====
            if (std::isfinite(raw_pos) && std::fabs(raw_pos) <= INIT_Q_ABS_LIMIT)
            {
                if (i < motor_feedback_seen_.size())
                {
                    motor_feedback_seen_[i] = true;
                }

                if (i < last_valid_motor_pos_.size())
                {
                    last_valid_motor_pos_[i] = raw_pos;
                }
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[InitValidation] Invalid q feedback ignored. motor_index=%zu raw_pos=%.6f",
                    i,
                    raw_pos);
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
            msg.states[i].position = wrapToPi(static_cast<float>(all_motors_[i]->getPosition()));
            {
                const float raw_velocity = static_cast<float>(all_motors_[i]->getVelocity());
                float velocity = raw_velocity;
                if (i < velocity_filters_.size())
                {
                    velocity = velocity_filters_[i].filter(raw_velocity, dt_sec);
                }
                msg.states[i].velocity = velocity;
            }
            msg.states[i].current  = static_cast<float>(all_motors_[i]->getCurrent());

            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //     "[Read] bus=%s packet_index=%zu motor_id=%u update failed (success=%d fail=%d)",
            //     packet_index_to_bus_[i].c_str(),
            //     i,
            //     msg.states[i].motor_id,
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

    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "stabilzation_ : %s, cycle_fali_count : %d", last_read_cycle_all_updated_ ? "true" : "false", cycle_fail_count);
    state_pub->publish(msg);
    transition_to(ControlState::WRITE_PACKET);
}

void MainControlNode::logWriteSummaryThrottle()
{
    for (const auto& group : can_groups_)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[WriteSummary] "
            "bus=%s "
            "ok=%u fail=%u "
            "eagain_total=%u "
            "eagain_cycles=%u "
            "enobufs_total=%u "
            "enobufs_cycles=%u",
            group.interface_name.c_str(),
            group.write_stats.ok_writes,
            group.write_stats.fail_writes,
            group.write_stats.eagain_count,
            group.write_stats
                .consecutive_eagain_cycles,
            group.write_stats.enobufs_count,
            group.write_stats
                .consecutive_enobufs_cycles);
    }
}

void MainControlNode::handle_write_packet()
{
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "[Write] ENTER packet_initialized=%s walk_initialized=%s",
    //     packet_initialized_ ? "true" : "false",
    //     walk_initialized_ ? "true" : "false");
    constexpr auto EAGAIN_FATAL_DURATION =
        std::chrono::milliseconds(200);
    constexpr uint32_t ENOBUFS_FATAL_CYCLES = 5;

    float alpha = 0.0f;

    roa_interfaces::msg::MotorCommandArray command_snapshot;
    bool packet_initialized_snapshot = false;

    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        packet_initialized_snapshot = packet_initialized_;

        if (packet_initialized_snapshot)
        {
            command_snapshot = packet_commands_;
        }
    }

    if (!packet_initialized_snapshot)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Write] No command received yet, skipping write. HINT: check if the command publisher is active and publishing to /hardware_interface/command");

        transition_to(ControlState::READ_PACKET);
        return;
    }

    if (!walk_initialized_ && !start_positions_captured_)
    {
        std::string reason;
        if (!isStartPositionReady(&reason))
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Init] Aborting initialization-write due to invalid start position reason=%s",
                reason.c_str());
            transition_to(ControlState::READ_PACKET);
            return;
        }
        // printInitialRawPositionsOnce("before_start_capture");
        start_positions_.assign(all_motors_.size(), 0.0f);

        // // todo 추후 삭제 
        // return;

        for (size_t i = 0; i < all_motors_.size(); ++i)
        {
            const float pos = last_valid_motor_pos_[i];

            if (!std::isfinite(pos) || std::fabs(pos) > INIT_Q_ABS_LIMIT)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "[Init] Final capture rejected. motor=%zu pos=%.6f",
                    i,
                    pos);

                start_positions_.clear();
                start_positions_captured_ = false;

                transition_to(ControlState::READ_PACKET);
                return;
            }

            start_positions_[i] = pos;

            RCLCPP_INFO(this->get_logger(),
                "[Init] Captured Motor %zu at position %.6f",
                i,
                start_positions_[i]);
        }

        start_positions_captured_ = true;
        init_tick_count_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "[Init] Captured validated start positions for %zu motors",
            all_motors_.size());

        transition_to(ControlState::READ_PACKET);
        return;
    }
    else if (!walk_initialized_)
    {
        ++init_tick_count_;

        alpha = static_cast<float>(init_tick_count_) /
                static_cast<float>(INIT_TOTAL_TICKS);

        if (alpha > 1.0f)
        {
            alpha = 1.0f;
        }
    }

    for (auto& group : can_groups_)
    {
        bool eagain_this_cycle = false;
        bool enobufs_this_cycle = false;
        bool bus_down_this_cycle = false;

        for (size_t local_idx = 0; local_idx < group.motors.size(); ++local_idx)
        {
            const size_t packet_index = group.global_packet_indices[local_idx];

            if (packet_index >= command_snapshot.commands.size())
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[Write] packet_index=%zu out of command_snapshot size=%zu",
                    packet_index,
                    command_snapshot.commands.size());
                continue;
            }

            auto& motor = group.motors[local_idx];
            const auto& cmd = command_snapshot.commands[packet_index];

            float command_pos = wrapToPi(cmd.position);

            if (!walk_initialized_)
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

                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                //     "[Init] bus=%s packet_index=%zu motor_id=%u start=%.3f target=%.3f cmd=%.3f alpha=%.3f",
                //     group.interface_name.c_str(),
                //     packet_index,
                //     cmd.motor_id,
                //     start_positions_[packet_index],
                //     wrapToPi(cmd.position),
                //     command_pos,
                //     alpha);
            }
            else
            {
                const float raw_pos =
                    static_cast<float>(all_motors_[packet_index]->getPosition());

                command_pos = computeWrappedCommand(raw_pos, wrapToPi(cmd.position));
            }
         
            const WriteResult result = safeSendCommand(
                *motor,
                cmd.torque,
                command_pos,
                cmd.velocity,
                cmd.kp,
                cmd.kd);

            motor_write_stats_[packet_index].last_result = result;

            if (result == WriteResult::Ok)
            {
                motor_write_stats_[packet_index].consecutive_failures = 0;
                ++group.write_stats.ok_writes;
                // group.write_stats.consecutive_enobufs = 0;
                continue;
            }

            ++motor_write_stats_[packet_index].consecutive_failures;
            ++group.write_stats.fail_writes;

            const char* phase = walk_initialized_ ? "track" : "init";

            if (result == WriteResult::TryAgain)
            {
                ++group.write_stats.eagain_count;

                eagain_this_cycle = true;

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "[Write] EAGAIN: "
                    "phase=%s bus=%s motor_id=%u "
                    "packet_index=%zu total=%u",
                    phase,
                    group.interface_name.c_str(),
                    cmd.motor_id,
                    packet_index,
                    group.write_stats.eagain_count);

                // TX 큐가 현재 가득 차 있으므로
                // 나머지 모터도 실패할 가능성이 높음
                break;
            }

            if (result == WriteResult::NoBuffer)
            {
                ++group.write_stats.enobufs_count;

                enobufs_this_cycle = true;

                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "[Write] ENOBUFS: "
                    "phase=%s bus=%s motor_id=%u "
                    "packet_index=%zu total=%u",
                    phase,
                    group.interface_name.c_str(),
                    cmd.motor_id,
                    packet_index,
                    group.write_stats.enobufs_count);

                break;
            }

            if (result == WriteResult::BusDown)
            {
                bus_down_this_cycle = true;

                RCLCPP_ERROR(
                    this->get_logger(),
                    "[Write] CAN bus unavailable: "
                    "phase=%s bus=%s motor_id=%u "
                    "packet_index=%zu",
                    phase,
                    group.interface_name.c_str(),
                    cmd.motor_id,
                    packet_index);

                break;
            }
            // if (result == WriteResult::WouldBlock)
            // {
            //     ++group.write_stats.enobufs_count;
            //     ++group.write_stats.consecutive_enobufs;

            //     // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //     //     "[Write] phase=%s bus=%s motor_id=%u packet_index=%zu result=%s motor_fail_streak=%u bus_enobufs_streak=%u",
            //     //     phase,
            //     //     group.interface_name.c_str(),
            //     //     cmd.motor_id,
            //     //     packet_index,
            //     //     toString(result),
            //     //     motor_write_stats_[packet_index].consecutive_failures,
            //     //     group.write_stats.consecutive_enobufs);

            //     bus_blocked_this_cycle = true;
            //     break;
            // }

            // if (result == WriteResult::BusDown)
            // {
            //     RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //         "[Write] phase=%s bus=%s motor_id=%u packet_index=%zu result=%s",
            //         phase,
            //         group.interface_name.c_str(),
            //         cmd.motor_id,
            //         packet_index,
            //         toString(result));

            //     bus_blocked_this_cycle = true;
            //     break;
            // }

            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[Write] phase=%s bus=%s "
                "motor_id=%u packet_index=%zu "
                "result=%s fail_streak=%u",
                phase,
                group.interface_name.c_str(),
                cmd.motor_id,
                packet_index,
                toString(result),
                motor_write_stats_[packet_index]
                    .consecutive_failures);
        } // 모터 반복문 end

        if (bus_down_this_cycle)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "[Safety] CAN bus unavailable: bus=%s",
                group.interface_name.c_str());

            timer_.reset();
            // disableAllMotors();

            return;
        }

        if (eagain_this_cycle)
        {
            ++group.write_stats.consecutive_eagain_cycles;

            if (!group.write_stats.eagain_active)
            {
                group.write_stats.eagain_active = true;
                group.write_stats.first_eagain_time =
                    std::chrono::steady_clock::now();
            }
        }
        else
        {
            group.write_stats.consecutive_eagain_cycles = 0;
            group.write_stats.eagain_active = false;
        }

        if (enobufs_this_cycle)
        {
            ++group.write_stats.consecutive_enobufs_cycles;
        }
        else
        {
            group.write_stats.consecutive_enobufs_cycles = 0;
        }

        if (group.write_stats.eagain_active)
        {
            const auto now_steady =
                std::chrono::steady_clock::now();

            const auto elapsed =
                now_steady -
                group.write_stats.first_eagain_time;

            if (elapsed >= EAGAIN_FATAL_DURATION)
            {
                const auto elapsed_ms =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                        elapsed)
                        .count();

                RCLCPP_FATAL(
                    this->get_logger(),
                    "[Safety] Persistent CAN EAGAIN: "
                    "bus=%s duration=%ld ms "
                    "failed_cycles=%u",
                    group.interface_name.c_str(),
                    static_cast<long>(elapsed_ms),
                    group.write_stats
                        .consecutive_eagain_cycles);

                timer_.reset();
                // disableAllMotors();

                return;
            }
        }

        if (group.write_stats.consecutive_enobufs_cycles >=
            ENOBUFS_FATAL_CYCLES)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "[Safety] Persistent ENOBUFS: "
                "bus=%s cycles=%u",
                group.interface_name.c_str(),
                group.write_stats
                    .consecutive_enobufs_cycles);

            timer_.reset();
            // disableAllMotors();
            return;
        }
    }

    if (!walk_initialized_ && alpha >= 1.0f)
    {
        walk_initialized_ = true;

        RCLCPP_INFO(this->get_logger(),
            "[Init] initialization interpolation completed after %d ticks",
            init_tick_count_);
    }

    logWriteSummaryThrottle();
    transition_to(ControlState::READ_PACKET);
}

void MainControlNode::walkCallback(const roa_interfaces::msg::MotorCommandArray::SharedPtr msg)
{
    std_msgs::msg::Bool init_msg;
    init_msg.data = walk_initialized_;
    initial_pub->publish(init_msg);

    std::lock_guard<std::mutex> lock(command_mutex_);

    packet_commands_.header = msg->header;

    for (const auto& cmd : msg->commands)
    {
        const auto it = motor_id_to_index_.find(cmd.motor_id);
        if (it == motor_id_to_index_.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                "[ros_sub] Unknown motor_id: %u", cmd.motor_id);
            continue;
        }

        const size_t packet_index = it->second;
        packet_commands_.commands[packet_index] = cmd;

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "[Walk Callback] bus=%s motor_id=%u -> packet_index=%zu | pos=%.3f vel=%.3f kp=%.3f kd=%.3f tq=%.3f",
        //     packet_index_to_bus_[packet_index].c_str(),
        //     cmd.motor_id,
        //     packet_index,
        //     cmd.position,
        //     cmd.velocity,
        //     cmd.kp,
        //     cmd.kd,
        //     cmd.torque);
    }

    if (!packet_initialized_)
    {
        RCLCPP_INFO(this->get_logger(), "[ros_sub] First valid command received");
    }

    packet_initialized_ = true;
}

void MainControlNode::torqueCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    const bool torque_enable = msg->data;

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        bool ok = false;

        if (torque_enable)
        {
            ok = all_motors_[i]->enable();
            if (ok)
            {
                RCLCPP_INFO(this->get_logger(),
                    "[Torque Callback] Motor[%zu] enable command sent", i);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),
                    "[Torque Callback] Failed to enable motor[%zu] (bus=%s, motor_id=%u)",
                    i,
                    packet_index_to_bus_[i].c_str(),
                    all_motors_[i]->getMotorId());
            }
        }
        else
        {
            ok = all_motors_[i]->disable();
            if (ok)
            {
                RCLCPP_INFO(this->get_logger(),
                    "[Torque Callback] Motor[%zu] disabled successfully", i);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),
                    "[Torque Callback] Failed to disable motor[%zu] (bus=%s, motor_id=%u)",
                    i,
                    packet_index_to_bus_[i].c_str(),
                    all_motors_[i]->getMotorId());
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
        int flushed = 0;

        while (group.transport && group.transport->receive(rx_id, rx_data, 0))
        {
            flushed++;
        }

        RCLCPP_WARN(this->get_logger(),
            "[CAN Flush:%s] bus=%s flushed %d pending RX frames",
            tag,
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
