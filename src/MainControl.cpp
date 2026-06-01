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

bool MainControlNode::canSetup()
{
    const auto can_interfaces = get_parameter("can_interfaces").as_string_array();
    const auto baud_rate = get_parameter("baud_rate").as_int();

    if (can_interfaces.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "[CAN Setup] 'can_interfaces' is empty");
        return false;
    }

    bool all_ok = true;

    for (const auto& can_name : can_interfaces)
    {
        RCLCPP_INFO(this->get_logger(),
            "[CAN Setup] Setting up interface '%s' with bitrate %ld",
            can_name.c_str(), baud_rate);

        std::string result = execute_command("ip link show " + can_name + " 2>&1");
        if (result.find("does not exist") != std::string::npos ||
            result.find("Cannot find device") != std::string::npos)
        {
            RCLCPP_ERROR(this->get_logger(),
                "[CAN Setup] Interface '%s' does not exist",
                can_name.c_str());
            all_ok = false;
            continue;
        }

        execute_command("sudo ip link set " + can_name + " down 2>&1");
        execute_command("sudo ip link set " + can_name +" type can bitrate " + std::to_string(baud_rate) + " echo off 2>&1");
        execute_command("sudo ip link set " + can_name + " up 2>&1");
        execute_command("sleep 0.2");
        execute_command("sudo ip link set " + can_name + " txqueuelen 4096 2>&1");
        result = execute_command("ip -details link show " + can_name + " 2>&1");

        const bool is_up = (result.find("state UP") != std::string::npos);
        const bool bitrate_ok =
            (result.find("bitrate " + std::to_string(baud_rate)) != std::string::npos);

        if (is_up && bitrate_ok)
        {
            RCLCPP_INFO(this->get_logger(),
                "[CAN Setup] '%s' activated successfully (bitrate=%ld)",
                can_name.c_str(), baud_rate);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                "[CAN Setup] '%s' setup failed. Expected UP and bitrate=%ld",
                can_name.c_str(), baud_rate);
            RCLCPP_ERROR(this->get_logger(),
                "[CAN Setup] Details:\n%s", result.c_str());
            all_ok = false;
        }
    }

    return all_ok;
}

void MainControlNode::resetRuntimeStates()
{
    walk_initialized_ = false;
    start_positions_captured_ = false;
    init_tick_count_ = 0;
    start_positions_.clear();
    stabilization_ = false;

    packet_initialized_ = false;

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
            RCLCPP_ERROR(this->get_logger(),
                "[Configure] Failed to open CAN interface '%s'",
                can_name.c_str());
            return CallbackReturn::FAILURE;
        }

        const auto motor_ids = get_parameter(can_name + ".motor_ids").as_integer_array();
        const auto motor_types = get_parameter(can_name + ".motor_type").as_integer_array();

        if (motor_ids.size() != motor_types.size())
        {
            RCLCPP_ERROR(this->get_logger(),
                "[Configure] %s: motor_ids and motor_type size mismatch",
                can_name.c_str());
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
            RCLCPP_ERROR(this->get_logger(),
                "[Configure] Duplicate motor_id detected: %u", id);
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

    for (size_t i = 0; i < all_motors_.size(); ++i)
    {
        if (all_motors_[i]->enable())
        {
            RCLCPP_INFO(this->get_logger(),
                "[Activate] Motor[%zu] enabled successfully", i);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                "[Activate] Failed to enable motor[%zu] (bus=%s, motor_id=%u)",
                i,
                (i < packet_index_to_bus_.size() ? packet_index_to_bus_[i].c_str() : "unknown"),
                all_motors_[i]->getMotorId());
            return CallbackReturn::FAILURE;
        }
    }
    walk_initialized_ = false;
    start_positions_captured_ = false;
    init_tick_count_ = 0;

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
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[Loop] current_state=%s",
    (current_state == ControlState::READ_PACKET ? "READ" : "WRITE"));

    static auto last_print_time = std::chrono::steady_clock::now();
    static int write_count = 0;

    if (current_state == ControlState::WRITE_PACKET)
    {
        ++write_count;

        const auto current_time = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = current_time - last_print_time;

        if (elapsed.count() >= 1.0)
        {
            const double hz = write_count / elapsed.count();
            RCLCPP_INFO(this->get_logger(), "Motor Control Rate: %.2f Hz", hz);
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

void MainControlNode::transition_to(ControlState new_state)
{
    current_state = new_state;
}

const char* MainControlNode::toString(WriteResult result) const
{
    switch (result)
    {
        case WriteResult::Ok:         return "Ok";
        case WriteResult::WouldBlock: return "WouldBlock";
        case WriteResult::BusDown:    return "BusDown";
        case WriteResult::IoError:    return "IoError";
        case WriteResult::InvalidArg: return "InvalidArg";
        default:                      return "Unknown";
    }
}

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

    const bool ok = motor.sendMotionCommand(torque, position, velocity, kp, kd);
    if (ok)
    {
        return WriteResult::Ok;
    }

    switch (errno)
    {
        case ENOBUFS:
        case EAGAIN:
            return WriteResult::WouldBlock;
        case ENETDOWN:
        case ENODEV:
            return WriteResult::BusDown;
        case EINVAL:
            return WriteResult::InvalidArg;
        default:
            return WriteResult::IoError;
    }
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

                if (motor->processPacket(rx_id, rx_data))
                {
                    if (local_idx < group.global_packet_indices.size())
                    {
                        const size_t packet_index = group.global_packet_indices[local_idx];

                        if (packet_index < current_cycle_updated.size())
                        {
                            current_cycle_updated[packet_index] = true;
                        }
                    }
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

            const float wrapped_pos = wrapToPi(static_cast<float>(all_motors_[i]->getPosition()));
            const float raw_velocity = static_cast<float>(all_motors_[i]->getVelocity());
            const float current  = static_cast<float>(all_motors_[i]->getCurrent());

            float velocity = raw_velocity;
            if (i < velocity_filters_.size())
            {
                velocity = velocity_filters_[i].filter(raw_velocity, dt_sec);
            }

            msg.states[i].position = wrapped_pos;
            msg.states[i].velocity = velocity;
            msg.states[i].current  = current;

            RCLCPP_DEBUG(this->get_logger(),
                "[Read] bus=%s packet_index=%zu motor_id=%u pos=%.3f vel=%.3f cur=%.3f",
                packet_index_to_bus_[i].c_str(),
                i,
                msg.states[i].motor_id,
                wrapped_pos,
                velocity,
                current);
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

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Read] bus=%s packet_index=%zu motor_id=%u update failed (success=%d fail=%d)",
                packet_index_to_bus_[i].c_str(),
                i,
                msg.states[i].motor_id,
                success_count,
                fail_count);
        }
    }

    if(cycle_fail_count != 0)
    {
        stabilization_ = false;
    }
    else
    {
        stabilization_ = true;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "stabilzation_ : %s, cycle_fali_count : %d", stabilization_ ? "true" : "false", cycle_fail_count);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[Read] BEFORE_PUBLISH");
    state_pub->publish(msg);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[Read] AFTER_PUBLISH");
    transition_to(ControlState::WRITE_PACKET);
}

void MainControlNode::logWriteSummaryThrottle()
{
    for (const auto& group : can_groups_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[WriteSummary] bus=%s ok=%u fail=%u enobufs_total=%u enobufs_streak=%u cooldown=%s",
            group.interface_name.c_str(),
            group.write_stats.ok_writes,
            group.write_stats.fail_writes,
            group.write_stats.enobufs_count,
            group.write_stats.consecutive_enobufs,
            group.write_stats.cooldown ? "true" : "false");
    }
}

void MainControlNode::handle_write_packet()
{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[Write] ENTER packet_initialized=%s walk_initialized=%s",
    packet_initialized_ ? "true" : "false",
    walk_initialized_ ? "true" : "false");
    float alpha = 0.0f;

    std::lock_guard<std::mutex> lock(command_mutex_);


    if (!packet_initialized_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Write] packet_commands_ not initialized yet");
        transition_to(ControlState::READ_PACKET);
        return;
    }

    if (!walk_initialized_ && !start_positions_captured_)
    {
        if (!stabilization_)
        {
            start_positions_.resize(all_motors_.size(), 0.0f);

            for (size_t i = 0; i < all_motors_.size(); ++i)
            {
                start_positions_[i] = static_cast<float>(all_motors_[i]->getPosition());
                RCLCPP_INFO(this->get_logger(), "[Init] Captured Motor %zu at position %.3f", i, start_positions_[i]);
            }

            start_positions_captured_ = true;
            init_tick_count_ = 0;

            RCLCPP_INFO(this->get_logger(),
                "[Init] Captured start positions for %zu motors",
                all_motors_.size());

            // transition_to(ControlState::READ_PACKET);
            // return;

        }

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
        const auto now = this->get_clock()->now();

        if (group.write_stats.cooldown && now < group.write_stats.cooldown_until)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Write] bus=%s in cooldown",
                group.interface_name.c_str());
            continue;
        }

        group.write_stats.cooldown = false;
        bool bus_blocked_this_cycle = false;

        for (size_t local_idx = 0; local_idx < group.motors.size(); ++local_idx)
        {
            const size_t packet_index = group.global_packet_indices[local_idx];
            auto& motor = group.motors[local_idx];
            auto& cmd = packet_commands_.commands[packet_index];

            float command_pos = wrapToPi(cmd.position);

            if (!walk_initialized_)
            {
                const float start_raw = start_positions_[packet_index];
                float diff = command_pos - wrapToPi(start_raw);

                if (diff > static_cast<float>(M_PI))
                    diff -= 2.0f * static_cast<float>(M_PI);
                if (diff < -static_cast<float>(M_PI))
                    diff += 2.0f * static_cast<float>(M_PI);

                command_pos = start_raw + alpha * diff;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                    "[Init] bus=%s packet_index=%zu motor_id=%u start=%.3f target=%.3f cmd=%.3f alpha=%.3f",
                    group.interface_name.c_str(),
                    packet_index,
                    cmd.motor_id,
                    start_positions_[packet_index],
                    wrapToPi(cmd.position),
                    command_pos,
                    alpha);
            }
            else
            {
                const float raw_pos = static_cast<float>(all_motors_[packet_index]->getPosition());
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
                group.write_stats.consecutive_enobufs = 0;
                continue;
            }

            ++motor_write_stats_[packet_index].consecutive_failures;
            ++group.write_stats.fail_writes;

            const char* phase = walk_initialized_ ? "track" : "init";

            if (result == WriteResult::WouldBlock)
            {
                ++group.write_stats.enobufs_count;
                ++group.write_stats.consecutive_enobufs;

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[Write] phase=%s bus=%s motor_id=%u packet_index=%zu result=%s motor_fail_streak=%u bus_enobufs_streak=%u",
                    phase,
                    group.interface_name.c_str(),
                    cmd.motor_id,
                    packet_index,
                    toString(result),
                    motor_write_stats_[packet_index].consecutive_failures,
                    group.write_stats.consecutive_enobufs);

                bus_blocked_this_cycle = true;
                break;
            }

            if (result == WriteResult::BusDown)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[Write] phase=%s bus=%s motor_id=%u packet_index=%zu result=%s",
                    phase,
                    group.interface_name.c_str(),
                    cmd.motor_id,
                    packet_index,
                    toString(result));

                bus_blocked_this_cycle = true;
                break;
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Write] phase=%s bus=%s motor_id=%u packet_index=%zu result=%s fail_streak=%u",
                phase,
                group.interface_name.c_str(),
                cmd.motor_id,
                packet_index,
                toString(result),
                motor_write_stats_[packet_index].consecutive_failures);
        }

        if (bus_blocked_this_cycle &&
            group.write_stats.consecutive_enobufs >= enobufs_cooldown_threshold_)
        {
            group.write_stats.cooldown = true;
            group.write_stats.cooldown_until = now + bus_write_cooldown_;

            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Write] bus=%s entered cooldown: enobufs_streak=%u cooldown_ms=%.1f",
                group.interface_name.c_str(),
                group.write_stats.consecutive_enobufs,
                bus_write_cooldown_.seconds() * 1000.0);
        }
    }

    if (!walk_initialized_ && alpha >= 1.0f)
    {
        walk_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[Init] Walk initialized after %d ticks", init_tick_count_);
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
            RCLCPP_WARN(this->get_logger(),
                "[Walk Callback] Unknown motor_id: %u", cmd.motor_id);
            continue;
        }

        const size_t packet_index = it->second;
        packet_commands_.commands[packet_index] = cmd;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[Walk Callback] bus=%s motor_id=%u -> packet_index=%zu | pos=%.3f vel=%.3f kp=%.3f kd=%.3f tq=%.3f",
            packet_index_to_bus_[packet_index].c_str(),
            cmd.motor_id,
            packet_index,
            cmd.position,
            cmd.velocity,
            cmd.kp,
            cmd.kd,
            cmd.torque);
    }

    if (!packet_initialized_)
    {
        RCLCPP_INFO(this->get_logger(), "[Walk Callback] First valid command received");
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
                    "[Torque Callback] Motor[%zu] enabled successfully", i);
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

void MainControlNode::toCSV(float pos, float vel)
{
  static std::ofstream csv_file("motor_data.csv");

  if(!csv_file.is_open())
  {
    std::cerr << "Failed to open CSV file for writing!" << std::endl;
    return;
  }

  csv_file << "Timestamp(ms),Position(rad),Velocity(rad/s)" << std::endl;

  static bool initialized = false;

  if (!initialized)
  {
    pos = 0.0f;
    vel = 0.0f;

    initialized = true;
  }

  static int timestamp = 0;

  csv_file << std::fixed << std::setprecision(5);
  csv_file << timestamp << "," << pos << "," << vel << "\n";

  std::cout << "Converting data to CSV..." << std::endl;

  if(timestamp >= 10000) // 10초 동안 데이터 기록 후 종료
  {
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;
    std::cout << "Finished writing to CSV. Closing file." << std::endl;

    csv_file.close();
  }
  else
  {
    timestamp += 2;
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
