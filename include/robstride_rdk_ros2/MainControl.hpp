#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "RobStrideMotor.hpp"
#include "CanTransport.hpp"
#include "filter.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "roa_interfaces/msg/motor_state.hpp"
#include "roa_interfaces/msg/motor_state_array.hpp"
#include "roa_interfaces/msg/motor_command.hpp"
#include "roa_interfaces/msg/motor_command_array.hpp"

#include <unordered_map>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <chrono>

enum class ControlState
{
    WRITE_PACKET,
    READ_PACKET
};

enum class WriteResult
{
    Ok = 0,
    WouldBlock,   // ENOBUFS, EAGAIN
    BusDown,      // ENETDOWN, ENODEV
    IoError,      // generic write failure
    InvalidArg,
};

struct BusWriteStats
{
    uint32_t ok_writes = 0;
    uint32_t fail_writes = 0;
    uint32_t enobufs_count = 0;
    uint32_t consecutive_enobufs = 0;

    bool cooldown = false;
    rclcpp::Time cooldown_until{0, 0, RCL_ROS_TIME};
};

struct MotorWriteStats
{
    uint32_t consecutive_failures = 0;
    WriteResult last_result = WriteResult::Ok;
};

struct CanBusGroup
{
    std::string interface_name;
    std::shared_ptr<CanTransport> transport;
    std::vector<std::shared_ptr<RobStrideMotor>> motors;

    BusWriteStats write_stats;
    std::vector<size_t> global_packet_indices;
};

class MainControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit MainControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~MainControlNode();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);

private:
    void control_loop();
    void walkCallback(const roa_interfaces::msg::MotorCommandArray::SharedPtr msg);
    void torqueCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void handle_read_packet();
    void handle_write_packet();
    void transition_to(ControlState new_state);
    void initParameters();

    bool canSetup();
    void toCSV(float pos, float vel);

    std::string execute_command(const std::string& cmd);

    WriteResult safeSendCommand(
        RobStrideMotor& motor,
        float torque,
        float position,
        float velocity,
        float kp,
        float kd);

    const char* toString(WriteResult result) const;
    float computeWrappedCommand(float current_raw_pos, float target_wrapped_pos) const;
    void resetRuntimeStates();
    void logWriteSummaryThrottle();

    std::vector<CanBusGroup> can_groups_;
    std::vector<std::shared_ptr<RobStrideMotor>> all_motors_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<roa_interfaces::msg::MotorCommandArray>::SharedPtr walk_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_sub;
    rclcpp::Publisher<roa_interfaces::msg::MotorStateArray>::SharedPtr state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initial_pub;

    ControlState current_state{ControlState::READ_PACKET};

    std::vector<std::string> packet_index_to_bus_;
    std::vector<MotorWriteStats> motor_write_stats_;

    rclcpp::Duration bus_write_cooldown_{0, 50 * 1000 * 1000}; // 50ms
    uint32_t enobufs_cooldown_threshold_ = 5;

    std::mutex command_mutex_;

    std::vector<Butterworth2ndOrderLPF> velocity_filters_;
    std::chrono::steady_clock::time_point last_velocity_filter_time_;
    bool velocity_filter_time_initialized_ = false;

    // 초기 set 자세
    bool walk_initialized_ = false;
    bool start_positions_captured_ = false;
    std::vector<float> start_positions_;
    int init_tick_count_ = 0;
    static constexpr int INIT_TOTAL_TICKS = 100;

    bool stabilization_ = false;

    std::unordered_map<uint16_t, size_t> motor_id_to_index_;

    roa_interfaces::msg::MotorCommandArray packet_commands_;
    bool packet_initialized_{false};
};

#endif // MAIN_CONTROL_HPP
