#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP
#include <stdexcept>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "RobStrideMotor.hpp"
#include "CanTransport.hpp"
#include "CanBusWorker.hpp"
#include "HardwareData.hpp"
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
#include <deque>
#include <algorithm>

enum class InitPhase    // WJ: 초기화시 모터 상태 확인
{
    COLLECT_FEEDBACK,   // Zero Command를 보내며 피드백 수집
    WAIT_COMMAND,       // 초기 위치 확정 후 상위 명령 대기
    INTERPOLATING,      // 초기 위치에서 목표 위치까지 이동
    RUNNING,            // 정상 제어
    FAILED
};
enum class ControlState
{
    WRITE_PACKET,
    READ_PACKET
};

enum class InitSampleCheckResult
{
    Collecting,  // 아직 샘플이 부족함
    Ready,       // 샘플이 충분하고 안정적임
    Fatal        // 샘플은 충분하지만 비정상 상태
};

struct CanBusGroup
{
    std::string interface_name;
    std::shared_ptr<CanTransport> transport;
    std::vector<std::shared_ptr<RobStrideMotor>> motors;
    std::vector<size_t> global_packet_indices;
    std::shared_ptr<CanBusWorker> worker;
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

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);

private:
    void publishWalkInitialized(bool initialized);
    void flushCanRxQueues(const char* tag);
    void control_loop();
    void walkCallback(const roa_interfaces::msg::MotorCommandArray::SharedPtr msg);
    void torqueCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void handle_read_packet();
    void handle_write_packet();
    void transition_to(ControlState new_state);
    void initParameters();
    bool updateControlLoopPeriod();
    void startControlLoopTimer();

    // NOTE: use can-setup.sh to set up CAN interfaces before running. This function is a placeholder if we want to do dynamic setup in the future.
    // bool canSetup(); 
    // void toCSV(float pos, float vel);

    std::string execute_command(const std::string& cmd);

    float computeWrappedCommand(float current_raw_pos, float target_wrapped_pos) const;
    void resetRuntimeStates();
    void logWriteSummaryThrottle();
    bool startWorkers();
    void stopWorkers();
    bool waitForWorkerOperations(
        const std::vector<uint64_t>& generations,
        std::chrono::microseconds timeout,
        const char* operation);
    // bool isStartPositionReady(std::string* reason);

    std::vector<CanBusGroup> can_groups_;
    std::vector<std::shared_ptr<RobStrideMotor>> all_motors_;
    std::vector<uint16_t> motor_ids_;
    std::vector<MotorStateData> latest_motor_states_;

    rclcpp::TimerBase::SharedPtr timer_;
    double control_frequency_hz_{150.0};
    std::chrono::nanoseconds control_loop_period_{std::chrono::nanoseconds(3333333)};
    rclcpp::Subscription<roa_interfaces::msg::MotorCommandArray>::SharedPtr walk_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_sub;
    rclcpp::Publisher<roa_interfaces::msg::MotorStateArray>::SharedPtr state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initial_pub;

    ControlState current_state{ControlState::READ_PACKET};

    std::vector<std::string> packet_index_to_bus_;

    static constexpr auto WORKER_COMPLETION_TIMEOUT =
        std::chrono::microseconds(2500);

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
    bool start_position_init_attempted_ = false;
    bool last_read_cycle_all_updated_ = false;

    std::unordered_map<uint16_t, size_t> motor_id_to_index_;

    roa_interfaces::msg::MotorCommandArray packet_commands_;
    bool packet_initialized_{false};
    bool torque_enabled_{false};

    // ===== Init start position validation =====
    static constexpr float INIT_Q_ABS_LIMIT =
        4.0f * static_cast<float>(M_PI);

    std::vector<bool> motor_feedback_seen_;
    std::vector<float> last_valid_motor_pos_;

    // For debugging
    // void printInitialRawPositionsOnce(const char* tag);
    bool initial_raw_position_printed_ = false;

    void disableAllMotors();
    void requestFatalShutdown(const std::string& reason);


    // ----------------------------- WJ 초기 피드백 수집 및 상태 확인 -----------------------------
    bool sendZeroCommands();
    InitSampleCheckResult checkInitialSamples(
        std::string* reason) const;
    float computeMedian(const std::deque<float>& samples) const;

    static constexpr std::size_t INIT_REQUIRED_SAMPLES = 20;

    // 초기 피드백의 위치 변화 허용 범위
    static constexpr float INIT_POSITION_SPREAD_LIMIT = 0.03f;

    // 초기 상태에서 허용할 최대 속도
    static constexpr float INIT_VELOCITY_LIMIT = 0.5f;

    // 피드백이 이 시간보다 오래되면 stale로 판단
    static constexpr int INIT_FEEDBACK_STALE_MS = 100;

    // 초기 피드백 수집 전체 제한 시간
    static constexpr int INIT_COLLECTION_TIMEOUT_MS = 2000;

    InitPhase init_phase_ = InitPhase::COLLECT_FEEDBACK;

    std::vector<std::deque<float>> init_position_samples_;
    std::vector<std::deque<float>> init_velocity_samples_;

    std::vector<std::chrono::steady_clock::time_point>
        last_feedback_time_;

    std::chrono::steady_clock::time_point
        init_phase_start_time_;
};

#endif // MAIN_CONTROL_HPP
