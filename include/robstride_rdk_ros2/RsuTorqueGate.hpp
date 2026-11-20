#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <roa_interfaces/msg/rsu_torque_command.hpp>
#include <roa_interfaces/msg/motor_command_array.hpp>

namespace rsu_torque {
inline int64_t stamp_ns(const builtin_interfaces::msg::Time& s) {
  return int64_t(s.sec)*1000000000LL+s.nanosec;
}
inline bool fresh(int64_t now, int64_t stamp, double timeout_ms) {
  return stamp>0 && now>=stamp && double(now-stamp)<=timeout_ms*1e6;
}
constexpr std::array<uint16_t,4> ids{18,20,19,21};
inline int index(uint16_t id) {
  for (int i=0;i<4;++i) if (ids[i]==id) return i;
  return -1;
}
// Legacy MotorCommand torque is in protocol axes. Direct RSU torque is in
// the model axes used by position/velocity: all four ankle directions are -1.
inline double protocol_torque(double model_torque) { return -model_torque; }

class Gate {
public:
  using Clock=std::chrono::steady_clock;
  explicit Gate(double limit=11.9, double timeout_ms=20.0,
      double feedback_timeout_ms=20.0, double target_timeout_ms=100.0)
  : limit_(limit), timeout_(timeout_ms), feedback_timeout_(feedback_timeout_ms),
    target_timeout_(target_timeout_ms) {
    for (auto x : {limit_,timeout_,feedback_timeout_,target_timeout_})
      if (!std::isfinite(x) || x<=0) throw std::invalid_argument("invalid RSU gate limits");
    if (limit_>36.0) throw std::invalid_argument("RS06 torque limit exceeds protocol range");
  }
  void reset() { received_=false; fault_=false; last_stamp_=0; command_=roa_interfaces::msg::RsuTorqueCommand(); }
  bool accept(const roa_interfaces::msg::RsuTorqueCommand& c,
      int64_t ros_now, Clock::time_point steady_now) {
    const auto stamp=stamp_ns(c.header.stamp);
    if (fault_ || !c.valid || c.fault_code!=0 || stamp<=last_stamp_ ||
        !fresh(ros_now,stamp,timeout_) ||
        !fresh(ros_now,stamp_ns(c.feedback_header.stamp),feedback_timeout_) ||
        !fresh(ros_now,stamp_ns(c.target_header.stamp),target_timeout_)) {
      fault_=true; return false;
    }
    for (auto x : c.torque) if (!std::isfinite(x) || std::abs(x)>limit_+1e-9) {
      fault_=true; return false;
    }
    command_=c; last_stamp_=stamp; received_at_=steady_now; received_=true;
    return true;
  }
  bool apply(roa_interfaces::msg::MotorCommandArray& packet,
      int64_t ros_now, Clock::time_point steady_now) {
    if (!ready(ros_now,steady_now)) { fault_=true; return false; }
    std::array<int,4> counts{};
    for (const auto& c : packet.commands) { const auto i=index(c.motor_id); if (i>=0) ++counts[i]; }
    for (auto n:counts) if(n!=1) {fault_=true;return false;}
    for (auto& c : packet.commands) {
      const auto i=index(c.motor_id);
      if (i<0) continue;
      c.position=0.0f; c.velocity=0.0f; c.kp=0.0f; c.kd=0.0f;
      c.torque=static_cast<float>(protocol_torque(command_.torque[i]));
    }
    return true;
  }
  bool ready(int64_t ros_now, Clock::time_point steady_now) const {
    return !(fault_ || !received_ || steady_now<received_at_ ||
        std::chrono::duration<double,std::milli>(steady_now-received_at_).count()>timeout_ ||
        !fresh(ros_now,last_stamp_,timeout_) ||
        !fresh(ros_now,stamp_ns(command_.feedback_header.stamp),feedback_timeout_) ||
        !fresh(ros_now,stamp_ns(command_.target_header.stamp),target_timeout_));
  }
  bool faulted() const {return fault_;}
private:
  double limit_,timeout_,feedback_timeout_,target_timeout_;
  bool received_{false},fault_{false};
  int64_t last_stamp_{0};
  Clock::time_point received_at_{};
  roa_interfaces::msg::RsuTorqueCommand command_{};
};
enum class OwnershipResult { HoldInitialization, Torque, Fault };
// Shared production/mock startup state machine. Existing system manager waits
// for position initialization BEFORE activating the policy. Do not require a
// policy torque before that activation can happen.
class Ownership {
public:
  explicit Ownership(double limit=11.9,double timeout=20.,double state_timeout=20.,double target_timeout=100.)
  : gate_(limit,timeout,state_timeout,target_timeout) {}
  void reset() { gate_.reset(); active_=false; requested_={}; last_apply_={}; previous_.fill(0.); }
  bool receive(const roa_interfaces::msg::RsuTorqueCommand& c,int64_t now,Gate::Clock::time_point t) {
    if (gate_.accept(c,now,t)) return true;
    if (active_) return false;
    gate_.reset(); return true; // invalid startup data cannot arm the gate
  }
  OwnershipResult apply(roa_interfaces::msg::MotorCommandArray& p,int64_t now,Gate::Clock::time_point t) {
    if (!active_) {
      if (p.header.frame_id != "/CTRL/RT_CONTROL") return OwnershipResult::HoldInitialization;
      if (requested_==Gate::Clock::time_point{}) requested_=t;
      if (!gate_.ready(now,t)) {
        return t-requested_>std::chrono::milliseconds(100) ? OwnershipResult::Fault : OwnershipResult::HoldInitialization;
      }
    } else if (p.header.frame_id != "/CTRL/RT_CONTROL") return OwnershipResult::Fault;
    if (!gate_.apply(p,now,t)) return OwnershipResult::Fault;
    // Slew the actually applied protocol torque from zero at handover. The
    // manager may have calculated torque for a while during initialization.
    const double dt=last_apply_==Gate::Clock::time_point{} ? .005 :
      std::clamp(std::chrono::duration<double>(t-last_apply_).count(),0.,.005);
    for (auto& c : p.commands) {
      const int i=index(c.motor_id); if(i<0) continue;
      c.torque=static_cast<float>(std::clamp(double(c.torque),previous_[i]-200.*dt,previous_[i]+200.*dt));
      previous_[i]=c.torque;
    }
    last_apply_=t;
    active_=true; return OwnershipResult::Torque;
  }
  bool active() const { return active_; }
private:
  Gate gate_;
  bool active_{false};
  Gate::Clock::time_point requested_{},last_apply_{};
  std::array<double,4> previous_{};
};
} // namespace rsu_torque
