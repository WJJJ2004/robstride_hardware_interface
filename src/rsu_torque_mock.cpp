#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robstride_rdk_ros2/RsuTorqueGate.hpp"
#include "robstride_rdk_ros2/RobStrideMotor.hpp"
#include <cstdlib>
class Mock : public rclcpp::Node {
public:
  Mock(): Node("rsu_torque_mock") {
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    if(!domain || std::string(domain)!="77") throw std::runtime_error("mock requires ROS_DOMAIN_ID=77");
    out_=create_publisher<roa_interfaces::msg::MotorCommandArray>("/replay/applied_command",10);
    status_=create_publisher<std_msgs::msg::Bool>("/replay/frozen",10);
    base_=create_subscription<roa_interfaces::msg::MotorCommandArray>("/hardware_interface/command",10,
      [this](roa_interfaces::msg::MotorCommandArray::SharedPtr p) {
        packet_=*p; base_at_=rsu_torque::Gate::Clock::now();
      });
    torque_=create_subscription<roa_interfaces::msg::RsuTorqueCommand>("/rsu/torque_command",rclcpp::QoS(1).best_effort(),
      [this](roa_interfaces::msg::RsuTorqueCommand::SharedPtr p) {
        // Startup is permitted to wait for a valid stream; once engaged faults latch.
        if (!engaged_ && !p->valid) return;
        if (!gate_.receive(*p,now().nanoseconds(),rsu_torque::Gate::Clock::now())) frozen_=true;
        else engaged_=true;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(5),[this] {
      auto packet=packet_;
      // Production HW cannot reach RUNNING before a complete initialization
      // packet. Emulate that prerequisite despite independent DDS discovery.
      if(engaged_ && !frozen_ && !packet.commands.empty()) {
        auto t=rsu_torque::Gate::Clock::now();
        frozen_=std::chrono::duration<double,std::milli>(t-base_at_).count()>50.0 ||
          !rsu_torque::fresh(now().nanoseconds(),rsu_torque::stamp_ns(packet.header.stamp),50.0) ||
          gate_.apply(packet,now().nanoseconds(),t)==rsu_torque::OwnershipResult::Fault;
        if(!frozen_ && gate_.active()) {
          // Exercise actual protocol encoder offline. Null transport is never used.
          for(const auto& c:packet.commands) if(rsu_torque::index(c.motor_id)>=0) {
            RobStrideMotor motor(nullptr,c.motor_id,ActuatorType::ROBSTRIDE_06);
            auto frame=motor.createMotionCommand(c.torque,c.position,c.velocity,c.kp,c.kd);
            if(frame.data.size()!=8 || frame.data[4]!=0 || frame.data[5]!=0 ||
               frame.data[6]!=0 || frame.data[7]!=0) throw std::runtime_error("nonzero motor gains");
          }
          packet.header.stamp=now(); out_->publish(packet);
        }
      }
      std_msgs::msg::Bool state; state.data=frozen_; status_->publish(state);
    });
  }
private:
  rsu_torque::Ownership gate_;
  bool engaged_{false},frozen_{false};
  rsu_torque::Gate::Clock::time_point base_at_{};
  roa_interfaces::msg::MotorCommandArray packet_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<roa_interfaces::msg::MotorCommandArray>::SharedPtr out_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_;
  rclcpp::Subscription<roa_interfaces::msg::MotorCommandArray>::SharedPtr base_;
  rclcpp::Subscription<roa_interfaces::msg::RsuTorqueCommand>::SharedPtr torque_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Mock>());rclcpp::shutdown();}
