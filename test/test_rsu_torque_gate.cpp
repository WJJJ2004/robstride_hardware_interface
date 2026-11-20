#include <gtest/gtest.h>
#include "robstride_rdk_ros2/RsuTorqueGate.hpp"
#include "robstride_rdk_ros2/RobStrideMotor.hpp"
using namespace rsu_torque;
namespace {
const int64_t epoch=1000000000LL;
auto steady=Gate::Clock::now();
roa_interfaces::msg::RsuTorqueCommand command() {
 roa_interfaces::msg::RsuTorqueCommand c;c.valid=true;
 c.header.stamp.sec=1;c.feedback_header=c.header;c.target_header=c.header;
 c.torque={1,-2,3,-4};return c;
}
roa_interfaces::msg::MotorCommandArray packet() {
 roa_interfaces::msg::MotorCommandArray p;
 for(uint16_t id=0;id<=23;++id)if(id!=8){
  roa_interfaces::msg::MotorCommand c;c.motor_id=id;c.kp=20;c.kd=2;c.position=.2;
  p.commands.push_back(c);
 }return p;
}
}
TEST(Gate, OwnershipSignsAndActualCanBytes) {
 Gate g;auto c=command();ASSERT_TRUE(g.accept(c,epoch,steady));auto p=packet();
 ASSERT_TRUE(g.apply(p,epoch+5000000,steady+std::chrono::milliseconds(5)));
 for(const auto& m:p.commands) {
  int i=index(m.motor_id);
  if(i<0){EXPECT_EQ(m.kp,20);EXPECT_FLOAT_EQ(m.position,.2f);continue;}
  EXPECT_EQ(m.kp,0);EXPECT_EQ(m.kd,0);EXPECT_EQ(m.velocity,0);EXPECT_EQ(m.position,0);
  EXPECT_DOUBLE_EQ(m.torque,-c.torque[i]);
  RobStrideMotor motor(nullptr,m.motor_id,ActuatorType::ROBSTRIDE_06);
  auto frame=motor.createMotionCommand(m.torque,0,0,0,0);
  for(int b=4;b<8;++b)EXPECT_EQ(frame.data[b],0);
  const auto encoded=(frame.can_id>>8)&0xffff;
  const double decoded=encoded/65535.0*72.-36.;
  EXPECT_NEAR(decoded,-c.torque[i],72./65535.);
 }
}
TEST(Gate, MissingDuplicateAndNonFiniteRejectWithoutPartialMutation) {
 for(int variant=0;variant<3;++variant){
  Gate g;auto c=command();auto p=packet();
  if(variant==2)c.torque[0]=NAN;
  bool accepted=g.accept(c,epoch,steady);
  if(variant==2){EXPECT_FALSE(accepted);continue;}
  ASSERT_TRUE(accepted);
  if(variant==0)p.commands.erase(p.commands.begin()+17);
  else p.commands.push_back(p.commands[17]);
  EXPECT_FALSE(g.apply(p,epoch,steady));
  for(auto& m:p.commands)EXPECT_EQ(m.kp,20);
 }
}
TEST(Gate, WatchdogsLatchUntilExplicitReset) {
 Gate g;auto c=command();auto p=packet();ASSERT_TRUE(g.accept(c,epoch,steady));
 EXPECT_FALSE(g.apply(p,epoch+21000000,steady+std::chrono::milliseconds(21)));
 c.header.stamp.nanosec=22000000;c.feedback_header=c.header;c.target_header=c.header;
 EXPECT_FALSE(g.accept(c,epoch+22000000,steady+std::chrono::milliseconds(22)));
 g.reset();EXPECT_TRUE(g.accept(c,epoch+22000000,steady+std::chrono::milliseconds(22)));
}
TEST(Gate, RejectReplayAndFutureOldSourcesOverlimit) {
 for(int variant=0;variant<6;++variant){
  Gate g;auto c=command();
  if(variant==0){ASSERT_TRUE(g.accept(c,epoch,steady));}
  if(variant==1)c.header.stamp.sec=2;
  if(variant==2)c.feedback_header.stamp.nanosec=1;
  if(variant==3)c.target_header.stamp.sec=0;
  if(variant==4)c.torque[0]=12.;
  if(variant==5)c.valid=false;
  EXPECT_FALSE(g.accept(c,epoch+(variant==2?30000000:0),steady));
 }
}
TEST(Ownership, InitializationDoesNotDependOnPolicyTorque) {
 Ownership owner;auto p=packet();p.header.frame_id="INIT POS HOLD";
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::HoldInitialization);
 EXPECT_EQ(owner.apply(p,epoch+2000000000,steady+std::chrono::seconds(2)),OwnershipResult::HoldInitialization);
 EXPECT_FALSE(owner.active());
 p.header.frame_id="/CTRL/RT_CONTROL";
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::HoldInitialization);
 auto c=command();ASSERT_TRUE(owner.receive(c,epoch,steady));
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::Torque);EXPECT_TRUE(owner.active());
 c.valid=false;EXPECT_FALSE(owner.receive(c,epoch,steady));
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::Fault);
 owner.reset();EXPECT_FALSE(owner.active());
}
TEST(Ownership, HandoverTimeoutAndNoImplicitFallback) {
 Ownership owner;auto p=packet();p.header.frame_id="/CTRL/RT_CONTROL";
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::HoldInitialization);
 EXPECT_EQ(owner.apply(p,epoch+101000000,steady+std::chrono::milliseconds(101)),OwnershipResult::Fault);
 owner.reset();auto c=command();ASSERT_TRUE(owner.receive(c,epoch,steady));
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::Torque);
 p.header.frame_id="INIT POS HOLD";
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::Fault);
}

TEST(Ownership, HandoverLimitsActualTorqueFromZero) {
 Ownership owner;auto p=packet();p.header.frame_id="/CTRL/RT_CONTROL";
 auto c=command();c.torque={10,-10,10,-10};ASSERT_TRUE(owner.receive(c,epoch,steady));
 EXPECT_EQ(owner.apply(p,epoch,steady),OwnershipResult::Torque);
 for(const auto& m:p.commands)if(index(m.motor_id)>=0)EXPECT_LE(std::abs(m.torque),1.0);
}
