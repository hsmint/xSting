#include <gtest/gtest.h>
#include <atomic>
#include <limits>
#include <thread>
#include "xsting_robot/command_check.hpp"
#include "xsting_robot/latest_slot.hpp"
#include "xsting_robot/qos.hpp"
using xsting_robot::reject_reason;
class ArmContract : public ::testing::Test {
 protected:
  xsting_robot::CommandContext c;
  xsting_interfaces::msg::ArmTarget t;
  void SetUp() override {
    c.task_id="task-1";c.model_version="v001";c.frame_id="arm_base";
    c.generation=2;c.input_id=10;c.now_ns=1000000000LL;
    c.input_stamp_ns=950000000;c.joint_stamp_ns=980000000;c.max_age_ns=100000000;c.active=true;
    c.joint_names={"j1","j2"};c.positions={0,0};c.lower={-1,-1};c.upper={1,1};c.max_delta={0.1,0.1};
    t.task_id=c.task_id;t.model_version=c.model_version;t.generation=c.generation;t.input_id=c.input_id;
    t.header.frame_id=c.frame_id;t.header.stamp.sec=1;t.expires_at.sec=1;t.expires_at.nanosec=50000000;
    t.joint_names=c.joint_names;t.delta_positions={0.01,-0.02};
  }
};
TEST_F(ArmContract, ValidCandidateAccepted) { EXPECT_TRUE(reject_reason(t,c).empty()); }
TEST_F(ArmContract, CancelledTaskRejected) { c.active=false;EXPECT_EQ(reject_reason(t,c),"inactive"); }
TEST_F(ArmContract, PreviousGenerationRejected) { --t.generation;EXPECT_EQ(reject_reason(t,c),"identity"); }
TEST_F(ArmContract, PreviousObservationRejected) { --t.input_id;EXPECT_EQ(reject_reason(t,c),"identity"); }
TEST_F(ArmContract, WrongTaskRejected) { t.task_id="other";EXPECT_EQ(reject_reason(t,c),"identity"); }
TEST_F(ArmContract, OldModelRejected) { t.model_version="v000";EXPECT_EQ(reject_reason(t,c),"model"); }
TEST_F(ArmContract, FrameMismatchRejected) { t.header.frame_id="camera";EXPECT_EQ(reject_reason(t,c),"frame"); }
TEST_F(ArmContract, DeadlineBoundaryRejected) { t.expires_at=t.header.stamp;EXPECT_EQ(reject_reason(t,c),"stale"); }
TEST_F(ArmContract, FutureOutputRejected) { t.header.stamp.nanosec=1;EXPECT_EQ(reject_reason(t,c),"stale"); }
TEST_F(ArmContract, OldInputRejectedEvenWithNewOutput) { c.input_stamp_ns=800000000;EXPECT_EQ(reject_reason(t,c),"stale"); }
TEST_F(ArmContract, OldJointStateRejected) { c.joint_stamp_ns=800000000;EXPECT_EQ(reject_reason(t,c),"stale"); }
TEST_F(ArmContract, InvalidNanosecondsRejected) { t.header.stamp.nanosec=1000000000;EXPECT_EQ(reject_reason(t,c),"time"); }
TEST_F(ArmContract, ReorderedJointsRejected) { std::swap(t.joint_names[0],t.joint_names[1]);EXPECT_EQ(reject_reason(t,c),"shape"); }
TEST_F(ArmContract, WrongDimensionsRejected) { t.delta_positions.pop_back();EXPECT_EQ(reject_reason(t,c),"shape"); }
TEST_F(ArmContract, DuplicateJointNamesRejected) { t.joint_names={"j1","j1"};c.joint_names=t.joint_names;EXPECT_EQ(reject_reason(t,c),"names"); }
TEST_F(ArmContract, NonFiniteRejected) { t.delta_positions[0]=std::numeric_limits<double>::quiet_NaN();EXPECT_EQ(reject_reason(t,c),"nonfinite"); }
TEST_F(ArmContract, ExcessDeltaRejected) { t.delta_positions[0]=0.101;EXPECT_EQ(reject_reason(t,c),"limits"); }
TEST_F(ArmContract, FinalJointLimitRejected) { c.positions[0]=0.999;EXPECT_EQ(reject_reason(t,c),"limits"); }
TEST_F(ArmContract, InvalidConfigurationRejected) { c.lower[0]=c.upper[0];EXPECT_EQ(reject_reason(t,c),"limits_config"); }
TEST(LatestInput, DropsWaitingOldItemsButPreservesRunningInput) {
  xsting_robot::LatestSlot<int> slot;slot.put(1);auto running=slot.take();
  slot.put(2);slot.put(3);EXPECT_EQ(*running,1);EXPECT_EQ(*slot.take(),3);EXPECT_FALSE(slot.take());
}
TEST(LatestInput, ConcurrentProducerConsumerNeverReplays) {
  xsting_robot::LatestSlot<int> slot;std::atomic<bool> done{false};
  std::thread producer([&]{for(int i=1;i<=10000;++i)slot.put(i);done=true;});
  int previous=0;
  while(!done.load()) { auto item=slot.take();if(item){EXPECT_GT(*item,previous);previous=*item;} }
  producer.join();if(auto item=slot.take()){EXPECT_GT(*item,previous);previous=*item;}
  EXPECT_EQ(previous,10000);
}
TEST(ArmQos, SingleShotReliableVolatileDepthOne) {
  auto q=xsting_robot::arm_qos().get_rmw_qos_profile();
  EXPECT_EQ(q.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(q.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(q.history,RMW_QOS_POLICY_HISTORY_KEEP_LAST);EXPECT_EQ(q.depth,1u);
}
