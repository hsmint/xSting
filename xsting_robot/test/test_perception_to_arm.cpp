#include <gtest/gtest.h>
#include "xsting_vision/target_check.hpp"
#include "xsting_robot/command_check.hpp"
TEST(Pipeline, FreshDetectionThenCancelledArmCannotDispatch) {
  vision_msgs::msg::Detection3D detection;detection.id="box";detection.header.frame_id="arm_base";
  detection.header.stamp.sec=1;vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id="box";hypothesis.hypothesis.score=0.99;hypothesis.pose.pose.orientation.w=1;
  detection.results.push_back(hypothesis);
  ASSERT_TRUE(xsting_vision::reject_reason(detection,1000000000,{"arm_base",100000000,0.8}).empty());
  xsting_robot::CommandContext c;c.active=true;c.task_id="pick";c.model_version="test";c.frame_id="arm_base";
  c.generation=1;c.input_id=1;c.now_ns=c.input_stamp_ns=c.joint_stamp_ns=1000000000;c.max_age_ns=100000000;
  c.joint_names={"test_joint"};c.positions={0};c.lower={-1};c.upper={1};c.max_delta={0.1};
  xsting_interfaces::msg::ArmTarget target;target.task_id=c.task_id;target.model_version=c.model_version;
  target.header=detection.header;target.generation=1;target.input_id=1;target.expires_at.sec=2;
  target.joint_names=c.joint_names;target.delta_positions={0.01};
  ASSERT_TRUE(xsting_robot::reject_reason(target,c).empty());
  c.generation=2; // cancellation wins before the final send, even if inference already completed
  EXPECT_EQ(xsting_robot::reject_reason(target,c),"identity");
  EXPECT_EQ(xsting_vision::reject_reason(detection,1200000000,{"arm_base",100000000,0.8}),"stale");
}
