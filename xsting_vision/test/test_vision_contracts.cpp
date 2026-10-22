#include <gtest/gtest.h>
#include <limits>
#include "xsting_vision/target_check.hpp"
using xsting_vision::reject_reason;
class VisionContract:public ::testing::Test {
 protected:
  vision_msgs::msg::Detection3D d;
  xsting_vision::TargetRules rules{"arm_base",100000000,0.8};
  void SetUp() override {
    d.id="box-1";d.header.frame_id="arm_base";d.header.stamp.sec=1;
    vision_msgs::msg::ObjectHypothesisWithPose h;h.hypothesis.class_id="box";h.hypothesis.score=0.9;h.pose.pose.orientation.w=1;
    d.results.push_back(h);
  }
};
TEST_F(VisionContract, ValidTargetAccepted) { EXPECT_TRUE(reject_reason(d,1050000000,rules).empty()); }
TEST_F(VisionContract, AgeBoundaryAccepted) { EXPECT_TRUE(reject_reason(d,1100000000,rules).empty()); }
TEST_F(VisionContract, OldTargetRejected) { EXPECT_EQ(reject_reason(d,1100000001,rules),"stale"); }
TEST_F(VisionContract, FutureStampRejected) { EXPECT_EQ(reject_reason(d,999999999,rules),"stale"); }
TEST_F(VisionContract, MissingTransformIsNotSubstituted) { d.header.frame_id="camera_optical";EXPECT_EQ(reject_reason(d,1050000000,rules),"frame"); }
TEST_F(VisionContract, MissingIdRejected) { d.id.clear();EXPECT_EQ(reject_reason(d,1050000000,rules),"id"); }
TEST_F(VisionContract, NoHypothesisRejected) { d.results.clear();EXPECT_EQ(reject_reason(d,1050000000,rules),"empty"); }
TEST_F(VisionContract, LowConfidenceRejected) { d.results[0].hypothesis.score=0.79;EXPECT_EQ(reject_reason(d,1050000000,rules),"confidence"); }
TEST_F(VisionContract, NonFiniteDepthRejected) { d.results[0].pose.pose.position.z=std::numeric_limits<double>::quiet_NaN();EXPECT_EQ(reject_reason(d,1050000000,rules),"invalid"); }
TEST_F(VisionContract, InvalidProbabilityRejected) { d.results[0].hypothesis.score=1.1;EXPECT_EQ(reject_reason(d,1050000000,rules),"invalid"); }
TEST_F(VisionContract, NonUnitQuaternionRejected) { d.results[0].pose.pose.orientation.w=0;EXPECT_EQ(reject_reason(d,1050000000,rules),"rotation"); }
TEST_F(VisionContract, InvalidConfigRejected) { rules.max_age_ns=0;EXPECT_EQ(reject_reason(d,1050000000,rules),"config"); }
TEST(VisionQos, PeriodicDetectionBestEffortVolatileDepthOne) {
  auto q=xsting_vision::detection_qos().get_rmw_qos_profile();
  EXPECT_EQ(q.reliability,RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(q.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(q.history,RMW_QOS_POLICY_HISTORY_KEEP_LAST);EXPECT_EQ(q.depth,1u);
}
