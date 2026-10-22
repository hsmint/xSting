#include <gtest/gtest.h>
#include "xsting_hardware/link_guard.hpp"
using xsting_hardware::LinkGuard;
TEST(LinkGuard, StartsDisarmed) { LinkGuard g(100);EXPECT_FALSE(g.ready(0));EXPECT_FALSE(g.arm(0)); }
TEST(LinkGuard, InvalidTimeoutRejected) { EXPECT_THROW(LinkGuard(0),std::invalid_argument); }
TEST(LinkGuard, HandshakeNeedsFreshFeedbackAndExplicitArm) {
  LinkGuard g(100);g.connect(7);EXPECT_FALSE(g.arm(0));ASSERT_TRUE(g.feedback(7,1,0));
  EXPECT_FALSE(g.ready(1));ASSERT_TRUE(g.arm(1));EXPECT_TRUE(g.ready(99));
}
TEST(LinkGuard, TimeoutBoundaryLatchesDisarmed) {
  LinkGuard g(100);g.connect(7);g.feedback(7,1,0);g.arm(0);
  EXPECT_FALSE(g.ready(100));g.feedback(7,2,101);EXPECT_FALSE(g.ready(102));
  EXPECT_TRUE(g.arm(102));EXPECT_TRUE(g.ready(103));
}
TEST(LinkGuard, LateFeedbackWithoutPollStillDisarms) {
  LinkGuard g(100);g.connect(7);g.feedback(7,1,0);g.arm(0);
  g.feedback(7,2,150);EXPECT_FALSE(g.ready(151));
}
TEST(LinkGuard, ReplayedFeedbackCannotExtendTimeout) {
  LinkGuard g(100);g.connect(7);g.feedback(7,5,0);g.arm(0);
  EXPECT_FALSE(g.feedback(7,5,99));EXPECT_FALSE(g.feedback(7,4,99));EXPECT_FALSE(g.ready(100));
}
TEST(LinkGuard, RebootRequiresNewHandshakeAndRearm) {
  LinkGuard g(100);g.connect(7);g.feedback(7,5,0);g.arm(0);
  EXPECT_FALSE(g.feedback(8,1,1));EXPECT_FALSE(g.arm(2));
  g.connect(8);g.feedback(8,1,3);EXPECT_FALSE(g.ready(4));EXPECT_TRUE(g.arm(4));
}
TEST(LinkGuard, BackwardFeedbackClockDisconnects) {
  LinkGuard g(100);g.connect(7);g.feedback(7,1,20);g.arm(20);
  EXPECT_FALSE(g.feedback(7,2,19));EXPECT_FALSE(g.ready(21));
}
TEST(LinkGuard, BackwardPollingClockDisarms) {
  LinkGuard g(100);g.connect(7);g.feedback(7,1,20);g.arm(20);g.ready(30);
  EXPECT_FALSE(g.ready(29));EXPECT_FALSE(g.ready(31));
}
TEST(LinkGuard, DisconnectDropsOldFeedback) {
  LinkGuard g(100);g.connect(7);g.feedback(7,1,0);g.arm(0);g.disconnect();
  EXPECT_FALSE(g.ready(1));g.connect(7);EXPECT_FALSE(g.arm(2));
}
