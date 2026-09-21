#include "rtabmap_slam/conditional_link_commit.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include <rtabmap_conversions/MsgConversion.h>

namespace rc = rtabmap_slam::conditional_commit;

namespace
{

rtabmap_msgs::msg::Link makeLink(
  const int from,
  const int to,
  const double tx)
{
  rtabmap_msgs::msg::Link value;
  value.from_id = from;
  value.to_id = to;
  value.type = 4;
  value.transform.translation.x = tx;
  value.transform.rotation.w = 1.0;
  for (std::size_t i = 0; i < 6U; ++i) {
    value.information[i * 6U + i] =
      static_cast<double>(i + 1U);
  }
  value.information[5] = 0.25;
  value.information[30] = 0.25;
  return value;
}
rtabmap_msgs::msg::MapGraph makeGraph()
{
  rtabmap_msgs::msg::MapGraph value;
  value.map_to_odom.rotation.w = 1.0;
  value.poses_id = {1, 2};
  geometry_msgs::msg::Pose p0;
  geometry_msgs::msg::Pose p1;
  p0.orientation.w = 1.0;
  p1.orientation.w = 1.0;
  p1.position.x = 1.0;
  value.poses = {p0, p1};
  value.links = {makeLink(1, 2, 1.0)};
  return value;
}

}  // namespace

TEST(ConditionalLinkCommit, ExactGraphIgnoresHeader)
{
  auto lhs = makeGraph();
  auto rhs = lhs;
  lhs.header.stamp.sec = 1;
  rhs.header.stamp.sec = 999;
  EXPECT_TRUE(rc::sameMapGraphPayload(lhs, rhs));
}
TEST(ConditionalLinkCommit, DetectsStaleGraphPayload)
{
  const auto expected = makeGraph();

  auto changed_pose = expected;
  changed_pose.poses[1].position.y = 0.01;
  EXPECT_FALSE(rc::sameMapGraphPayload(expected, changed_pose));

  auto changed_link = expected;
  changed_link.links[0].information[0] += 1.0;
  EXPECT_FALSE(rc::sameMapGraphPayload(expected, changed_link));

  auto changed_map_to_odom = expected;
  changed_map_to_odom.map_to_odom.translation.z = 0.1;
  EXPECT_FALSE(
    rc::sameMapGraphPayload(expected, changed_map_to_odom));
}

TEST(ConditionalLinkCommit, RtabRoundTripProducesStableExactPayload)
{
  auto original = makeLink(3, 12, 1.234567890123);
  original.transform.translation.y = -0.456789012345;
  original.transform.translation.z = 0.078901234567;
  original.transform.rotation.x = 0.013;
  original.transform.rotation.y = -0.021;
  original.transform.rotation.z = 0.031;
  original.transform.rotation.w =
    std::sqrt(
      1.0 -
      original.transform.rotation.x * original.transform.rotation.x -
      original.transform.rotation.y * original.transform.rotation.y -
      original.transform.rotation.z * original.transform.rotation.z);
  original.information[1] = -0.125;
  original.information[6] = -0.125;
  original.information[11] = 0.375;
  original.information[30] = 0.375;

  rtabmap_msgs::msg::Link first;
  rtabmap_conversions::linkToROS(
    rtabmap_conversions::linkFromROS(original), first);

  rtabmap_msgs::msg::Link second;
  rtabmap_conversions::linkToROS(
    rtabmap_conversions::linkFromROS(first), second);

  EXPECT_TRUE(rc::sameLinkPayload(first, second));
  EXPECT_EQ(first.information, original.information);
  EXPECT_EQ(second.information, original.information);

  // Non-trivial double transforms are expected to change once because
  // rtabmap::Transform stores float data. The first RTAB round-trip is the
  // canonical wire representation used for exact compare-and-swap ACKs.
  EXPECT_FALSE(rc::sameLinkPayload(original, first));
}

TEST(ConditionalLinkCommit, DetectsExactAndConflictingPair)
{
  const auto candidate = makeLink(5, 2, 3.0);
  rtabmap_msgs::msg::MapGraph value;
  value.links.push_back(candidate);

  EXPECT_TRUE(rc::hasAnyLinkBetween(value, 2, 5));
  EXPECT_TRUE(rc::hasExactLink(value, candidate));

  auto conflict = candidate;
  conflict.transform.translation.x += 0.5;
  EXPECT_TRUE(rc::hasAnyLinkBetween(value, 2, 5));
  EXPECT_FALSE(rc::hasExactLink(value, conflict));
}
