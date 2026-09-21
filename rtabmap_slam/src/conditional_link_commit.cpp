#include "rtabmap_slam/conditional_link_commit.hpp"

#include <algorithm>

#include <rtabmap_conversions/MsgConversion.h>

namespace rtabmap_slam::conditional_commit
{
namespace
{

bool sameTransform(
  const geometry_msgs::msg::Transform & lhs,
  const geometry_msgs::msg::Transform & rhs) noexcept
{
  return lhs.translation.x == rhs.translation.x &&
         lhs.translation.y == rhs.translation.y &&
         lhs.translation.z == rhs.translation.z &&
         lhs.rotation.x == rhs.rotation.x &&
         lhs.rotation.y == rhs.rotation.y &&
         lhs.rotation.z == rhs.rotation.z &&
         lhs.rotation.w == rhs.rotation.w;
}

bool samePose(
  const geometry_msgs::msg::Pose & lhs,
  const geometry_msgs::msg::Pose & rhs) noexcept
{
  return lhs.position.x == rhs.position.x &&
         lhs.position.y == rhs.position.y &&
         lhs.position.z == rhs.position.z &&
         lhs.orientation.x == rhs.orientation.x &&
         lhs.orientation.y == rhs.orientation.y &&
         lhs.orientation.z == rhs.orientation.z &&
         lhs.orientation.w == rhs.orientation.w;
}

bool sameUnorderedPair(
  const int lhs_from,
  const int lhs_to,
  const int rhs_from,
  const int rhs_to) noexcept
{
  return (lhs_from == rhs_from && lhs_to == rhs_to) ||
         (lhs_from == rhs_to && lhs_to == rhs_from);
}

}  // namespace

rtabmap_msgs::msg::Link canonicalizeLinkPayload(
  const rtabmap_msgs::msg::Link & value)
{
  rtabmap_msgs::msg::Link canonical;
  rtabmap_conversions::linkToROS(
    rtabmap_conversions::linkFromROS(value), canonical);
  return canonical;
}

bool sameLinkPayload(
  const rtabmap_msgs::msg::Link & lhs,
  const rtabmap_msgs::msg::Link & rhs) noexcept
{
  return lhs.from_id == rhs.from_id &&
         lhs.to_id == rhs.to_id &&
         lhs.type == rhs.type &&
         sameTransform(lhs.transform, rhs.transform) &&
         lhs.information == rhs.information;
}

bool sameMapGraphPayload(
  const rtabmap_msgs::msg::MapGraph & lhs,
  const rtabmap_msgs::msg::MapGraph & rhs) noexcept
{
  if (!sameTransform(lhs.map_to_odom, rhs.map_to_odom) ||
    lhs.poses_id != rhs.poses_id ||
    lhs.poses.size() != rhs.poses.size() ||
    lhs.links.size() != rhs.links.size())
  {
    return false;
  }

  for (std::size_t index = 0U; index < lhs.poses.size(); ++index) {
    if (!samePose(lhs.poses[index], rhs.poses[index])) {
      return false;
    }
  }
  for (std::size_t index = 0U; index < lhs.links.size(); ++index) {
    if (!sameLinkPayload(lhs.links[index], rhs.links[index])) {
      return false;
    }
  }
  return true;
}

bool hasAnyLinkBetween(
  const rtabmap_msgs::msg::MapGraph & graph,
  const int from_id,
  const int to_id) noexcept
{
  return std::any_of(
    graph.links.cbegin(), graph.links.cend(),
    [from_id, to_id](const rtabmap_msgs::msg::Link & link) {
      return sameUnorderedPair(
        link.from_id, link.to_id, from_id, to_id);
    });
}

bool hasExactLink(
  const rtabmap_msgs::msg::MapGraph & graph,
  const rtabmap_msgs::msg::Link & expected) noexcept
{
  return std::any_of(
    graph.links.cbegin(), graph.links.cend(),
    [&expected](const rtabmap_msgs::msg::Link & link) {
      return sameLinkPayload(link, expected);
    });
}

}  // namespace rtabmap_slam::conditional_commit
