#pragma once

#include "rtabmap_msgs/msg/link.hpp"
#include "rtabmap_msgs/msg/map_graph.hpp"

namespace rtabmap_slam::conditional_commit
{

[[nodiscard]] rtabmap_msgs::msg::Link canonicalizeLinkPayload(
  const rtabmap_msgs::msg::Link & value);

[[nodiscard]] bool sameLinkPayload(
  const rtabmap_msgs::msg::Link & lhs,
  const rtabmap_msgs::msg::Link & rhs) noexcept;

[[nodiscard]] bool sameMapGraphPayload(
  const rtabmap_msgs::msg::MapGraph & lhs,
  const rtabmap_msgs::msg::MapGraph & rhs) noexcept;

[[nodiscard]] bool hasAnyLinkBetween(
  const rtabmap_msgs::msg::MapGraph & graph,
  int from_id,
  int to_id) noexcept;

[[nodiscard]] bool hasExactLink(
  const rtabmap_msgs::msg::MapGraph & graph,
  const rtabmap_msgs::msg::Link & expected) noexcept;

}  // namespace rtabmap_slam::conditional_commit
