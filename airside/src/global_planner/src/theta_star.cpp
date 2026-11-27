#include "theta_star.hpp"

#include <cmath>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace GlobalPlanner
{
namespace
{
struct GridKey
{
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const GridKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & k) const
  {
    std::size_t seed = 0;
    auto hash_combine = [&seed](int64_t v) {
        seed ^= std::hash<int64_t>{}(v) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      };

    hash_combine(k.x);
    hash_combine(k.y);
    hash_combine(k.z);
    return seed;
  }
};

inline GridKey toGridKey(const octomap::point3d & p, double step_size)
{
  const double inv_step = 1.0 / (step_size > 1e-4 ? step_size : 0.5);
  return GridKey{
    static_cast<int64_t>(std::round(p.x() * inv_step)),
    static_cast<int64_t>(std::round(p.y() * inv_step)),
    static_cast<int64_t>(std::round(p.z() * inv_step))
  };
}

struct PQElement
{
  int node_idx;
  double f_cost;

  bool operator>(const PQElement & other) const
  {
    return f_cost > other.f_cost;
  }
};

}

ThetaStar::ThetaStar(Config config)
: config_(config) {}

bool ThetaStar::isFree(const octomap::point3d & point, const octomap::OcTree & map) const
{
  octomap::OcTreeNode * node = map.search(point);

  return !node || !map.isNodeOccupied(node);
}

bool ThetaStar::isFreeSafe(
  const octomap::point3d & point, double margin,
  const octomap::OcTree & map) const
{
  if (!isFree(point, map)) {
    return false;
  }

  if (margin <= 1e-4) {
    return true;
  }

  const float margin_f = static_cast<float>(margin);
  const octomap::point3d min_pt(point.x() - margin_f, point.y() - margin_f, point.z() - margin_f);
  const octomap::point3d max_pt(point.x() + margin_f, point.y() + margin_f, point.z() + margin_f);
  const double margin_sq = margin * margin;

  for (octomap::OcTree::leaf_bbx_iterator it = map.begin_leafs_bbx(min_pt, max_pt),
    end = map.end_leafs_bbx();
    it != end; ++it)
  {
    if (!map.isNodeOccupied(*it)) {
      continue;
    }

    const octomap::point3d node_pos = it.getCoordinate();
    const double half_size = it.getSize() * 0.5;
    const double cx =
      std::max(
      node_pos.x() - half_size,
      std::min(static_cast<double>(point.x()), node_pos.x() + half_size));
    const double cy =
      std::max(
      node_pos.y() - half_size,
      std::min(static_cast<double>(point.y()), node_pos.y() + half_size));
    const double cz =
      std::max(
      node_pos.z() - half_size,
      std::min(static_cast<double>(point.z()), node_pos.z() + half_size));
    const double dx = point.x() - cx;
    const double dy = point.y() - cy;
    const double dz = point.z() - cz;

    if (dx * dx + dy * dy + dz * dz <= margin_sq) {
      return false;
    }
  }
  return true;
}

std::vector<octomap::point3d> ThetaStar::getNeighbors(const octomap::point3d & point) const
{
  std::vector<octomap::point3d> neighbors;
  neighbors.reserve(26);

  const float step = static_cast<float>(config_.step_size);

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }

        octomap::point3d neighbor(point.x() + static_cast<float>(dx) * step,
          point.y() + static_cast<float>(dy) * step, point.z() + static_cast<float>(dz) * step);

        // Check altitudes
        if (neighbor.z() > config_.max_altitude) {
          continue;
        }
        if (neighbor.z() < config_.min_altitude) {
          if (point.z() < config_.min_altitude && neighbor.z() > point.z()) {} else {
            continue;
          }
        }
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}

bool ThetaStar::lineOfSight(
  const octomap::point3d & a, const octomap::point3d & b,
  const octomap::OcTree & map) const
{
  const double dist = a.distance(b);
  if (dist < 1e-4) {
    return isFreeSafe(a, config_.safety_margin, map);
  }

  double ds = std::min(config_.step_size * 0.5, config_.safety_margin * 0.5);
  if (ds < 0.05) {
    ds = 0.05;
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(dist / ds)));
  const octomap::point3d delta = (b - a) * (1.0f / static_cast<float>(steps));

  for (int i = 0; i <= steps; ++i) {
    octomap::point3d pt;

    if (i == steps) {
      pt = b;
    } else {
      pt = a + delta * static_cast<float>(i);
    }

    if (pt.z() < config_.min_altitude || pt.z() > config_.max_altitude) {
      const bool near_endpoint =
        (pt.distance(a) <= config_.step_size && a.z() < config_.min_altitude) ||
        (pt.distance(b) <= config_.step_size && b.z() < config_.min_altitude);

      if (!near_endpoint) {
        return false;
      }
    }

    if (!isFreeSafe(pt, config_.safety_margin, map)) {
      return false;
    }
  }
  return true;
}

std::vector<octomap::point3d> ThetaStar::plan(
  const octomap::point3d & start,
  const octomap::point3d & goal,
  const octomap::OcTree & map)
{
  if (start.z() > config_.max_altitude || goal.z() > config_.max_altitude ||
    !isFree(start, map) || !isFreeSafe(goal, config_.safety_margin, map))
  {
    return{}
  }

  std::vector<PathNode> nodes;
  nodes.reserve(1024);

  std::unordered_map<GridKey, int, GridKeyHash> node_map;
  node_map.reserve(1024);

  std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;

  PathNode start_node;
  start_node.position = start;
  start_node.h_cost = start.distance(goal);

  nodes.push_back(start_node);
  node_map[toGridKey(start, config_.step_size)] = 0;
  open_set.push({0, start_node.fCost()});

  const auto start_time = std::chrono::steady_clock::now();
  uint64_t iterations = 0;

  while (!open_set.empty()) {
    if ((++iterations & 0x7F) == 0) {       // check for timeout
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time
      ).count();
      if (elapsed > config_.max_planning_time) {
        return{};
      }
    }

    const int curr_idx = open_set.top().node_idx;
    open_set.pop();

    if (nodes[curr_idx].closed) {
      continue;
    }
    nodes[curr_idx].closed = true;

    const octomap::point3d curr_pos = nodes[curr_idx].position;

    // Lazy Theta*
    const int p_idx = nodes[curr_idx].parent_idx;
    const int gp_idx = (p_idx >= 0) ? nodes[p_idx].parent_idx : -1;
    if (gp_idx >= 0) {
      const double los_g = nodes[gp_idx].g_cost + nodes[gp_idx].position.distance(curr_pos);
      if (los_g < nodes[curr_idx].g_cost &&
        lineOfSight(nodes[gp_idx].position, curr_pos, map))
      {
        nodes[curr_idx].g_cost = los_g;
        nodes[curr_idx].parent_idx = gp_idx;
      }
    }

    // Goal reached
    if (curr_pos.distance(goal) <= config_.step_size * 2.0 &&
      lineOfSight(curr_pos, goal, map))
    {
      std::vector<octomap::point3d> path = reconstructPath(nodes, curr_idx);
      if (path.back().distance(goal) > 1e-3) {
        path.push_back(goal);
      }
      return path;
    }

    for (const auto & neighbor : getNeighbors(curr_pos)) {
      if (!isFreeSafe(neighbor, config_.safety_margin, map) ||
        !lineOfSight(curr_pos, neighbor, map))
      {
        continue;
      }

      const double tentative_g = nodes[curr_idx].g_cost + curr_pos.distance(neighbor);
      const GridKey n_key = toGridKey(neighbor, config_.step_size);
      const auto it = node_map.find(n_key);

      if (it == node_map.end()) {
        const int new_idx = static_cast<int>(nodes.size());
        PathNode new_node;
        new_node.position = neighbor;
        new_node.g_cost = tentative_g;
        new_node.h_cost = neighbor.distance(goal);
        new_node.parent_idx = curr_idx;

        nodes.push_back(new_node);
        node_map[n_key] = new_idx;
        open_set.push({new_idx, new_node.fCost()});
      } else if (!nodes[it->second].closed && tentative_g < nodes[it->second].g_cost) {
        nodes[it->second].g_cost = tentative_g;
        nodes[it->second].parent_idx = curr_idx;
        open_set.push({it->second, nodes[it->second].fCost()});
      }
    }
  }

  return{};        // No path found
}

std::vector<octomap::point3d> ThetaStar::reconstructPath(
  const std::vector<PathNode> & nodes,
  int goal_idx) const
{
  std::vector<octomap::point3d> path;

  for (int curr = goal_idx; curr >= 0; curr = nodes[curr].parent_idx) {
    path.push_back(nodes[curr].position);
  }

  std::reverse(path.begin(), path.end());
  return path;
}
}
