#ifndef GLOBAL_PLANNER__THETA_STAR_HPP_
#define GLOBAL_PLANNER__THETA_STAR_HPP_

#include <vector>
#include <memory>
#include <cstdint>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace GlobalPlanner
{
struct PathNode
{
  octomap::point3d position;
  double g_cost = 0.0;
  double h_cost = 0.0;
  int parent_idx = -1;
  bool closed = false;

  double fCost() const
  {
    return g_cost + h_cost;
  }
};

class ThetaStar
{
public:
  struct Config
  {
    double step_size = 0.5;
    double safety_margin = 0.5;
    double min_altitude = 1.0;
    double max_altitude = 30.0;
    double max_planning_time = 5.0;
  };

  ThetaStar() = default;
  explicit ThetaStar(Config config);
  std::vector<octomap::point3d> plan(
    const octomap::point3d & start, const octomap::point3d & goal,
    const octomap::OcTree & map);
  bool lineOfSight(
    const octomap::point3d & a, const octomap::point3d & b,
    const octomap::OcTree & map) const;

  const Config & config() const
  {
    return config_;
  }

  void setConfig(const Config & config)
  {
    config_ = config;
  }

private:
  Config config_;

  std::vector<octomap::point3d> getNeighbors(const octomap::point3d & point) const;
  std::vector<octomap::point3d> reconstructPath(
    const std::vector<PathNode> & nodes,
    int goal_idx) const;
  std::vector<octomap::point3d> prunePath(
    const std::vector<octomap::point3d> & path,
    const octomap::OcTree & map) const;

  bool isFree(const octomap::point3d & point, const octomap::OcTree & map) const;
  bool isFreeSafe(const octomap::point3d & point, double margin, const octomap::OcTree & map) const;
};
}

#endif
