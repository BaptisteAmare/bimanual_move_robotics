// Bidirectional RRT-Connect planner in a move group's joint space, used as a
// fallback to route around obstacles when the straight-line path is blocked.
// Validity comes from the same FCL collision validator used everywhere else.
#ifndef BIMANUAL_MANIPULATION_PLANNER_HPP
#define BIMANUAL_MANIPULATION_PLANNER_HPP

#include <string>
#include <utility>
#include <vector>

#include "bimanual_manipulation/trajectory_generator.hpp"  // JointPath, StateValidator

namespace bimanual_manipulation
{

struct RRTConnectOptions
{
  double step_size = 0.25;       // tree extension step [rad]
  double edge_resolution = 0.02; // collision check step along an edge [rad]
  int max_iterations = 4000;
  int shortcut_iterations = 200; // path smoothing attempts
  unsigned seed = 88172645u;
};

// Plans a collision-free joint path from start to goal within `bounds`
// (lower, upper per joint). Returns a shortcut (sparse) path on success; false
// (with a reason) if start/goal are invalid or no path was found in time.
bool planRRTConnect(
  const std::vector<std::pair<double, double>> & bounds,
  const std::vector<double> & start, const std::vector<double> & goal,
  const StateValidator & valid, const RRTConnectOptions & opt,
  JointPath & path, std::string & error);

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_PLANNER_HPP
