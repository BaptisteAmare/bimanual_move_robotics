#include "bimanual_manipulation/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace bimanual_manipulation
{

namespace
{

using Config = std::vector<double>;

struct Tree
{
  std::vector<Config> nodes;
  std::vector<int> parent;
  int add(const Config & q, int p)
  {
    nodes.push_back(q);
    parent.push_back(p);
    return static_cast<int>(nodes.size()) - 1;
  }
};

double dist(const Config & a, const Config & b)
{
  double s = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {const double d = a[i] - b[i]; s += d * d;}
  return std::sqrt(s);
}

int nearest(const Tree & t, const Config & q)
{
  int best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < t.nodes.size(); ++i) {
    const double d = dist(t.nodes[i], q);
    if (d < best_d) {best_d = d; best = static_cast<int>(i);}
  }
  return best;
}

// Move from `from` toward `to` by at most step; snaps to `to` when within step.
Config steer(const Config & from, const Config & to, double step)
{
  const double d = dist(from, to);
  if (d <= step || d < 1e-9) {return to;}
  Config out(from.size());
  const double r = step / d;
  for (size_t i = 0; i < from.size(); ++i) {out[i] = from[i] + r * (to[i] - from[i]);}
  return out;
}

// Collision-checks the interior of the segment [a, b] at `res`.
bool edgeValid(const Config & a, const Config & b, const StateValidator & valid, double res)
{
  const double d = dist(a, b);
  const int steps = std::max(1, static_cast<int>(std::ceil(d / std::max(res, 1e-4))));
  for (int s = 1; s <= steps; ++s) {
    const double f = static_cast<double>(s) / steps;
    Config q(a.size());
    for (size_t i = 0; i < a.size(); ++i) {q[i] = a[i] + f * (b[i] - a[i]);}
    if (!valid(q)) {return false;}
  }
  return true;
}

enum class Status { REACHED, ADVANCED, TRAPPED };

Status extend(Tree & t, const Config & target, const StateValidator & valid, const RRTConnectOptions & o)
{
  const int near = nearest(t, target);
  const Config q_new = steer(t.nodes[near], target, o.step_size);
  if (!valid(q_new)) {return Status::TRAPPED;}
  if (!edgeValid(t.nodes[near], q_new, valid, o.edge_resolution)) {return Status::TRAPPED;}
  t.add(q_new, near);
  return dist(q_new, target) < 1e-6 ? Status::REACHED : Status::ADVANCED;
}

Status connect(Tree & t, const Config & target, const StateValidator & valid, const RRTConnectOptions & o)
{
  Status s = Status::ADVANCED;
  while (s == Status::ADVANCED) {s = extend(t, target, valid, o);}
  return s;
}

void pathToRoot(const Tree & t, int idx, std::vector<Config> & out)
{
  out.clear();
  for (int i = idx; i >= 0; i = t.parent[i]) {out.push_back(t.nodes[i]);}
}

void shortcut(JointPath & path, const StateValidator & valid, const RRTConnectOptions & o)
{
  if (path.size() < 3) {return;}
  std::mt19937 rng(o.seed ^ 0x9e3779b9u);
  for (int it = 0; it < o.shortcut_iterations && path.size() > 2; ++it) {
    std::uniform_int_distribution<size_t> d(0, path.size() - 1);
    size_t i = d(rng), j = d(rng);
    if (i > j) {std::swap(i, j);}
    if (j <= i + 1) {continue;}
    if (edgeValid(path[i], path[j], valid, o.edge_resolution)) {
      path.erase(path.begin() + i + 1, path.begin() + j);
    }
  }
}

}  // namespace

bool planRRTConnect(
  const std::vector<std::pair<double, double>> & bounds,
  const std::vector<double> & start, const std::vector<double> & goal,
  const StateValidator & valid, const RRTConnectOptions & opt,
  JointPath & path, std::string & error)
{
  if (!valid(start)) {error = "start configuration is in collision"; return false;}
  if (!valid(goal)) {error = "goal configuration is in collision"; return false;}

  Tree start_tree, goal_tree;
  start_tree.add(start, -1);
  goal_tree.add(goal, -1);

  std::mt19937 rng(opt.seed);
  std::vector<std::uniform_real_distribution<double>> sampler;
  sampler.reserve(bounds.size());
  for (const auto & b : bounds) {sampler.emplace_back(b.first, b.second);}

  bool from_start = true;
  for (int iter = 0; iter < opt.max_iterations; ++iter) {
    Config q_rand(bounds.size());
    for (size_t i = 0; i < bounds.size(); ++i) {q_rand[i] = sampler[i](rng);}

    Tree & ta = from_start ? start_tree : goal_tree;
    Tree & tb = from_start ? goal_tree : start_tree;

    if (extend(ta, q_rand, valid, opt) != Status::TRAPPED) {
      const Config & q_new = ta.nodes.back();
      if (connect(tb, q_new, valid, opt) == Status::REACHED) {
        const int idx_a = static_cast<int>(ta.nodes.size()) - 1;
        const int idx_b = static_cast<int>(tb.nodes.size()) - 1;
        const int s_idx = from_start ? idx_a : idx_b;
        const int g_idx = from_start ? idx_b : idx_a;

        std::vector<Config> s_part, g_part;
        pathToRoot(start_tree, s_idx, s_part);          // s_idx .. start
        std::reverse(s_part.begin(), s_part.end());     // start .. s_idx
        pathToRoot(goal_tree, g_idx, g_part);           // g_idx(==junction) .. goal

        path = s_part;
        for (size_t k = 1; k < g_part.size(); ++k) {path.push_back(g_part[k]);}
        shortcut(path, valid, opt);
        return true;
      }
    }
    from_start = !from_start;
  }

  error = "RRT-Connect reached the iteration limit without connecting";
  return false;
}

}  // namespace bimanual_manipulation
