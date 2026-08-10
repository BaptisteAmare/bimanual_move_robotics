#include "bimanual_manipulation/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <string>

#include <Eigen/Dense>

namespace
{
// Per Cartesian waypoint, reject IK solutions that jump further than this from
// the seed (the previous waypoint): they would break the straight-line
// continuity (e.g. an elbow flip) and are unsafe to execute.
constexpr double kMaxJointJump = 1.0;  // rad
}  // namespace

namespace bimanual_manipulation
{

bool GroupKinematics::init(
  const KDL::Tree & tree, const GroupConfig & cfg, const urdf::Model & model,
  std::string & error)
{
  if (cfg.base_link.empty() || cfg.tip_link.empty()) {
    error = "group '" + cfg.name + "' has no base_link/tip_link for kinematics";
    return false;
  }
  if (!tree.getChain(cfg.base_link, cfg.tip_link, chain_)) {
    error = "could not extract KDL chain " + cfg.base_link + " -> " + cfg.tip_link;
    return false;
  }

  group_dof_ = cfg.joints.size();

  // For each actuated chain joint: read its URDF limits and map it to its
  // position in the group's joint list (by name).
  limits_.clear();
  chain_to_group_.clear();
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;  // fixed joint, contributes no DoF
    }
    const std::string & jn = joint.getName();

    int group_idx = -1;
    for (size_t g = 0; g < cfg.joints.size(); ++g) {
      if (cfg.joints[g] == jn) {group_idx = static_cast<int>(g); break;}
    }
    if (group_idx < 0) {
      error = "chain joint '" + jn + "' of group '" + cfg.name +
        "' is not listed in its 'joints'";
      return false;
    }
    chain_to_group_.push_back(group_idx);

    const bool locked =
      std::find(cfg.locked_joints.begin(), cfg.locked_joints.end(), jn) !=
      cfg.locked_joints.end();
    locked_mask_.push_back(locked ? 1 : 0);
    if (locked) {has_locked_ = true;}

    const bool assist =
      std::find(cfg.assist_joints.begin(), cfg.assist_joints.end(), jn) !=
      cfg.assist_joints.end();
    assist_mask_.push_back(assist ? 1 : 0);
    if (assist) {has_assist_ = true;}

    auto uj = model.getJoint(jn);
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    if (uj && uj->limits && uj->type != urdf::Joint::CONTINUOUS) {
      lower = uj->limits->lower;
      upper = uj->limits->upper;
    }
    limits_.emplace_back(lower, upper);
  }

  if (limits_.size() != chain_.getNrOfJoints()) {
    error = "joint-limit count mismatch on group '" + cfg.name + "'";
    return false;
  }

  fk_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_, 1e-5, 500);
  // Position-only solver: zero the orientation task weights so only x/y/z are
  // constrained (orientation left free).
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  ik_pos_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_, weights, 1e-5, 500);
  return true;
}

bool GroupKinematics::fkTip(const std::vector<double> & q, Eigen::Isometry3d & pose) const
{
  if (q.size() != group_dof_) {
    return false;
  }
  const unsigned int n = chain_.getNrOfJoints();
  KDL::JntArray ja(n);
  for (unsigned int i = 0; i < n; ++i) {
    ja(i) = q[chain_to_group_[i]];
  }
  KDL::Frame frame;
  if (fk_->JntToCart(ja, frame) < 0) {
    return false;
  }
  pose = kdlToEigen(frame);
  return true;
}

bool GroupKinematics::ik(
  const Eigen::Isometry3d & goal, const std::vector<double> & seed,
  std::vector<double> & q, bool limit_jump, bool position_only,
  const std::function<bool(const std::vector<double> &)> & accept) const
{
  if (seed.size() != group_dof_) {
    return false;
  }
  if (has_locked_ || has_assist_) {
    return ikLocked(goal, seed, q, limit_jump, position_only, accept);
  }
  const unsigned int n = chain_.getNrOfJoints();
  const KDL::Frame goal_kdl = eigenToKdl(goal);
  const auto & solver = position_only ? ik_pos_ : ik_;

  // Attempt a single IK solve from a given start configuration. Accepts the
  // result only if it respects the joint limits and (when limit_jump) stays
  // close to the seed (continuity of the Cartesian path).
  auto attempt = [&](const std::vector<double> & start, std::vector<double> & out) -> bool {
      KDL::JntArray q_init(n);
      for (unsigned int i = 0; i < n; ++i) {
        q_init(i) = start[chain_to_group_[i]];
      }
      KDL::JntArray q_out(n);
      if (solver->CartToJnt(q_init, goal_kdl, q_out) < 0) {
        return false;
      }
      out = seed;  // preserve any group joint that is not part of the chain
      for (unsigned int i = 0; i < n; ++i) {
        const int gi = chain_to_group_[i];
        if (q_out(i) < limits_[i].first - 1e-6 || q_out(i) > limits_[i].second + 1e-6) {
          return false;
        }
        if (limit_jump && std::abs(q_out(i) - seed[gi]) > kMaxJointJump) {
          return false;  // discontinuous jump w.r.t. the previous waypoint
        }
        out[gi] = q_out(i);
      }
      if (accept && !accept(out)) {
        return false;  // e.g. self-collision: keep searching for another IK
      }
      return true;
    };

  // 1) natural continuation from the seed.
  if (attempt(seed, q)) {
    return true;
  }

  // 2) random restarts to escape LMA local failures. For path following, keep
  //    them close to the seed; for a one-shot goto, explore the whole range.
  static thread_local std::mt19937 rng(2718281u);
  std::vector<double> start = seed;
  for (int k = 0; k < 30; ++k) {
    for (unsigned int i = 0; i < n; ++i) {
      const int gi = chain_to_group_[i];
      double v;
      if (limit_jump) {
        std::uniform_real_distribution<double> jitter(-0.25, 0.25);
        v = seed[gi] + jitter(rng);
      } else if (std::isfinite(limits_[i].first) && std::isfinite(limits_[i].second)) {
        std::uniform_real_distribution<double> uni(limits_[i].first, limits_[i].second);
        v = uni(rng);
      } else {
        std::uniform_real_distribution<double> uni(-3.14159, 3.14159);
        v = seed[gi] + uni(rng);
      }
      start[gi] = std::clamp(v, limits_[i].first, limits_[i].second);
    }
    if (attempt(start, q)) {
      return true;
    }
  }
  return false;
}

void GroupKinematics::ensureSolver(
  const std::vector<char> & fixed_mask, const std::vector<double> & seed,
  ReducedSolver & rs) const
{
  // Baked values (group order) for the currently-fixed joints -> cache key.
  std::vector<double> bv;
  for (size_t i = 0; i < fixed_mask.size(); ++i) {
    if (fixed_mask[i]) {bv.push_back(seed[chain_to_group_[i]]);}
  }
  if (rs.ik && bv.size() == rs.baked_vals.size()) {
    bool same = true;
    for (size_t k = 0; k < bv.size(); ++k) {
      if (std::abs(bv[k] - rs.baked_vals[k]) > 1e-9) {same = false; break;}
    }
    if (same) {return;}
  }

  // Rebuild: bake each fixed joint into a fixed segment at its current value.
  rs.chain = KDL::Chain();
  rs.to_group.clear();
  rs.limits.clear();
  unsigned int aj = 0;   // actuated-joint index in the full chain
  for (unsigned int s = 0; s < chain_.getNrOfSegments(); ++s) {
    const KDL::Segment & seg = chain_.getSegment(s);
    const KDL::Joint & j = seg.getJoint();
    if (j.getType() == KDL::Joint::None) {
      rs.chain.addSegment(seg);   // already fixed
      continue;
    }
    if (fixed_mask[aj]) {
      const double v = seed[chain_to_group_[aj]];
      const KDL::Frame baked = j.pose(v) * seg.getFrameToTip();
      rs.chain.addSegment(
        KDL::Segment(seg.getName(), KDL::Joint(j.getName(), KDL::Joint::None), baked));
    } else {
      rs.chain.addSegment(seg);
      rs.to_group.push_back(chain_to_group_[aj]);
      rs.limits.push_back(limits_[aj]);
    }
    ++aj;
  }

  rs.fk = std::make_shared<KDL::ChainFkSolverPos_recursive>(rs.chain);
  rs.ik = std::make_shared<KDL::ChainIkSolverPos_LMA>(rs.chain, 1e-5, 500);
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  rs.ik_pos = std::make_shared<KDL::ChainIkSolverPos_LMA>(rs.chain, weights, 1e-5, 500);
  rs.baked_vals = bv;
}

bool GroupKinematics::ikLocked(
  const Eigen::Isometry3d & goal, const std::vector<double> & seed,
  std::vector<double> & q, bool limit_jump, bool position_only,
  const std::function<bool(const std::vector<double> &)> & accept) const
{
  const KDL::Frame goal_kdl = eigenToKdl(goal);

  // Solve on a reduced chain (fixed joints held at their seed value).
  auto attempt = [&](const ReducedSolver & rs, const std::vector<double> & start,
      std::vector<double> & out, std::string & why) -> bool {
      const unsigned int nr = rs.chain.getNrOfJoints();
      if (nr == 0) {why = "no free joints"; return false;}
      const auto & solver = position_only ? rs.ik_pos : rs.ik;
      KDL::JntArray q_init(nr), q_out(nr);
      for (unsigned int i = 0; i < nr; ++i) {q_init(i) = start[rs.to_group[i]];}
      const int rc = solver->CartToJnt(q_init, goal_kdl, q_out);
      if (rc < 0) {
        why = "LMA did not converge (rc=" + std::to_string(rc) +
          ", pose unreachable or singular)";
        return false;
      }
      // KDL's LMA ignores joint limits; clamp and re-check the pose is reached
      // (tiny overshoots are absorbed, true violations fail).
      for (unsigned int i = 0; i < nr; ++i) {
        q_out(i) = std::clamp(q_out(i), rs.limits[i].first, rs.limits[i].second);
      }
      KDL::Frame f;
      rs.fk->JntToCart(q_out, f);
      const KDL::Twist e = KDL::diff(f, goal_kdl);
      const double perr = e.vel.Norm();
      const double rerr = position_only ? 0.0 : e.rot.Norm();
      if (perr > 2e-3 || rerr > 5e-3) {
        why = "unreachable within joint limits (residual " + std::to_string(perr) +
          " m, " + std::to_string(rerr) + " rad)";
        return false;
      }
      out = seed;   // fixed joints stay put
      for (unsigned int i = 0; i < nr; ++i) {
        const int gi = rs.to_group[i];
        if (limit_jump && std::abs(q_out(i) - seed[gi]) > kMaxJointJump) {
          why = "free joint jumped " + std::to_string(std::abs(q_out(i) - seed[gi])) +
            " rad from the previous waypoint";
          return false;
        }
        out[gi] = q_out(i);
      }
      if (accept && !accept(out)) {why = "solution in collision"; return false;}
      return true;
    };

  std::string why;

  // 1) Primary: hold the assist joints too, so the arm alone does the work
  //    whenever it can (this matches the plain arm's well-behaved solution).
  if (has_assist_) {
    std::vector<char> fixed(locked_mask_.size(), 0);
    for (size_t i = 0; i < fixed.size(); ++i) {fixed[i] = locked_mask_[i] || assist_mask_[i];}
    ensureSolver(fixed, seed, primary_);
    if (attempt(primary_, seed, q, why)) {return true;}
  }

  // 2) Full: free the assist joints for the extra reach the arm alone lacked.
  ensureSolver(locked_mask_, seed, full_);
  if (attempt(full_, seed, q, why)) {return true;}

  // Path following (limit_jump) must stay continuous with the previous waypoint,
  // so it never jumps to a far restart. Only one-shot goals explore.
  if (limit_jump) {
    std::fprintf(stderr, "[kinematics] locked IK failed (path): %s\n", why.c_str());
    return false;
  }

  static thread_local std::mt19937 rng(1618033u);
  const unsigned int nr = full_.chain.getNrOfJoints();
  std::vector<double> start = seed;
  for (int k = 0; k < 30; ++k) {
    for (unsigned int i = 0; i < nr; ++i) {
      const int gi = full_.to_group[i];
      const double lo = full_.limits[i].first, hi = full_.limits[i].second;
      if (std::isfinite(lo) && std::isfinite(hi)) {
        std::uniform_real_distribution<double> uni(lo, hi);
        start[gi] = uni(rng);
      } else {
        std::uniform_real_distribution<double> uni(-3.14159, 3.14159);
        start[gi] = seed[gi] + uni(rng);
      }
    }
    if (attempt(full_, start, q, why)) {return true;}
  }
  std::fprintf(stderr, "[kinematics] locked IK failed (goal): %s\n", why.c_str());
  return false;
}

}  // namespace bimanual_manipulation
