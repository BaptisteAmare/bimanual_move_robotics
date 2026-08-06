#include "bimanual_manipulation/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

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
  jac_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
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
  if (has_locked_) {
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

bool GroupKinematics::ikLocked(
  const Eigen::Isometry3d & goal, const std::vector<double> & seed,
  std::vector<double> & q, bool limit_jump, bool position_only,
  const std::function<bool(const std::vector<double> &)> & accept) const
{
  const unsigned int n = chain_.getNrOfJoints();
  if (n == 0) {return false;}
  const KDL::Frame goal_kdl = eigenToKdl(goal);

  // Damped least-squares descent from a start configuration, holding the locked
  // joints fixed (their Jacobian column and step are zeroed).
  auto solve = [&](KDL::JntArray q_kdl, std::vector<double> & out) -> bool {
      KDL::Jacobian jac(n);
      KDL::Frame f;
      Eigen::Matrix<double, 6, 1> err;
      auto twist = [&](const KDL::Frame & cur) {
          const KDL::Twist t = KDL::diff(cur, goal_kdl);
          err << t.vel.x(), t.vel.y(), t.vel.z(), t.rot.x(), t.rot.y(), t.rot.z();
          if (position_only) {err[3] = err[4] = err[5] = 0.0;}
        };
      for (int iter = 0; iter < 150; ++iter) {
        if (fk_->JntToCart(q_kdl, f) < 0) {return false;}
        twist(f);
        if (err.norm() < 1e-6) {break;}
        if (jac_->JntToJac(q_kdl, jac) < 0) {return false;}
        Eigen::Matrix<double, 6, Eigen::Dynamic> J = jac.data;
        if (position_only) {J.bottomRows(3).setZero();}
        for (unsigned int i = 0; i < n; ++i) {
          if (locked_mask_[i]) {J.col(i).setZero();}
        }
        const double lambda = 0.05;
        const Eigen::Matrix<double, 6, 6> A =
          J * J.transpose() + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(err);
        const double mx = dq.cwiseAbs().maxCoeff();
        if (mx > 0.1) {dq *= 0.1 / mx;}   // clamp the step
        for (unsigned int i = 0; i < n; ++i) {
          if (locked_mask_[i]) {continue;}
          q_kdl(i) = std::clamp(q_kdl(i) + dq(i), limits_[i].first, limits_[i].second);
        }
      }
      if (fk_->JntToCart(q_kdl, f) < 0) {return false;}
      twist(f);
      if (err.norm() > 1e-3) {return false;}   // did not converge

      out = seed;
      for (unsigned int i = 0; i < n; ++i) {
        const int gi = chain_to_group_[i];
        if (q_kdl(i) < limits_[i].first - 1e-6 || q_kdl(i) > limits_[i].second + 1e-6) {
          return false;
        }
        if (limit_jump && !locked_mask_[i] && std::abs(q_kdl(i) - seed[gi]) > kMaxJointJump) {
          return false;
        }
        out[gi] = q_kdl(i);
      }
      if (accept && !accept(out)) {return false;}
      return true;
    };

  KDL::JntArray q0(n);
  for (unsigned int i = 0; i < n; ++i) {q0(i) = seed[chain_to_group_[i]];}
  if (solve(q0, q)) {return true;}

  // Random restarts on the free joints (locked ones stay at their seed value).
  static thread_local std::mt19937 rng(1618033u);
  for (int k = 0; k < 20; ++k) {
    KDL::JntArray qs(n);
    for (unsigned int i = 0; i < n; ++i) {
      const int gi = chain_to_group_[i];
      if (locked_mask_[i]) {qs(i) = seed[gi]; continue;}
      std::uniform_real_distribution<double> jitter(-0.5, 0.5);
      qs(i) = std::clamp(seed[gi] + jitter(rng), limits_[i].first, limits_[i].second);
    }
    if (solve(qs, q)) {return true;}
  }
  return false;
}

}  // namespace bimanual_manipulation
