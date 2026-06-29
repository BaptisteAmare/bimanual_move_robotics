#include "bimanual_manipulation/kinematics.hpp"

#include <algorithm>
#include <limits>

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
  ik_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_, 1e-5, 200);
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
  std::vector<double> & q) const
{
  if (seed.size() != group_dof_) {
    return false;
  }
  const unsigned int n = chain_.getNrOfJoints();
  KDL::JntArray q_init(n);
  for (unsigned int i = 0; i < n; ++i) {
    q_init(i) = seed[chain_to_group_[i]];
  }
  KDL::JntArray q_out(n);
  if (ik_->CartToJnt(q_init, eigenToKdl(goal), q_out) < 0) {
    return false;
  }

  // KDL LMA does not enforce joint limits; reject out-of-range solutions.
  // Start from the seed so any group joint not part of the chain is preserved.
  q = seed;
  for (unsigned int i = 0; i < n; ++i) {
    if (q_out(i) < limits_[i].first - 1e-6 || q_out(i) > limits_[i].second + 1e-6) {
      return false;
    }
    q[chain_to_group_[i]] = q_out(i);
  }
  return true;
}

}  // namespace bimanual_manipulation
