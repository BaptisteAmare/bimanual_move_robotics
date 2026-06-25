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

  // Collect joint limits in chain order, reading them from the URDF.
  limits_.clear();
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;  // fixed joint, contributes no DoF
    }
    auto uj = model.getJoint(joint.getName());
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
  if (q.size() != chain_.getNrOfJoints()) {
    return false;
  }
  KDL::JntArray ja(chain_.getNrOfJoints());
  for (size_t i = 0; i < q.size(); ++i) {
    ja(i) = q[i];
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
  const unsigned int n = chain_.getNrOfJoints();
  if (seed.size() != n) {
    return false;
  }
  KDL::JntArray q_init(n);
  for (unsigned int i = 0; i < n; ++i) {
    q_init(i) = seed[i];
  }
  KDL::JntArray q_out(n);
  if (ik_->CartToJnt(q_init, eigenToKdl(goal), q_out) < 0) {
    return false;
  }

  // KDL LMA does not enforce joint limits; reject out-of-range solutions.
  q.resize(n);
  for (unsigned int i = 0; i < n; ++i) {
    if (q_out(i) < limits_[i].first - 1e-6 || q_out(i) > limits_[i].second + 1e-6) {
      return false;
    }
    q[i] = q_out(i);
  }
  return true;
}

}  // namespace bimanual_manipulation
