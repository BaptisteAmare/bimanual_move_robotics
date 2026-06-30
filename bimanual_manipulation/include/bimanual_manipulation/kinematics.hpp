// Per-group forward / inverse kinematics built on Orocos KDL.
#ifndef BIMANUAL_MANIPULATION_KINEMATICS_HPP
#define BIMANUAL_MANIPULATION_KINEMATICS_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <urdf/model.h>

#include "bimanual_manipulation/types.hpp"

namespace bimanual_manipulation
{

// Wraps a single base_link -> tip_link kinematic chain and provides closed
// loop FK and a fast Levenberg-Marquardt IK. Only used by Cartesian-capable
// groups; aggregate groups (e.g. both arms) do not need it.
class GroupKinematics
{
public:
  bool init(
    const KDL::Tree & tree, const GroupConfig & cfg, const urdf::Model & model,
    std::string & error);

  // Forward kinematics of the tip for the given group joint values.
  bool fkTip(const std::vector<double> & q, Eigen::Isometry3d & pose) const;

  // Inverse kinematics. Returns false if KDL fails to converge or the result
  // violates joint limits. The seed is used as the initial guess.
  //
  // limit_jump=true (Cartesian path following) rejects solutions far from the
  // seed to keep continuity and restarts only with small perturbations.
  // limit_jump=false (one-shot "go to this pose") accepts any in-limit
  // solution and restarts from anywhere in the joint range.
  bool ik(
    const Eigen::Isometry3d & goal, const std::vector<double> & seed,
    std::vector<double> & q, bool limit_jump = true) const;

  size_t dof() const {return chain_.getNrOfJoints();}
  const std::vector<std::pair<double, double>> & limits() const {return limits_;}

private:
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_;
  std::vector<std::pair<double, double>> limits_;  // (lower, upper) per chain joint

  // q vectors handed to / returned by this class are ordered like the group's
  // "joints" list; chain_to_group_[i] is the position, in that list, of the
  // i-th actuated joint of the KDL chain. This makes the YAML joint order
  // irrelevant (everything is matched by name).
  size_t group_dof_ = 0;
  std::vector<int> chain_to_group_;
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_KINEMATICS_HPP
