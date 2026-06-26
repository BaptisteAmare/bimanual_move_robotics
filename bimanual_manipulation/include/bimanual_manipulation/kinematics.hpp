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
  bool ik(
    const Eigen::Isometry3d & goal, const std::vector<double> & seed,
    std::vector<double> & q) const;

  size_t dof() const {return chain_.getNrOfJoints();}
  const std::vector<std::pair<double, double>> & limits() const {return limits_;}

private:
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_;
  std::vector<std::pair<double, double>> limits_;  // (lower, upper) per joint
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_KINEMATICS_HPP
