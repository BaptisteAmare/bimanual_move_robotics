// Lightweight collision world built directly on FCL (no MoveIt). It checks
// robot self-collision and collisions against dynamically added world objects.
//
// Forward kinematics for every link is computed with a small URDF walker so a
// single robot joint configuration can be validated in O(num_links).
#ifndef BIMANUAL_MANIPULATION_COLLISION_MODEL_HPP
#define BIMANUAL_MANIPULATION_COLLISION_MODEL_HPP

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <urdf/model.h>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "bimanual_manipulation/types.hpp"

namespace bimanual_manipulation
{

class CollisionModel
{
public:
  // Build the static robot collision geometry from the URDF.
  bool init(
    const urdf::Model & model, const CollisionSettings & settings,
    std::string & error);

  // True when the configuration is collision free. joint_values maps joint
  // name -> value; any joint omitted is treated as 0.
  bool checkState(const std::map<std::string, double> & joint_values) const;

  // --- world object management (thread-safe) -------------------------------
  // Free object, pose expressed in the planning (root) frame.
  bool addObject(
    const std::string & id, const shape_msgs::msg::SolidPrimitive & primitive,
    const Eigen::Isometry3d & pose, std::string & error);
  // Object rigidly attached to a link; pose expressed in that link's frame.
  bool addAttachedObject(
    const std::string & id, const shape_msgs::msg::SolidPrimitive & primitive,
    const std::string & link, const Eigen::Isometry3d & pose_in_link,
    std::string & error);
  bool removeObject(const std::string & id);
  void clearObjects();

  const std::string & rootFrame() const {return root_frame_;}
  bool enabled() const {return settings_.enabled;}

private:
  // A single rigid link in the kinematic walk.
  struct LinkNode
  {
    std::string name;
    int parent = -1;                 // index into nodes_, -1 for the root
    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();  // parent joint origin
    int joint_type = 0;              // urdf::Joint::* constant
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    std::string joint_name;
  };

  // A collision shape rigidly attached to a link.
  struct LinkShape
  {
    int node;                        // index into nodes_
    std::shared_ptr<fcl::CollisionGeometryd> geom;
    Eigen::Isometry3d origin;        // link -> shape
  };

  struct WorldObject
  {
    std::string id;
    std::shared_ptr<fcl::CollisionObjectd> obj;
    Eigen::Isometry3d pose;          // in root frame (or link frame if attached)
    int attached_node = -1;          // -1 when free in the world
  };

  std::shared_ptr<fcl::CollisionGeometryd> makeGeometry(
    const shape_msgs::msg::SolidPrimitive & primitive) const;

  void computeLinkTransforms(
    const std::map<std::string, double> & joint_values,
    std::vector<Eigen::Isometry3d> & out) const;

  CollisionSettings settings_;
  std::string root_frame_;

  std::vector<LinkNode> nodes_;
  std::map<std::string, int> node_index_;
  std::vector<LinkShape> shapes_;

  // Precomputed list of shape index pairs that must be tested for
  // self-collision (i.e. all pairs except the allowed/adjacent ones).
  std::vector<std::pair<int, int>> check_pairs_;

  mutable std::mutex world_mutex_;
  std::vector<WorldObject> world_;
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_COLLISION_MODEL_HPP
