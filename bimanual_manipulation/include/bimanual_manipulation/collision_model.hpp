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

  // Same, but only checks pairs involving a link that actually moves with one
  // of `active_joints` (links whose transform does not depend on those joints
  // are assumed unchanged, so static-vs-static pairs are skipped). This makes
  // validating a single arm's motion much cheaper.
  bool checkState(
    const std::map<std::string, double> & joint_values,
    const std::set<std::string> & active_joints) const;

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

  // Snapshot of the current world objects, for visualization.
  struct ObjectInfo
  {
    std::string id;
    shape_msgs::msg::SolidPrimitive primitive;
    Eigen::Isometry3d pose;        // root frame, or link frame if attached
    std::string attached_link;     // empty when free in the world
  };
  std::vector<ObjectInfo> objects() const;

  const std::string & rootFrame() const {return root_frame_;}
  bool enabled() const {return settings_.enabled;}

  // Diagnostics (valid after init()).
  size_t shapeCount() const {return shapes_.size();}
  size_t checkPairCount() const {return check_pairs_.size();}
  size_t visualFallbackCount() const {return visual_fallback_links_;}
  size_t meshTotal() const {return meshes_total_;}
  size_t meshFailed() const {return meshes_failed_;}
  size_t meshTriangles() const {return mesh_triangles_;}
  const std::vector<std::string> & linksWithoutCollision() const
  {
    return links_without_collision_;
  }

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
    shape_msgs::msg::SolidPrimitive primitive;  // kept for visualization
    std::string attached_link;
  };

  std::shared_ptr<fcl::CollisionGeometryd> makeGeometry(
    const shape_msgs::msg::SolidPrimitive & primitive) const;

  void computeLinkTransforms(
    const std::map<std::string, double> & joint_values,
    std::vector<Eigen::Isometry3d> & out) const;

  bool checkStateImpl(
    const std::map<std::string, double> & joint_values,
    const std::set<std::string> * active_joints) const;

  CollisionSettings settings_;
  std::string root_frame_;

  std::vector<LinkNode> nodes_;
  std::map<std::string, int> node_index_;
  std::vector<LinkShape> shapes_;
  // Reusable FCL objects for the robot shapes: built once, only their transform
  // is refreshed per check (avoids reallocating 33 objects on every query).
  mutable std::vector<std::shared_ptr<fcl::CollisionObjectd>> shape_objs_;
  std::vector<std::string> links_without_collision_;
  size_t visual_fallback_links_ = 0;
  size_t meshes_total_ = 0;
  size_t meshes_failed_ = 0;
  size_t mesh_triangles_ = 0;

  // Precomputed list of shape index pairs that must be tested for
  // self-collision (i.e. all pairs except the allowed/adjacent ones).
  std::vector<std::pair<int, int>> check_pairs_;

  mutable std::mutex world_mutex_;
  std::vector<WorldObject> world_;
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_COLLISION_MODEL_HPP
