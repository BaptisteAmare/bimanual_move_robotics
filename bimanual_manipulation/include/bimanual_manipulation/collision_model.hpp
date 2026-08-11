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

  // Forward kinematics: pose of `link` (in the model root frame) for the given
  // joint values. False if the link is unknown.
  bool linkPose(
    const std::string & link, const std::map<std::string, double> & joint_values,
    Eigen::Isometry3d & out) const;

  // Human-readable list of the collisions at this configuration — for
  // diagnosing why a state was rejected. When active_joints is given, only the
  // pairs that actually block a move of that group are listed (same filter as
  // checkState); pass nullptr for the full picture.
  std::vector<std::string> describeCollisions(
    const std::map<std::string, double> & joint_values,
    const std::set<std::string> * active_joints = nullptr) const;

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

  // Mesh objects. The resource (package:// or file://) is loaded and, exactly
  // like the robot's own meshes, decimated (mesh mode) or reduced to a sphere
  // proxy (spheres mode) so it stays cheap to collision-check. `scale` scales
  // the mesh (components <= 0 are treated as 1).
  bool addMeshObject(
    const std::string & id, const std::string & resource, const Eigen::Vector3d & scale,
    const Eigen::Isometry3d & pose, std::string & error);
  bool addAttachedMeshObject(
    const std::string & id, const std::string & resource, const Eigen::Vector3d & scale,
    const std::string & link, const Eigen::Isometry3d & pose_in_link, std::string & error);

  bool removeObject(const std::string & id);
  void clearObjects();

  // Snapshot of the current world objects, for visualization.
  struct ObjectInfo
  {
    std::string id;
    shape_msgs::msg::SolidPrimitive primitive;
    Eigen::Isometry3d pose;        // root frame, or link frame if attached
    std::string attached_link;     // empty when free in the world
    bool is_mesh = false;          // true -> mesh_resource/mesh_scale describe it
    std::string mesh_resource;
    Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
  };
  std::vector<ObjectInfo> objects() const;

  const std::string & rootFrame() const {return root_frame_;}
  bool enabled() const {return settings_.enabled;}
  void setEnabled(bool e) {settings_.enabled = e;}

  // Diagnostics (valid after init()).
  size_t shapeCount() const {return sphere_mode_ ? spheres_.size() : shapes_.size();}
  size_t checkPairCount() const
  {
    return sphere_mode_ ? check_node_pairs_.size() : check_pairs_.size();
  }
  bool sphereMode() const {return sphere_mode_;}
  size_t visualFallbackCount() const {return visual_fallback_links_;}
  size_t meshTotal() const {return meshes_total_;}
  size_t meshFailed() const {return meshes_failed_;}
  size_t meshTriangles() const {return mesh_triangles_;}
  size_t autoDisabledCount() const {return auto_disabled_;}
  const std::vector<std::string> & autoDisabledPairs() const {return auto_disabled_names_;}
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

  // A sphere approximating part of a link (spheres mode).
  struct LinkSphere
  {
    int node;                        // index into nodes_
    Eigen::Vector3d center;          // in the link frame
    double radius;
  };

  struct WorldObject
  {
    std::string id;
    std::shared_ptr<fcl::CollisionObjectd> obj;   // FCL geom (mesh mode: BVH)
    Eigen::Isometry3d pose;          // in root frame (or link frame if attached)
    int attached_node = -1;          // -1 when free in the world
    shape_msgs::msg::SolidPrimitive primitive;  // primitive objects (also for viz)
    std::string attached_link;

    // Mesh objects.
    bool is_mesh = false;
    std::string mesh_resource;
    Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
    // Sphere proxy of the mesh (object frame), used in spheres mode. (center, radius)
    std::vector<std::pair<Eigen::Vector3d, double>> mesh_spheres;
    Eigen::Vector3d mesh_bound_center = Eigen::Vector3d::Zero();  // broad-phase
    double mesh_bound_radius = 0.0;
  };

  std::shared_ptr<fcl::CollisionGeometryd> makeGeometry(
    const shape_msgs::msg::SolidPrimitive & primitive) const;

  // Load + reduce a mesh resource into `wo` (BVH geom in mesh mode, sphere
  // proxy in spheres mode). Scale components <= 0 default to 1.
  bool buildMeshObject(
    const std::string & resource, const Eigen::Vector3d & scale,
    WorldObject & wo, std::string & error) const;

  void computeLinkTransforms(
    const std::map<std::string, double> & joint_values,
    std::vector<Eigen::Isometry3d> & out) const;

  // Number of parent hops if one node is an ancestor of the other on the same
  // chain (0 = same, 1 = parent/child, ...), or -1 if not in the same lineage.
  int chainDistance(int a, int b) const;

  // Remove from the check lists the pairs that are in collision in the default
  // pose or in every sampled config (permanent structural contacts), matching
  // MoveIt's "Default"/"Always in collision".
  void autoDisableAlwaysColliding(const urdf::Model & model);

  // Build the per-link sphere approximation (spheres mode).
  void buildSpheres(const urdf::Model & model);
  void appendSpheresForGeometry(
    int node, const urdf::GeometrySharedPtr & geom, const Eigen::Isometry3d & origin);

  bool checkStateImpl(
    const std::map<std::string, double> & joint_values,
    const std::set<std::string> * active_joints) const;
  bool checkStateMesh(
    const std::vector<Eigen::Isometry3d> & tf,
    const std::vector<char> & active, const std::set<std::string> * active_joints) const;
  bool checkStateSpheres(
    const std::vector<Eigen::Isometry3d> & tf,
    const std::vector<char> & active, const std::set<std::string> * active_joints) const;

  CollisionSettings settings_;
  std::string root_frame_;

  std::vector<LinkNode> nodes_;
  std::map<std::string, int> node_index_;
  bool sphere_mode_ = false;
  std::vector<LinkShape> shapes_;
  // Reusable FCL objects for the robot shapes: built once, only their transform
  // is refreshed per check (avoids reallocating 33 objects on every query).
  mutable std::vector<std::shared_ptr<fcl::CollisionObjectd>> shape_objs_;

  // Spheres mode.
  std::vector<LinkSphere> spheres_;
  std::vector<std::vector<int>> spheres_by_node_;   // node -> sphere indices
  std::vector<std::pair<int, int>> check_node_pairs_;  // node pairs to test
  // Per-node bounding sphere (link frame) for a cheap broad-phase reject.
  std::vector<Eigen::Vector3d> node_bound_center_;
  std::vector<double> node_bound_radius_;

  std::vector<std::string> links_without_collision_;
  size_t visual_fallback_links_ = 0;
  size_t meshes_total_ = 0;
  size_t meshes_failed_ = 0;
  size_t mesh_triangles_ = 0;
  size_t auto_disabled_ = 0;
  std::vector<std::string> auto_disabled_names_;

  // Precomputed list of shape index pairs that must be tested for
  // self-collision (i.e. all pairs except the allowed/adjacent ones).
  std::vector<std::pair<int, int>> check_pairs_;

  mutable std::mutex world_mutex_;
  std::vector<WorldObject> world_;
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_COLLISION_MODEL_HPP
