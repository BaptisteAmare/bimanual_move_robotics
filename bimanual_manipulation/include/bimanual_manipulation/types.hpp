// Shared data structures and small geometry helpers used across the
// bimanual_manipulation package.
#ifndef BIMANUAL_MANIPULATION_TYPES_HPP
#define BIMANUAL_MANIPULATION_TYPES_HPP

#include <string>
#include <vector>
#include <utility>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <urdf_model/pose.h>
#include <geometry_msgs/msg/pose.hpp>

namespace bimanual_manipulation
{

// One low-level controller exposed by the robot (a FollowJointTrajectory or a
// GripperCommand action server) together with the subset of group joints it
// drives.
struct ControllerConfig
{
  std::string name;                  // fully-qualified action name
  std::string type;                  // "follow_joint_trajectory" | "gripper_command"
  std::vector<std::string> joints;   // joints driven by this controller
};

// A logical group of joints that can be commanded together. A group either
// maps to a single kinematic chain (base_link -> tip_link, Cartesian capable)
// or aggregates several controllers (e.g. both arms) for joint-space moves.
struct GroupConfig
{
  std::string name;
  std::string base_link;             // kinematic chain root (Cartesian groups)
  std::string tip_link;              // kinematic chain tip (Cartesian groups)
  std::vector<std::string> joints;   // ordered joints owned by the group
  std::vector<ControllerConfig> controllers;

  double default_velocity_scaling = 0.3;
  double default_acceleration_scaling = 0.3;
  double cartesian_step = 0.005;     // Cartesian interpolation resolution [m]
  bool cartesian = false;            // true when base_link/tip_link are set
};

// Global collision-checking settings.
struct CollisionSettings
{
  double padding = 0.0;              // primitive-only geometry inflation [m]
  // Separation margin kept between any two collision elements (self and world,
  // meshes included): a configuration is rejected when the closest distance
  // between two checked elements drops below this. 0 -> plain contact test
  // (fastest). Works on meshes, unlike `padding`.
  double margin = 0.0;
  double resolution = 0.02;          // joint-space validation step [rad]
  // Collision-mesh simplification: vertices are clustered on a grid of this
  // size [m] at load time, drastically cutting triangle count (and therefore
  // collision-check time) on detailed meshes. 0 disables it.
  double mesh_decimation = 0.02;
  bool enabled = true;
  // Pairs of links whose mutual collisions are ignored (like an SRDF ACM).
  std::vector<std::pair<std::string, std::string>> disabled_pairs;
};

// --- geometry conversions --------------------------------------------------

inline Eigen::Isometry3d urdfToEigen(const urdf::Pose & p)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  t.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y,
      p.rotation.z).normalized().toRotationMatrix();
  return t;
}

inline Eigen::Isometry3d kdlToEigen(const KDL::Frame & f)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(f.p.x(), f.p.y(), f.p.z());
  double x, y, z, w;
  f.M.GetQuaternion(x, y, z, w);
  t.linear() = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
  return t;
}

inline KDL::Frame eigenToKdl(const Eigen::Isometry3d & t)
{
  Eigen::Quaterniond q(t.linear());
  const Eigen::Vector3d & p = t.translation();
  return KDL::Frame(
    KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()),
    KDL::Vector(p.x(), p.y(), p.z()));
}

inline Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  t.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
      p.orientation.y, p.orientation.z).normalized().toRotationMatrix();
  return t;
}

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_TYPES_HPP
