// Fast trajectory generation: dense linear interpolation in joint or Cartesian
// space, every waypoint validated against the collision model, then velocity /
// acceleration limited time parameterization. No sampling-based planner is
// involved, which keeps both compute and execution short.
#ifndef BIMANUAL_MANIPULATION_TRAJECTORY_GENERATOR_HPP
#define BIMANUAL_MANIPULATION_TRAJECTORY_GENERATOR_HPP

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "bimanual_manipulation/kinematics.hpp"

namespace bimanual_manipulation
{

// Collision validator: receives candidate joint values (group order) and
// returns true when the configuration is allowed.
using StateValidator = std::function<bool (const std::vector<double> &)>;

struct Limits
{
  std::vector<double> max_velocity;       // per joint [rad/s or m/s]
  std::vector<double> max_acceleration;   // per joint [rad/s^2 or m/s^2]
};

struct MotionLimits
{
  double velocity_scaling = 0.3;
  double acceleration_scaling = 0.3;
  double joint_resolution = 0.02;         // joint-space validation step [rad]
  double cartesian_step = 0.005;          // Cartesian validation step [m]
  bool limit_acceleration = true;         // false -> constant velocity-limited timing
};

// An ordered list of joint configurations (group order) forming a path.
using JointPath = std::vector<std::vector<double>>;

class TrajectoryGenerator
{
public:
  // --- path generation (geometry only, collision-validated) ---------------

  // Dense straight line in joint space from start to goal.
  static bool jointPath(
    const std::vector<double> & start, const std::vector<double> & goal,
    const MotionLimits & motion, const StateValidator & valid,
    JointPath & path, std::string & error);

  // Dense straight line of the tip in Cartesian space, IK per waypoint.
  static bool cartesianPath(
    const GroupKinematics & kin, const std::vector<double> & start,
    const Eigen::Isometry3d & start_pose, const Eigen::Isometry3d & goal_pose,
    const MotionLimits & motion, const StateValidator & valid,
    JointPath & path, std::string & error);

  // Round the corners of a concatenated path at the given junction indices,
  // each over ~radii[k] (rad) on either side. Re-validates each modified window
  // and reverts it if blending would cause a collision.
  // Returns the number of corners actually rounded.
  static size_t blendJunctions(
    JointPath & path, const std::vector<size_t> & junctions,
    const std::vector<double> & radii, const StateValidator & valid);

  // --- timing --------------------------------------------------------------

  // Velocity/acceleration-limited timing of a joint path (density independent).
  static void toTrajectory(
    const std::vector<std::string> & joints, const JointPath & path,
    const Limits & limits, const MotionLimits & motion,
    trajectory_msgs::msg::JointTrajectory & traj);

  // --- convenience (path + timing) -----------------------------------------

  static bool planJoint(
    const std::vector<std::string> & joints,
    const std::vector<double> & start, const std::vector<double> & goal,
    const Limits & limits, const MotionLimits & motion,
    const StateValidator & valid,
    trajectory_msgs::msg::JointTrajectory & traj, std::string & error);

  static bool planCartesian(
    const std::vector<std::string> & joints, const GroupKinematics & kin,
    const std::vector<double> & start, const Eigen::Isometry3d & start_pose,
    const Eigen::Isometry3d & goal_pose,
    const Limits & limits, const MotionLimits & motion,
    const StateValidator & valid,
    trajectory_msgs::msg::JointTrajectory & traj, std::string & error);
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_TRAJECTORY_GENERATOR_HPP
