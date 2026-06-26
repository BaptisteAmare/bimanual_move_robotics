// Parses the YAML configuration files (move groups, named poses, collision
// settings and predefined sequences). Everything tunable about the stack lives
// in these files: group/joint/controller names, poses, collisions, sequences.
#ifndef BIMANUAL_MANIPULATION_CONFIG_LOADER_HPP
#define BIMANUAL_MANIPULATION_CONFIG_LOADER_HPP

#include <map>
#include <string>
#include <vector>

#include <bimanual_msgs/msg/motion_step.hpp>

#include "bimanual_manipulation/types.hpp"

namespace bimanual_manipulation
{

struct PlanningDefaults
{
  double velocity_scaling = 0.3;
  double acceleration_scaling = 0.3;
  double joint_velocity = 1.0;        // fallback max joint velocity [rad/s]
  double joint_acceleration = 2.0;    // fallback max joint accel [rad/s^2]
  double execution_timeout = 30.0;    // [s]
};

struct ManipulationConfig
{
  PlanningDefaults defaults;
  CollisionSettings collision;
  std::map<std::string, GroupConfig> groups;
  // group -> pose name -> joint values (group order)
  std::map<std::string, std::map<std::string, std::vector<double>>> named_poses;
  // sequence name -> ordered steps
  std::map<std::string, std::vector<bimanual_msgs::msg::MotionStep>> sequences;
};

// Loads any subset of the four files (empty path -> skipped). Returns false and
// fills `error` on a parse problem.
bool loadConfig(
  const std::string & move_groups_yaml,
  const std::string & named_poses_yaml,
  const std::string & collision_yaml,
  const std::string & sequences_yaml,
  ManipulationConfig & out, std::string & error);

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_CONFIG_LOADER_HPP
