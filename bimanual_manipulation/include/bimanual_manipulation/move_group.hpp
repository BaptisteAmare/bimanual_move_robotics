// Runtime handle for a move group: owns the action clients of the underlying
// joint-trajectory / gripper controllers and dispatches a trajectory to them
// (splitting it per controller so both arms can move simultaneously).
#ifndef BIMANUAL_MANIPULATION_MOVE_GROUP_HPP
#define BIMANUAL_MANIPULATION_MOVE_GROUP_HPP

#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "bimanual_manipulation/types.hpp"

namespace bimanual_manipulation
{

class MoveGroup
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GripperCommand = control_msgs::action::GripperCommand;

  MoveGroup(rclcpp::Node * node, GroupConfig config);

  const GroupConfig & config() const {return config_;}

  // Send a (multi-controller) joint trajectory and block until every
  // controller reports success or the timeout elapses.
  bool execute(
    const trajectory_msgs::msg::JointTrajectory & traj, double timeout_s,
    std::string & error);

  // Actuate every gripper_command controller of this group.
  bool executeGripper(double position, double max_effort, double timeout_s, std::string & error);

  // Fire a trajectory at the controllers without waiting for the result,
  // preempting any goal in progress. Used for continuous servoing (follow).
  bool sendTrajectoryNoWait(
    const trajectory_msgs::msg::JointTrajectory & traj, std::string & error);

private:
  trajectory_msgs::msg::JointTrajectory sliceFor(
    const ControllerConfig & ctrl,
    const trajectory_msgs::msg::JointTrajectory & traj) const;

  rclcpp::Node * node_;
  GroupConfig config_;
  std::map<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> fjt_clients_;
  std::map<std::string, rclcpp_action::Client<GripperCommand>::SharedPtr> gripper_clients_;
};

}  // namespace bimanual_manipulation

#endif  // BIMANUAL_MANIPULATION_MOVE_GROUP_HPP
