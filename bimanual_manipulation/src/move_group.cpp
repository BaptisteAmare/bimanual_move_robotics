#include "bimanual_manipulation/move_group.hpp"

#include <chrono>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

namespace bimanual_manipulation
{

namespace
{
template<typename FutureT>
bool waitFor(FutureT & fut, double timeout_s)
{
  return fut.wait_for(std::chrono::duration<double>(timeout_s)) ==
         std::future_status::ready;
}
}  // namespace

MoveGroup::MoveGroup(rclcpp::Node * node, GroupConfig config)
: node_(node), config_(std::move(config))
{
  for (const auto & ctrl : config_.controllers) {
    if (ctrl.type == "gripper_command") {
      gripper_clients_[ctrl.name] =
        rclcpp_action::create_client<GripperCommand>(node_, ctrl.name);
    } else {
      fjt_clients_[ctrl.name] =
        rclcpp_action::create_client<FollowJointTrajectory>(node_, ctrl.name);
    }
  }
}

trajectory_msgs::msg::JointTrajectory MoveGroup::sliceFor(
  const ControllerConfig & ctrl,
  const trajectory_msgs::msg::JointTrajectory & traj) const
{
  trajectory_msgs::msg::JointTrajectory out;
  // Map controller joints to their index in the full trajectory.
  std::vector<int> idx;
  for (const auto & jn : ctrl.joints) {
    int found = -1;
    for (size_t k = 0; k < traj.joint_names.size(); ++k) {
      if (traj.joint_names[k] == jn) {found = static_cast<int>(k); break;}
    }
    if (found < 0) {return out;}  // controller joint absent -> nothing to send
    idx.push_back(found);
    out.joint_names.push_back(jn);
  }

  out.points.resize(traj.points.size());
  for (size_t p = 0; p < traj.points.size(); ++p) {
    const auto & src = traj.points[p];
    auto & dst = out.points[p];
    dst.time_from_start = src.time_from_start;
    for (int j : idx) {
      dst.positions.push_back(src.positions.empty() ? 0.0 : src.positions[j]);
      if (!src.velocities.empty()) {dst.velocities.push_back(src.velocities[j]);}
      if (!src.accelerations.empty()) {dst.accelerations.push_back(src.accelerations[j]);}
    }
  }
  return out;
}

bool MoveGroup::execute(
  const trajectory_msgs::msg::JointTrajectory & traj, double timeout_s,
  std::string & error)
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  std::vector<std::shared_future<GoalHandle::SharedPtr>> goal_futures;
  std::vector<std::string> names;

  // Fire every controller goal first so the arms move together.
  for (const auto & ctrl : config_.controllers) {
    if (ctrl.type == "gripper_command") {continue;}
    auto client = fjt_clients_.at(ctrl.name);
    auto sliced = sliceFor(ctrl, traj);
    if (sliced.joint_names.empty()) {continue;}
    if (!client->wait_for_action_server(2s)) {
      error = "controller action server '" + ctrl.name + "' unavailable";
      return false;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = sliced;
    goal_futures.push_back(client->async_send_goal(goal));
    names.push_back(ctrl.name);
  }

  if (goal_futures.empty()) {
    error = "no joint-trajectory controller produced a command";
    return false;
  }

  // Collect accepted goal handles.
  std::vector<std::shared_future<GoalHandle::WrappedResult>> result_futures;
  for (size_t i = 0; i < goal_futures.size(); ++i) {
    if (!waitFor(goal_futures[i], 5.0)) {
      error = "controller '" + names[i] + "' did not respond to goal";
      return false;
    }
    auto handle = goal_futures[i].get();
    if (!handle) {
      error = "controller '" + names[i] + "' rejected the goal";
      return false;
    }
    result_futures.push_back(fjt_clients_.at(names[i])->async_get_result(handle));
  }

  // Wait for every controller to finish.
  for (size_t i = 0; i < result_futures.size(); ++i) {
    if (!waitFor(result_futures[i], timeout_s)) {
      error = "controller '" + names[i] + "' timed out";
      return false;
    }
    auto wrapped = result_futures[i].get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error = "controller '" + names[i] + "' did not succeed";
      return false;
    }
    if (wrapped.result && wrapped.result->error_code != 0) {
      error = "controller '" + names[i] + "' error_code=" +
        std::to_string(wrapped.result->error_code);
      return false;
    }
  }
  return true;
}

bool MoveGroup::sendTrajectoryNoWait(
  const trajectory_msgs::msg::JointTrajectory & traj, std::string & error)
{
  bool sent = false;
  for (const auto & ctrl : config_.controllers) {
    if (ctrl.type == "gripper_command") {continue;}
    auto sliced = sliceFor(ctrl, traj);
    if (sliced.joint_names.empty()) {continue;}
    auto client = fjt_clients_.at(ctrl.name);
    if (!client->action_server_is_ready()) {
      error = "controller '" + ctrl.name + "' not ready";
      continue;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = sliced;
    client->async_send_goal(goal);   // fire and forget; preempts any prior goal
    sent = true;
  }
  return sent;
}

bool MoveGroup::executeGripper(
  double position, double max_effort, double timeout_s, std::string & error)
{
  if (gripper_clients_.empty()) {
    error = "group '" + config_.name + "' has no gripper_command controller";
    return false;
  }
  for (auto & kv : gripper_clients_) {
    if (!kv.second->wait_for_action_server(2s)) {
      error = "gripper action server '" + kv.first + "' unavailable";
      return false;
    }
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = max_effort;
    auto gf = kv.second->async_send_goal(goal);
    if (!waitFor(gf, 5.0)) {
      error = "gripper '" + kv.first + "' did not respond";
      return false;
    }
    auto handle = gf.get();
    if (!handle) {
      error = "gripper '" + kv.first + "' rejected the goal";
      return false;
    }
    auto rf = kv.second->async_get_result(handle);
    if (!waitFor(rf, timeout_s)) {
      error = "gripper '" + kv.first + "' timed out";
      return false;
    }
    if (rf.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      error = "gripper '" + kv.first + "' did not succeed";
      return false;
    }
  }
  return true;
}

}  // namespace bimanual_manipulation
