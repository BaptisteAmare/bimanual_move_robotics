#include "bimanual_manipulation/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/duration.hpp>

namespace bimanual_manipulation
{

namespace
{
constexpr double kMinSegmentTime = 1e-3;     // [s]
constexpr double kCartesianAngularStep = 0.05;  // [rad] per Cartesian waypoint
}  // namespace

bool TrajectoryGenerator::planJoint(
  const std::vector<std::string> & joints,
  const std::vector<double> & start, const std::vector<double> & goal,
  const Limits & limits, const MotionLimits & motion,
  const StateValidator & valid,
  trajectory_msgs::msg::JointTrajectory & traj, std::string & error)
{
  const size_t n = joints.size();
  if (start.size() != n || goal.size() != n) {
    error = "joint vector size mismatch";
    return false;
  }

  double max_delta = 0.0;
  for (size_t i = 0; i < n; ++i) {
    max_delta = std::max(max_delta, std::abs(goal[i] - start[i]));
  }

  size_t steps = static_cast<size_t>(std::ceil(max_delta / std::max(motion.joint_resolution, 1e-4)));
  steps = std::clamp<size_t>(steps, 1, 5000);

  std::vector<std::vector<double>> waypoints;
  waypoints.reserve(steps + 1);
  for (size_t s = 0; s <= steps; ++s) {
    const double f = static_cast<double>(s) / static_cast<double>(steps);
    std::vector<double> q(n);
    for (size_t i = 0; i < n; ++i) {
      q[i] = start[i] + f * (goal[i] - start[i]);
    }
    if (!valid(q)) {
      error = "collision detected along joint path at fraction " + std::to_string(f);
      return false;
    }
    waypoints.push_back(std::move(q));
  }

  timeParameterize(joints, waypoints, limits, motion, traj);
  return true;
}

bool TrajectoryGenerator::planCartesian(
  const std::vector<std::string> & joints, const GroupKinematics & kin,
  const std::vector<double> & start, const Eigen::Isometry3d & start_pose,
  const Eigen::Isometry3d & goal_pose,
  const Limits & limits, const MotionLimits & motion,
  const StateValidator & valid,
  trajectory_msgs::msg::JointTrajectory & traj, std::string & error)
{
  const Eigen::Vector3d p0 = start_pose.translation();
  const Eigen::Vector3d p1 = goal_pose.translation();
  const Eigen::Quaterniond q0(start_pose.linear());
  Eigen::Quaterniond q1(goal_pose.linear());
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;  // shortest path slerp
  }

  const double dist = (p1 - p0).norm();
  const double angle = q0.angularDistance(q1);
  size_t steps = static_cast<size_t>(std::ceil(
      std::max(dist / std::max(motion.cartesian_step, 1e-4),
      angle / kCartesianAngularStep)));
  steps = std::clamp<size_t>(steps, 1, 5000);

  std::vector<std::vector<double>> waypoints;
  waypoints.reserve(steps + 1);
  std::vector<double> seed = start;

  for (size_t s = 0; s <= steps; ++s) {
    const double f = static_cast<double>(s) / static_cast<double>(steps);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = p0 + f * (p1 - p0);
    pose.linear() = q0.slerp(f, q1).toRotationMatrix();

    std::vector<double> q;
    if (!kin.ik(pose, seed, q)) {
      error = "IK failed at Cartesian fraction " + std::to_string(f);
      return false;
    }
    if (!valid(q)) {
      error = "collision detected along Cartesian path at fraction " + std::to_string(f);
      return false;
    }
    seed = q;
    waypoints.push_back(std::move(q));
  }

  timeParameterize(joints, waypoints, limits, motion, traj);
  return true;
}

void TrajectoryGenerator::timeParameterize(
  const std::vector<std::string> & joints,
  const std::vector<std::vector<double>> & waypoints,
  const Limits & limits, const MotionLimits & motion,
  trajectory_msgs::msg::JointTrajectory & traj)
{
  const size_t n = joints.size();
  const double vscale = std::clamp(motion.velocity_scaling, 1e-3, 1.0);
  const double ascale = std::clamp(motion.acceleration_scaling, 1e-3, 1.0);

  traj.joint_names = joints;
  traj.points.clear();
  traj.points.resize(waypoints.size());

  // Segment durations limited by velocity, then stretched to respect the
  // acceleration limit when the velocity changes between segments.
  std::vector<double> dt(waypoints.size(), 0.0);
  for (size_t s = 1; s < waypoints.size(); ++s) {
    double seg = kMinSegmentTime;
    for (size_t i = 0; i < n; ++i) {
      const double dq = std::abs(waypoints[s][i] - waypoints[s - 1][i]);
      const double vmax = std::max(limits.max_velocity[i] * vscale, 1e-6);
      seg = std::max(seg, dq / vmax);
      const double amax = std::max(limits.max_acceleration[i] * ascale, 1e-6);
      seg = std::max(seg, std::sqrt(2.0 * dq / amax));
    }
    dt[s] = seg;
  }

  double t = 0.0;
  for (size_t s = 0; s < waypoints.size(); ++s) {
    t += dt[s];
    auto & pt = traj.points[s];
    pt.positions = waypoints[s];
    pt.velocities.assign(n, 0.0);
    pt.accelerations.assign(n, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
  }

  // Central-difference velocities for smoother tracking; endpoints stay at 0.
  for (size_t s = 1; s + 1 < waypoints.size(); ++s) {
    const double tprev = traj.points[s].time_from_start.sec +
      traj.points[s].time_from_start.nanosec * 1e-9 -
      (traj.points[s - 1].time_from_start.sec +
      traj.points[s - 1].time_from_start.nanosec * 1e-9);
    const double tnext = traj.points[s + 1].time_from_start.sec +
      traj.points[s + 1].time_from_start.nanosec * 1e-9 -
      (traj.points[s].time_from_start.sec +
      traj.points[s].time_from_start.nanosec * 1e-9);
    const double denom = std::max(tprev + tnext, 1e-6);
    for (size_t i = 0; i < n; ++i) {
      traj.points[s].velocities[i] =
        (waypoints[s + 1][i] - waypoints[s - 1][i]) / denom;
    }
  }
}

}  // namespace bimanual_manipulation
