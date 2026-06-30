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

bool TrajectoryGenerator::jointPath(
  const std::vector<double> & start, const std::vector<double> & goal,
  const MotionLimits & motion, const StateValidator & valid,
  JointPath & path, std::string & error)
{
  const size_t n = start.size();
  if (goal.size() != n) {
    error = "joint vector size mismatch";
    return false;
  }

  double max_delta = 0.0;
  for (size_t i = 0; i < n; ++i) {
    max_delta = std::max(max_delta, std::abs(goal[i] - start[i]));
  }

  size_t steps = static_cast<size_t>(std::ceil(max_delta / std::max(motion.joint_resolution, 1e-4)));
  steps = std::clamp<size_t>(steps, 1, 5000);

  path.clear();
  path.reserve(steps + 1);
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
    path.push_back(std::move(q));
  }
  return true;
}

bool TrajectoryGenerator::cartesianPath(
  const GroupKinematics & kin, const std::vector<double> & start,
  const Eigen::Isometry3d & start_pose, const Eigen::Isometry3d & goal_pose,
  const MotionLimits & motion, const StateValidator & valid,
  JointPath & path, std::string & error)
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

  path.clear();
  path.reserve(steps + 1);
  std::vector<double> seed = start;

  for (size_t s = 0; s <= steps; ++s) {
    const double f = static_cast<double>(s) / static_cast<double>(steps);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = p0 + f * (p1 - p0);
    pose.linear() = q0.slerp(f, q1).toRotationMatrix();

    std::vector<double> q;
    if (!kin.ik(pose, seed, q)) {
      error = "IK failed at Cartesian fraction " + std::to_string(f) + " (" +
        std::to_string(f * dist) + " m of " + std::to_string(dist) +
        " m reached) - target likely out of reach or near a singularity from "
        "the current pose";
      return false;
    }
    if (!valid(q)) {
      error = "collision detected along Cartesian path at fraction " + std::to_string(f);
      return false;
    }
    seed = q;
    path.push_back(std::move(q));
  }
  return true;
}

size_t TrajectoryGenerator::blendJunctions(
  JointPath & path, const std::vector<size_t> & junctions,
  const std::vector<double> & radii, const StateValidator & valid)
{
  if (path.size() < 3 || junctions.size() != radii.size()) {return 0;}
  const size_t n = path.front().size();
  size_t blended_count = 0;

  auto dist = [n](const std::vector<double> & a, const std::vector<double> & b) {
      double s = 0.0;
      for (size_t i = 0; i < n; ++i) {const double d = a[i] - b[i]; s += d * d;}
      return std::sqrt(s);
    };

  for (size_t ji = 0; ji < junctions.size(); ++ji) {
    const double blend_radius = radii[ji];
    if (blend_radius <= 0.0) {continue;}
    const size_t j = junctions[ji];
    if (j == 0 || j + 1 >= path.size()) {continue;}
    const size_t lo_bound = (ji == 0) ? 0 : junctions[ji - 1];
    const size_t hi_bound = (ji + 1 < junctions.size()) ? junctions[ji + 1] : path.size() - 1;
    const std::vector<double> wj = path[j];

    // Try the requested rounding; if it would collide, retry with a smaller
    // radius and keep the largest collision-free corner cut.
    for (double frac : {1.0, 0.6, 0.35, 0.2}) {
      const double r = blend_radius * frac;
      size_t a = j, b = j;
      while (a > lo_bound + 1 && dist(path[a - 1], path[j]) < r) {--a;}
      while (b + 1 < hi_bound && dist(path[b + 1], path[j]) < r) {++b;}
      if (b <= a + 1) {continue;}

      // Quadratic Bezier cutting the corner: endpoints W[a], W[b] kept, the
      // corner W[j] is the control point.
      const std::vector<double> & wa = path[a];
      const std::vector<double> & wb = path[b];
      const double span = static_cast<double>(b - a);
      JointPath blended(b - a + 1, std::vector<double>(n));
      for (size_t k = 0; k <= b - a; ++k) {
        const double t = static_cast<double>(k) / span;
        const double c0 = (1 - t) * (1 - t), c1 = 2 * (1 - t) * t, c2 = t * t;
        for (size_t i = 0; i < n; ++i) {
          blended[k][i] = c0 * wa[i] + c1 * wj[i] + c2 * wb[i];
        }
      }

      bool ok = true;
      for (size_t k = 1; k + 1 < blended.size() && ok; ++k) {ok = valid(blended[k]);}
      if (ok) {
        for (size_t k = 0; k < blended.size(); ++k) {path[a + k] = blended[k];}
        ++blended_count;
        break;
      }
    }
  }
  return blended_count;
}

bool TrajectoryGenerator::planJoint(
  const std::vector<std::string> & joints,
  const std::vector<double> & start, const std::vector<double> & goal,
  const Limits & limits, const MotionLimits & motion,
  const StateValidator & valid,
  trajectory_msgs::msg::JointTrajectory & traj, std::string & error)
{
  JointPath path;
  if (!jointPath(start, goal, motion, valid, path, error)) {return false;}
  toTrajectory(joints, path, limits, motion, traj);
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
  JointPath path;
  if (!cartesianPath(kin, start, start_pose, goal_pose, motion, valid, path, error)) {
    return false;
  }
  toTrajectory(joints, path, limits, motion, traj);
  return true;
}

void TrajectoryGenerator::toTrajectory(
  const std::vector<std::string> & joints, const JointPath & waypoints,
  const Limits & limits, const MotionLimits & motion,
  trajectory_msgs::msg::JointTrajectory & traj)
{
  const size_t W = waypoints.size();
  const size_t n = joints.size();
  const double vscale = std::clamp(motion.velocity_scaling, 1e-3, 1.0);
  const double ascale = std::clamp(motion.acceleration_scaling, 1e-3, 1.0);

  traj.joint_names = joints;
  traj.points.clear();
  traj.points.resize(W);

  // 1) Velocity-limited segment durations. Summed over the path this gives a
  //    total time that is independent of how densely the path was sampled.
  std::vector<double> dt(W, 0.0);
  for (size_t s = 1; s < W; ++s) {
    double seg = kMinSegmentTime;
    for (size_t i = 0; i < n; ++i) {
      const double dq = std::abs(waypoints[s][i] - waypoints[s - 1][i]);
      const double vmax = std::max(limits.max_velocity[i] * vscale, 1e-6);
      seg = std::max(seg, dq / vmax);
    }
    dt[s] = seg;
  }

  // 2) Acceleration limiting, applied LOCALLY and SYMMETRICALLY: each pass
  //    measures the acceleration at every junction from a snapshot of the
  //    current times, then stretches only the offending segments in one shot.
  //    A sharp corner (e.g. a reversal) thus slows the motion only near that
  //    corner, with no left-to-right bias. Iterate until it settles.
  for (int iter = 0; iter < 20; ++iter) {
    std::vector<double> scale(W, 1.0);
    bool changed = false;
    for (size_t s = 1; s + 1 < W; ++s) {
      const double dtm = 0.5 * (dt[s] + dt[s + 1]);
      if (dtm < 1e-9) {continue;}
      double ratio = 1.0;
      for (size_t i = 0; i < n; ++i) {
        const double v_in = (waypoints[s][i] - waypoints[s - 1][i]) / dt[s];
        const double v_out = (waypoints[s + 1][i] - waypoints[s][i]) / dt[s + 1];
        const double a = std::abs(v_out - v_in) / dtm;
        const double amax = std::max(limits.max_acceleration[i] * ascale, 1e-6);
        ratio = std::max(ratio, a / amax);
      }
      if (ratio > 1.0001) {
        const double f = std::sqrt(ratio);  // a ~ 1/time^2
        scale[s] = std::max(scale[s], f);
        scale[s + 1] = std::max(scale[s + 1], f);
        changed = true;
      }
    }
    if (!changed) {break;}
    for (size_t s = 0; s < W; ++s) {dt[s] *= scale[s];}
  }

  // 3) Timestamps.
  double t = 0.0;
  for (size_t s = 0; s < W; ++s) {
    t += dt[s];
    auto & pt = traj.points[s];
    pt.positions = waypoints[s];
    pt.velocities.assign(n, 0.0);
    pt.accelerations.assign(n, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
  }

  // 4) Central-difference velocities for smoother tracking; endpoints stay 0.
  for (size_t s = 1; s + 1 < W; ++s) {
    const double denom = std::max(dt[s] + dt[s + 1], 1e-6);
    for (size_t i = 0; i < n; ++i) {
      traj.points[s].velocities[i] =
        (waypoints[s + 1][i] - waypoints[s - 1][i]) / denom;
    }
  }
}

}  // namespace bimanual_manipulation
