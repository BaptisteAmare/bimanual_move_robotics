#include "bimanual_manipulation/manipulation_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <thread>

#include <kdl_parser/kdl_parser.hpp>
#include <srdfdom/model.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "bimanual_manipulation/planner.hpp"
#include "bimanual_manipulation/trajectory_generator.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace bimanual_manipulation
{

ManipulationServer::ManipulationServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("bimanual_manipulation_server", options)
{
  declare_parameter<std::string>("robot_description", "");
  declare_parameter<std::string>("robot_description_file", "");
  declare_parameter<std::string>("robot_description_topic", "/robot_description");
  declare_parameter<std::string>("move_groups_config", "");
  declare_parameter<std::string>("named_poses_config", "");
  declare_parameter<std::string>("collision_config", "");
  declare_parameter<std::string>("sequences_config", "");
  declare_parameter<std::string>("srdf_config", "");
  declare_parameter<double>("gripper_max_effort", 50.0);
}

bool ManipulationServer::loadUrdf(std::string & urdf_xml)
{
  urdf_xml = get_parameter("robot_description").as_string();
  if (!urdf_xml.empty()) {
    return true;
  }

  // A file on disk (e.g. the full description URDF with collision meshes) takes
  // priority over the live topic, which may be a kinematics-only URDF.
  const std::string file = get_parameter("robot_description_file").as_string();
  if (!file.empty()) {
    std::ifstream in(file);
    if (!in) {
      RCLCPP_ERROR(get_logger(), "Cannot open robot_description_file '%s'", file.c_str());
      return false;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    urdf_xml = ss.str();
    return !urdf_xml.empty();
  }

  const std::string topic = get_parameter("robot_description_topic").as_string();
  RCLCPP_INFO(get_logger(), "Waiting for URDF on topic '%s' ...", topic.c_str());

  bool got = false;
  auto sub = create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::SharedPtr msg) {
      urdf_from_topic_ = msg->data;
      got = true;
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(get_node_base_interface());
  const auto start = now();
  while (rclcpp::ok() && !got && (now() - start).seconds() < 10.0) {
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  exec.remove_node(get_node_base_interface());

  urdf_xml = urdf_from_topic_;
  return got;
}

bool ManipulationServer::buildModel(const std::string & urdf_xml, std::string & error)
{
  if (!urdf_model_.initString(urdf_xml)) {
    error = "failed to parse URDF";
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)) {
    error = "failed to build KDL tree from URDF";
    return false;
  }

  // Merge the MoveIt SRDF allowed-collision matrix, if provided.
  const std::string srdf_path = get_parameter("srdf_config").as_string();
  if (!srdf_path.empty()) {
    srdf::Model srdf;
    if (srdf.initFile(urdf_model_, srdf_path)) {
      const auto & pairs = srdf.getDisabledCollisionPairs();
      for (const auto & p : pairs) {
        config_.collision.disabled_pairs.emplace_back(p.link1_, p.link2_);
      }
      RCLCPP_INFO(
        get_logger(), "Loaded %zu disabled collision pairs from SRDF '%s'.",
        pairs.size(), srdf_path.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Could not parse SRDF '%s'.", srdf_path.c_str());
    }
  }

  if (!collision_.init(urdf_model_, config_.collision, error)) {
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Collision model (%s): %zu %s, %zu link pairs checked (%zu auto-disabled as "
    "default/always-colliding).",
    collision_.sphereMode() ? "spheres" : "mesh", collision_.shapeCount(),
    collision_.sphereMode() ? "spheres" : "shapes", collision_.checkPairCount(),
    collision_.autoDisabledCount());
  {
    const auto & dp = collision_.autoDisabledPairs();
    for (size_t i = 0; i < dp.size() && i < 40; ++i) {
      RCLCPP_INFO(get_logger(), "    auto-disabled: %s", dp[i].c_str());
    }
  }
  if (collision_.visualFallbackCount() > 0) {
    RCLCPP_INFO(
      get_logger(), "%zu link(s) had no <collision> and use <visual> geometry instead.",
      collision_.visualFallbackCount());
  }
  if (collision_.meshTotal() > 0) {
    RCLCPP_INFO(
      get_logger(), "Meshes: %zu loaded, %zu failed, %zu collision triangles total.",
      collision_.meshTotal() - collision_.meshFailed(), collision_.meshFailed(),
      collision_.meshTriangles());
  }
  if (collision_.shapeCount() == 0) {
    RCLCPP_WARN(
      get_logger(),
      "No collision geometry at all - self-collision checking is INEFFECTIVE. "
      "Point 'robot_description_file' at a URDF that includes <collision>/<visual> "
      "meshes (e.g. your full robot_description), not a kinematics-only URDF.");
  }
  const auto & missing = collision_.linksWithoutCollision();
  if (!missing.empty()) {
    std::string list;
    for (const auto & l : missing) {list += l + " ";}
    RCLCPP_WARN(
      get_logger(),
      "%zu link(s) have NO collision geometry (self-collision NOT checked for "
      "them): %s", missing.size(), list.c_str());
  }

  // Per-joint velocity / acceleration limits (acceleration is rarely in the
  // URDF, so it falls back to the configured default).
  for (const auto & kv : urdf_model_.joints_) {
    double vmax = config_.defaults.joint_velocity;
    if (kv.second && kv.second->limits && kv.second->limits->velocity > 0.0) {
      vmax = kv.second->limits->velocity;
    }
    joint_limits_[kv.first] = {vmax, config_.defaults.joint_acceleration};

    // Position limits (for sampling-based planning). Continuous / unlimited
    // joints fall back to a bounded range so the sampler stays finite.
    double lo = -M_PI, hi = M_PI;
    if (kv.second && kv.second->limits && kv.second->type != urdf::Joint::CONTINUOUS) {
      lo = kv.second->limits->lower;
      hi = kv.second->limits->upper;
    }
    joint_pos_limits_[kv.first] = {lo, hi};
  }

  // Build kinematics + controller handles per group.
  for (auto & kv : config_.groups) {
    GroupConfig & g = kv.second;
    move_groups_[g.name] = std::make_shared<MoveGroup>(this, g);
    // Build a full-chain solver only for genuine single-chain Cartesian groups.
    // Constrained groups (TYPE_HOLD_TIP) borrow their compensating subgroup's
    // solver instead, and their own base->tip chain may contain joints they do
    // not own (e.g. a fixed waist_pitch), which the chain solver would reject.
    if (g.cartesian && g.compensating_subgroup.empty()) {
      auto kin = std::make_shared<GroupKinematics>();
      std::string kerr;
      if (!kin->init(kdl_tree_, g, urdf_model_, kerr)) {
        error = "kinematics init for group '" + g.name + "': " + kerr;
        return false;
      }
      kinematics_[g.name] = kin;
    }
  }
  return true;
}

bool ManipulationServer::initialize()
{
  std::string err;
  if (!loadConfig(
      get_parameter("move_groups_config").as_string(),
      get_parameter("named_poses_config").as_string(),
      get_parameter("collision_config").as_string(),
      get_parameter("sequences_config").as_string(),
      config_, err))
  {
    RCLCPP_ERROR(get_logger(), "Configuration error: %s", err.c_str());
    return false;
  }

  std::string urdf_xml;
  if (!loadUrdf(urdf_xml)) {
    RCLCPP_ERROR(get_logger(), "Could not obtain robot_description (URDF).");
    return false;
  }
  if (!buildModel(urdf_xml, err)) {
    RCLCPP_ERROR(get_logger(), "Model build error: %s", err.c_str());
    return false;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&ManipulationServer::onJointState, this, _1));

  move_server_ = rclcpp_action::create_server<Move>(
    this, "~/move",
    std::bind(&ManipulationServer::moveGoal, this, _1, _2),
    std::bind(&ManipulationServer::moveCancel, this, _1),
    std::bind(&ManipulationServer::moveAccepted, this, _1));

  seq_server_ = rclcpp_action::create_server<ExecuteSequence>(
    this, "~/execute_sequence",
    std::bind(&ManipulationServer::seqGoal, this, _1, _2),
    std::bind(&ManipulationServer::seqCancel, this, _1),
    std::bind(&ManipulationServer::seqAccepted, this, _1));

  collision_service_ = create_service<ManageCollisionObject>(
    "~/manage_collision_object",
    std::bind(&ManipulationServer::manageCollisionObject, this, _1, _2));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/collision_objects", rclcpp::QoS(1).transient_local());
  marker_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&ManipulationServer::publishMarkers, this));

  RCLCPP_INFO(
    get_logger(), "Bimanual manipulation server ready (%zu groups, %zu sequences).",
    config_.groups.size(), config_.sequences.size());
  return true;
}

// --- state -----------------------------------------------------------------

void ManipulationServer::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    joint_state_[msg->name[i]] = msg->position[i];
  }
}

std::map<std::string, double> ManipulationServer::currentState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_state_;
}

bool ManipulationServer::currentGroupValues(
  const GroupConfig & g, std::vector<double> & q, std::string & error) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  q.clear();
  for (const auto & jn : g.joints) {
    auto it = joint_state_.find(jn);
    if (it == joint_state_.end()) {
      error = "no joint state received yet for '" + jn + "'";
      return false;
    }
    q.push_back(it->second);
  }
  return true;
}

void ManipulationServer::makeLimits(const GroupConfig & g, Limits & limits) const
{
  limits.max_velocity.clear();
  limits.max_acceleration.clear();
  for (const auto & jn : g.joints) {
    auto it = joint_limits_.find(jn);
    if (it != joint_limits_.end()) {
      limits.max_velocity.push_back(it->second.first);
      limits.max_acceleration.push_back(it->second.second);
    } else {
      limits.max_velocity.push_back(config_.defaults.joint_velocity);
      limits.max_acceleration.push_back(config_.defaults.joint_acceleration);
    }
  }
}

// --- execution -------------------------------------------------------------

void ManipulationServer::stepScaling(
  const GroupConfig & g, const MotionStep & step, double & v, double & a) const
{
  v = step.velocity_scaling > 0 ? step.velocity_scaling : g.default_velocity_scaling;
  a = step.acceleration_scaling > 0 ? step.acceleration_scaling : g.default_acceleration_scaling;
}

bool ManipulationServer::computeStepPath(
  const GroupConfig & g, const MotionStep & step, const std::vector<double> & start,
  const std::map<std::string, double> & base_state, JointPath & path,
  std::string & error)
{
  const std::vector<std::string> joints = g.joints;
  const std::set<std::string> active(g.joints.begin(), g.joints.end());
  StateValidator valid = [this, joints, active, base_state](const std::vector<double> & q) {
      std::map<std::string, double> full = base_state;
      for (size_t i = 0; i < joints.size(); ++i) {full[joints[i]] = q[i];}
      return collision_.checkState(full, active);
    };

  MotionLimits motion;
  motion.joint_resolution = config_.collision.resolution;
  motion.cartesian_step = g.cartesian_step;

  // Straight line in joint space; if blocked and avoidance is on, route around
  // the obstacle with RRT-Connect and densify the resulting detour.
  auto jointWithFallback = [&](const std::vector<double> & target) -> bool {
      if (TrajectoryGenerator::jointPath(start, target, motion, valid, path, error)) {
        return true;
      }
      if (!config_.defaults.avoid_obstacles) {return false;}

      std::vector<std::pair<double, double>> bounds;
      bounds.reserve(g.joints.size());
      for (const auto & jn : g.joints) {
        auto it = joint_pos_limits_.find(jn);
        bounds.push_back(
          it != joint_pos_limits_.end() ? it->second : std::make_pair(-M_PI, M_PI));
      }

      RRTConnectOptions opt;
      opt.edge_resolution = config_.defaults.rrt_edge_resolution;  // coarse: fast search
      opt.max_iterations = config_.defaults.rrt_max_iterations;
      opt.step_size = config_.defaults.rrt_step;

      RCLCPP_INFO(
        get_logger(), "[%s] straight path blocked, planning around obstacles ...",
        g.name.c_str());
      JointPath sparse;
      std::string perr;
      if (!planRRTConnect(bounds, start, target, valid, opt, sparse, perr)) {
        error = "could not plan a collision-free path: " + perr;
        // Report exactly what collides so the cause is clear.
        if (perr.find("start") != std::string::npos) {
          logCollisions(g, start, "start configuration");
        } else if (perr.find("goal") != std::string::npos) {
          logCollisions(g, target, "goal configuration");
        }
        return false;
      }

      // Densify the sparse detour at the collision resolution for smooth timing,
      // re-validating each waypoint at the fine resolution (the search used a
      // coarser step for speed).
      path.clear();
      path.push_back(sparse.front());
      for (size_t s = 1; s < sparse.size(); ++s) {
        const auto & a = sparse[s - 1];
        const auto & b = sparse[s];
        double d = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {const double e = b[i] - a[i]; d += e * e;}
        d = std::sqrt(d);
        const int m = std::max(1, static_cast<int>(std::ceil(d / std::max(motion.joint_resolution, 1e-4))));
        for (int k = 1; k <= m; ++k) {
          const double f = static_cast<double>(k) / m;
          std::vector<double> q(a.size());
          for (size_t i = 0; i < a.size(); ++i) {q[i] = a[i] + f * (b[i] - a[i]);}
          if (!valid(q)) {
            error = "planned detour failed fine collision re-check";
            return false;
          }
          path.push_back(std::move(q));
        }
      }
      RCLCPP_INFO(get_logger(), "[%s] planned a detour (%zu waypoints)", g.name.c_str(), path.size());
      return true;
    };

  // Resolve a joint-space goal (named / joint / cartesian-as-joint-goto) or a
  // Cartesian straight line, all starting at `start`.
  if (step.type == MotionStep::TYPE_NAMED || step.type == MotionStep::TYPE_JOINT) {
    std::vector<double> target;
    if (step.type == MotionStep::TYPE_NAMED) {
      auto pit = config_.named_poses.find(g.name);
      if (pit == config_.named_poses.end() || !pit->second.count(step.named_target)) {
        error = "unknown named pose '" + step.named_target + "' for group '" + g.name + "'";
        return false;
      }
      target = pit->second.at(step.named_target);
    } else {
      target = step.joint_target;
    }
    if (target.size() != g.joints.size()) {
      error = "target has " + std::to_string(target.size()) + " values, group '" +
        g.name + "' expects " + std::to_string(g.joints.size());
      return false;
    }
    return jointWithFallback(target);
  }

  // Dual-arm coordinated Cartesian: apply ONE shared translation (step.offset)
  // to every subgroup tip simultaneously, so a two-handed grasp translates
  // rigidly (the relative pose between the hands is preserved). Orientation is
  // kept; both arms are sampled together so they finish in sync.
  if (step.type == MotionStep::TYPE_CARTESIAN && !g.cartesian_subgroups.empty()) {
    struct Sub
    {
      std::shared_ptr<GroupKinematics> kin;
      std::vector<int> idx;            // subgroup joint -> index in g.joints
      std::vector<double> seed;
      Eigen::Isometry3d tip0;
    };
    std::vector<Sub> subs;
    for (const auto & sname : g.cartesian_subgroups) {
      auto kit = kinematics_.find(sname);
      auto git = config_.groups.find(sname);
      if (kit == kinematics_.end() || git == config_.groups.end()) {
        error = "cartesian_subgroup '" + sname + "' is unknown or not Cartesian capable";
        return false;
      }
      Sub sub;
      sub.kin = kit->second;
      for (const auto & jn : git->second.joints) {
        auto it = std::find(g.joints.begin(), g.joints.end(), jn);
        if (it == g.joints.end()) {
          error = "subgroup joint '" + jn + "' is not a joint of group '" + g.name + "'";
          return false;
        }
        sub.idx.push_back(static_cast<int>(it - g.joints.begin()));
      }
      sub.seed.resize(sub.idx.size());
      for (size_t k = 0; k < sub.idx.size(); ++k) {sub.seed[k] = start[sub.idx[k]];}
      if (!sub.kin->fkTip(sub.seed, sub.tip0)) {
        error = "forward kinematics failed for subgroup '" + sname + "'";
        return false;
      }
      subs.push_back(std::move(sub));
    }

    // One offset per subgroup when step.offsets is given (move the hands
    // independently — e.g. spread), else the single `offset` for all (rigid).
    std::vector<Eigen::Vector3d> deltas;
    if (step.offsets.size() == subs.size()) {
      for (const auto & o : step.offsets) {deltas.emplace_back(o.x, o.y, o.z);}
    } else if (step.offsets.empty()) {
      deltas.assign(subs.size(), Eigen::Vector3d(step.offset.x, step.offset.y, step.offset.z));
    } else {
      error = "coordinated Cartesian: 'offsets' has " + std::to_string(step.offsets.size()) +
        " entries but the group has " + std::to_string(subs.size()) + " cartesian_subgroups";
      return false;
    }
    // Each offset is in that subgroup's base frame, or its tip frame when
    // offset_in_tip_frame is set (each hand moves along its own axes).
    double max_norm = 0.0;
    for (size_t si = 0; si < subs.size(); ++si) {
      if (step.offset_in_tip_frame) {deltas[si] = subs[si].tip0.linear() * deltas[si];}
      max_norm = std::max(max_norm, deltas[si].norm());
    }

    const int M = std::max(1, static_cast<int>(std::ceil(max_norm / std::max(g.cartesian_step, 1e-4))));
    path.clear();
    path.push_back(start);
    for (int s = 1; s <= M; ++s) {
      const double f = static_cast<double>(s) / M;
      std::vector<double> full = start;
      for (size_t si = 0; si < subs.size(); ++si) {
        Eigen::Isometry3d goal = subs[si].tip0;
        goal.translation() += deltas[si] * f;   // translation only, orientation kept
        std::vector<double> qsub;
        if (!subs[si].kin->ik(goal, subs[si].seed, qsub, /*limit_jump=*/true)) {
          error = "coordinated Cartesian: IK failed for subgroup '" +
            g.cartesian_subgroups[si] + "' at fraction " + std::to_string(f);
          return false;
        }
        for (size_t k = 0; k < subs[si].idx.size(); ++k) {full[subs[si].idx[k]] = qsub[k];}
        subs[si].seed = qsub;
      }
      if (!valid(full)) {
        error = "coordinated Cartesian: collision at fraction " + std::to_string(f);
        return false;
      }
      path.push_back(std::move(full));
    }
    return true;
  }

  // Drive the group's driven_joints (e.g. waist_yaw) while the compensating
  // subgroup (an arm) is IK-solved every step to hold the tip fixed in the
  // group's base frame — turn the waist while the hand stays on the object.
  if (step.type == MotionStep::TYPE_HOLD_TIP) {
    if (g.driven_joints.empty() || g.compensating_subgroup.empty()) {
      error = "group '" + g.name + "' has no driven_joints/compensating_subgroup";
      return false;
    }
    auto kit = kinematics_.find(g.compensating_subgroup);
    auto cit = config_.groups.find(g.compensating_subgroup);
    if (kit == kinematics_.end() || cit == config_.groups.end()) {
      error = "compensating_subgroup '" + g.compensating_subgroup +
        "' is unknown or not Cartesian capable";
      return false;
    }
    auto arm_kin = kit->second;
    const GroupConfig & arm = cit->second;
    const std::string & subbase = arm.base_link;

    // Index maps into the group's joint vector.
    auto indexIn = [&](const std::string & jn, int & out) -> bool {
        auto it = std::find(g.joints.begin(), g.joints.end(), jn);
        if (it == g.joints.end()) {return false;}
        out = static_cast<int>(it - g.joints.begin());
        return true;
      };
    std::vector<int> driven_idx(g.driven_joints.size());
    for (size_t i = 0; i < g.driven_joints.size(); ++i) {
      if (!indexIn(g.driven_joints[i], driven_idx[i])) {
        error = "driven joint '" + g.driven_joints[i] + "' is not in group '" + g.name + "'";
        return false;
      }
    }
    std::vector<int> arm_idx(arm.joints.size());
    for (size_t i = 0; i < arm.joints.size(); ++i) {
      if (!indexIn(arm.joints[i], arm_idx[i])) {
        error = "compensating joint '" + arm.joints[i] + "' is not in group '" + g.name + "'";
        return false;
      }
    }
    if (step.joint_target.size() != g.driven_joints.size()) {
      error = "hold_tip expects " + std::to_string(g.driven_joints.size()) +
        " driven delta(s), got " + std::to_string(step.joint_target.size());
      return false;
    }

    // Held target: current tip pose in the group's base frame.
    std::map<std::string, double> jv = base_state;
    for (size_t i = 0; i < g.joints.size(); ++i) {jv[g.joints[i]] = start[i];}
    Eigen::Isometry3d base0, tip0;
    if (!collision_.linkPose(g.base_link, jv, base0) ||
      !collision_.linkPose(g.tip_link, jv, tip0))
    {
      error = "hold_tip: unknown base_link/tip_link for group '" + g.name + "'";
      return false;
    }
    const Eigen::Isometry3d tip_in_base = base0.inverse() * tip0;

    std::vector<double> arm_seed(arm.joints.size());
    for (size_t i = 0; i < arm.joints.size(); ++i) {arm_seed[i] = start[arm_idx[i]];}

    // Sanity: the compensating arm must be able to reproduce the CURRENT tip
    // pose from its current joints. If not, the frames are inconsistent (wrong
    // base_link/tip_link, or base_link not downstream of the driven joints).
    {
      Eigen::Isometry3d sub0;
      collision_.linkPose(subbase, jv, sub0);
      const Eigen::Isometry3d goal0 = (base0.inverse() * sub0).inverse() * tip_in_base;
      Eigen::Isometry3d cur0;
      arm_kin->fkTip(arm_seed, cur0);
      const double d0 = (goal0.translation() - cur0.translation()).norm();
      std::vector<double> q0;
      if (!arm_kin->ik(goal0, arm_seed, q0, /*limit_jump=*/true, step.free_orientation)) {
        error = "hold_tip: the compensating subgroup '" + g.compensating_subgroup +
          "' cannot reproduce the current tip pose (FK residual " + std::to_string(d0) +
          " m). Check that its base_link '" + subbase + "' is downstream of the driven "
          "joints and its tip_link matches '" + g.tip_link + "'.";
        return false;
      }
    }

    // Effective per-driven-joint delta from current to the requested value
    // (absolute target -> target - current; else the given relative delta).
    std::vector<double> ddelta(g.driven_joints.size());
    double max_delta = 0.0;
    for (size_t i = 0; i < g.driven_joints.size(); ++i) {
      ddelta[i] = step.driven_absolute ?
        (step.joint_target[i] - start[driven_idx[i]]) : step.joint_target[i];
      max_delta = std::max(max_delta, std::abs(ddelta[i]));
    }
    const int M = std::max(1, static_cast<int>(std::ceil(max_delta / 0.02)));

    path.clear();
    path.push_back(start);
    for (int s = 1; s <= M; ++s) {
      const double f = static_cast<double>(s) / M;
      std::vector<double> full = start;
      // Ramp every driven joint from its current value toward the target.
      std::map<std::string, double> jvs = base_state;
      for (size_t i = 0; i < g.joints.size(); ++i) {jvs[g.joints[i]] = full[i];}
      for (size_t i = 0; i < g.driven_joints.size(); ++i) {
        const double v = start[driven_idx[i]] + ddelta[i] * f;
        full[driven_idx[i]] = v;
        jvs[g.driven_joints[i]] = v;
      }
      // Tip pose re-expressed in the (now-moved) arm base frame.
      Eigen::Isometry3d baseP, subP;
      if (!collision_.linkPose(g.base_link, jvs, baseP) ||
        !collision_.linkPose(subbase, jvs, subP))
      {
        error = "hold_tip: FK failed";
        return false;
      }
      const Eigen::Isometry3d goal_in_sub = (baseP.inverse() * subP).inverse() * tip_in_base;
      std::vector<double> arm_q;
      if (!arm_kin->ik(goal_in_sub, arm_seed, arm_q, /*limit_jump=*/true, step.free_orientation)) {
        Eigen::Isometry3d cur;
        arm_kin->fkTip(arm_seed, cur);
        const double dp = (goal_in_sub.translation() - cur.translation()).norm();
        error = "hold_tip: arm IK failed at driven delta " + std::to_string(max_delta * f) +
          " rad; the arm would need to move the tip " + std::to_string(dp) +
          " m in its base frame (large => frame/config mismatch; small => joint "
          "limit or singularity)";
        return false;
      }
      for (size_t i = 0; i < arm.joints.size(); ++i) {full[arm_idx[i]] = arm_q[i];}
      if (!valid(full)) {
        error = "hold_tip: collision at driven delta " + std::to_string(max_delta * f) + " rad";
        return false;
      }
      arm_seed = arm_q;
      path.push_back(std::move(full));
    }
    return true;
  }

  if (step.type == MotionStep::TYPE_CARTESIAN) {
    if (!g.cartesian || !kinematics_.count(g.name)) {
      error = "group '" + g.name + "' is not Cartesian capable";
      return false;
    }
    auto kin = kinematics_.at(g.name);
    Eigen::Isometry3d start_pose;
    if (!kin->fkTip(start, start_pose)) {
      error = "forward kinematics failed for group '" + g.name + "'";
      return false;
    }

    Eigen::Isometry3d goal_pose;
    if (!step.reference_frame.empty()) {
      geometry_msgs::msg::TransformStamped tfm;
      try {
        tfm = tf_buffer_->lookupTransform(
          g.base_link, step.reference_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException & ex) {
        error = "TF lookup '" + g.base_link + "' <- '" + step.reference_frame + "' failed: " +
          ex.what();
        return false;
      }
      goal_pose = tf2::transformToEigen(tfm) * poseMsgToEigen(step.pose_target);
    } else if (step.relative) {
      goal_pose = start_pose;
      Eigen::Vector3d off(step.offset.x, step.offset.y, step.offset.z);
      goal_pose.translation() += step.offset_in_tip_frame ? (start_pose.linear() * off) : off;
    } else {
      goal_pose = poseMsgToEigen(step.pose_target);
    }

    if (step.cartesian_path) {
      return TrajectoryGenerator::cartesianPath(
        *kin, start, start_pose, goal_pose, motion, valid, path, error,
        step.free_orientation);
    }
    // Collision-aware IK: among the (possibly redundant) solutions, pick one
    // that is not in self/world collision — important for high-DoF groups where
    // KDL might otherwise fold the arm into the body.
    std::vector<double> target;
    if (!kin->ik(goal_pose, start, target, /*limit_jump=*/false, step.free_orientation, valid)) {
      error = "IK failed for the requested pose target (group '" + g.name +
        "'): no collision-free solution found";
      return false;
    }
    return jointWithFallback(target);
  }

  error = "step type " + std::to_string(step.type) + " is not a motion step";
  return false;
}

bool ManipulationServer::runTrajectory(
  const GroupConfig & g, const JointPath & path, double vscale,
  double ascale, std::string & error)
{
  if (path.size() < 2) {return true;}  // nothing to move
  Limits limits;
  makeLimits(g, limits);
  MotionLimits motion;
  motion.velocity_scaling = vscale;
  motion.acceleration_scaling = ascale;
  motion.joint_resolution = config_.collision.resolution;
  motion.cartesian_step = g.cartesian_step;
  motion.limit_acceleration = config_.defaults.acceleration_limiting;

  trajectory_msgs::msg::JointTrajectory traj;
  TrajectoryGenerator::toTrajectory(g.joints, path, limits, motion, traj);

  // Wait for the trajectory's own duration plus a margin, instead of a fixed
  // timeout that would wrongly abort a legitimately long motion.
  double duration = 0.0;
  if (!traj.points.empty()) {
    const auto & tfs = traj.points.back().time_from_start;
    duration = tfs.sec + tfs.nanosec * 1e-9;
  }
  const double timeout = duration + config_.defaults.execution_timeout;

  RCLCPP_INFO(
    get_logger(), "[%s] executing %zu waypoints, planned duration %.1f s ...",
    g.name.c_str(), traj.points.size(), duration);
  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = move_groups_.at(g.name)->execute(traj, timeout, error);
  RCLCPP_INFO(
    get_logger(), "[%s] execution finished in %.0f ms (%s)", g.name.c_str(),
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(),
    ok ? "ok" : "failed");
  return ok;
}

bool ManipulationServer::executeGripperStep(
  const GroupConfig & g, const MotionStep & step, std::string & error)
{
  double position = step.gripper_position;
  if (!step.named_target.empty()) {
    auto pit = config_.named_poses.find(g.name);
    if (pit == config_.named_poses.end() || !pit->second.count(step.named_target) ||
      pit->second.at(step.named_target).empty())
    {
      error = "unknown gripper pose '" + step.named_target + "' for '" + g.name + "'";
      return false;
    }
    position = pit->second.at(step.named_target).front();
  }
  return move_groups_.at(g.name)->executeGripper(
    position, get_parameter("gripper_max_effort").as_double(),
    config_.defaults.execution_timeout, error);
}

void ManipulationServer::logCollisions(
  const GroupConfig & g, const std::vector<double> & q, const char * label)
{
  std::map<std::string, double> full = currentState();
  for (size_t i = 0; i < g.joints.size() && i < q.size(); ++i) {full[g.joints[i]] = q[i];}
  const std::set<std::string> active(g.joints.begin(), g.joints.end());
  const auto desc = collision_.describeCollisions(full, &active);
  if (desc.empty()) {
    RCLCPP_WARN(
      get_logger(), "[%s] %s reported in collision but no pair found (numerical edge case)",
      g.name.c_str(), label);
    return;
  }
  RCLCPP_WARN(get_logger(), "[%s] %s is in collision — %zu pair(s):", g.name.c_str(), label,
    desc.size());
  for (const auto & s : desc) {RCLCPP_WARN(get_logger(), "    - %s", s.c_str());}
}

bool ManipulationServer::executeFollow(
  const GroupConfig & g, const MotionStep & step, std::string & error)
{
  if (!g.cartesian || !kinematics_.count(g.name)) {
    error = "follow needs a Cartesian-capable group (base_link + tip_link)";
    return false;
  }
  if (step.reference_frame.empty()) {
    error = "follow requires a reference_frame";
    return false;
  }
  auto kin = kinematics_.at(g.name);
  const double rate = step.follow_rate > 0 ? step.follow_rate : 20.0;
  const auto period = std::chrono::duration<double>(1.0 / rate);
  const double horizon = std::max(2.0 / rate, 0.1);
  const Eigen::Isometry3d delta = poseMsgToEigen(step.pose_target);
  const std::set<std::string> active(g.joints.begin(), g.joints.end());

  // Per-cycle motion is velocity-limited so the arm approaches a far target at
  // a controlled speed and then tracks it once close.
  Limits limits;
  makeLimits(g, limits);
  double vscale, ascale;
  stepScaling(g, step, vscale, ascale);

  // Optional external stop signal (lets a sequence end the follow and proceed).
  std::atomic<bool> stop_flag{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
  if (!step.follow_stop_topic.empty()) {
    stop_sub = create_subscription<std_msgs::msg::Bool>(
      step.follow_stop_topic, 10,
      [&stop_flag](std_msgs::msg::Bool::SharedPtr m) {if (m->data) {stop_flag.store(true);}});
  }

  RCLCPP_INFO(
    get_logger(), "[%s] following frame '%s' ...", g.name.c_str(),
    step.reference_frame.c_str());

  const rclcpp::Time t_start = now();
  rclcpp::Time settled_since;
  bool settling = false;
  std::string ignore;

  while (rclcpp::ok() && !cancel_requested_.load() && !stop_flag.load()) {
    if (step.follow_timeout > 0 && (now() - t_start).seconds() >= step.follow_timeout) {break;}

    std::vector<double> q;
    if (!currentGroupValues(g, q, ignore)) {std::this_thread::sleep_for(period); continue;}

    Eigen::Isometry3d goal;
    try {
      const auto tfm = tf_buffer_->lookupTransform(
        g.base_link, step.reference_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
      goal = tf2::transformToEigen(tfm) * delta;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "[%s] waiting for frame '%s': %s",
        g.name.c_str(), step.reference_frame.c_str(), ex.what());
      std::this_thread::sleep_for(period);
      continue;
    }

    std::vector<double> target;
    if (!kin->ik(goal, q, target, /*limit_jump=*/false)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "[%s] follow target unreachable", g.name.c_str());
      std::this_thread::sleep_for(period);
      continue;
    }

    // Step toward the IK target, capped by the joint velocity limits over the
    // command horizon (so a far target is approached smoothly, not teleported).
    double factor = 1.0;
    for (size_t i = 0; i < g.joints.size(); ++i) {
      const double dq = std::abs(target[i] - q[i]);
      const double allowed = std::max(limits.max_velocity[i] * vscale * horizon, 1e-6);
      if (dq > allowed) {factor = std::min(factor, allowed / dq);}
    }
    std::vector<double> cmd(g.joints.size());
    for (size_t i = 0; i < g.joints.size(); ++i) {cmd[i] = q[i] + factor * (target[i] - q[i]);}

    std::map<std::string, double> full = currentState();
    for (size_t i = 0; i < g.joints.size(); ++i) {full[g.joints[i]] = cmd[i];}
    if (!collision_.checkState(full, active)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "[%s] follow target in collision, holding",
        g.name.c_str());
      std::this_thread::sleep_for(period);
      continue;
    }

    // Settled? (tip close enough to the target for long enough -> stop)
    if (step.follow_position_tolerance > 0) {
      Eigen::Isometry3d tip;
      kin->fkTip(q, tip);
      const double derr = (tip.translation() - goal.translation()).norm();
      if (derr < step.follow_position_tolerance) {
        if (!settling) {settling = true; settled_since = now();}
        else if ((now() - settled_since).seconds() >= step.follow_settle_time) {break;}
      } else {
        settling = false;
      }
    }

    // Stream a short trajectory toward the target (controller interpolates from
    // the current state; preempts the previous goal).
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = g.joints;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = cmd;
    pt.time_from_start = rclcpp::Duration::from_seconds(horizon);
    traj.points.push_back(pt);
    move_groups_.at(g.name)->sendTrajectoryNoWait(traj, ignore);

    std::this_thread::sleep_for(period);
  }

  RCLCPP_INFO(get_logger(), "[%s] follow finished", g.name.c_str());
  return true;
}

bool ManipulationServer::executeCollisionStep(const MotionStep & step, std::string & error)
{
  const std::string & op = step.collision_op;
  const Eigen::Isometry3d pose = poseMsgToEigen(step.pose_target);
  bool ok = true;
  if (op == "add") {
    ok = collision_.addObject(step.collision_id, step.collision_primitive, pose, error);
  } else if (op == "attach") {
    ok = collision_.addAttachedObject(
      step.collision_id, step.collision_primitive, step.collision_attach_link, pose, error);
  } else if (op == "remove" || op == "detach") {
    ok = collision_.removeObject(step.collision_id);
    if (!ok) {error = "collision object '" + step.collision_id + "' not found";}
  } else if (op == "clear") {
    collision_.clearObjects();
  } else if (op == "enable") {
    collision_.setEnabled(true);
  } else if (op == "disable") {
    collision_.setEnabled(false);
  } else {
    error = "unknown collision_op '" + op + "' (add/remove/attach/detach/clear/enable/disable)";
    return false;
  }
  if (ok) {
    if (step.collision_id.empty()) {
      RCLCPP_INFO(get_logger(), "collision: %s", op.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "collision: %s '%s'", op.c_str(), step.collision_id.c_str());
    }
  }
  return ok;
}

bool ManipulationServer::executeStep(const MotionStep & step, std::string & error)
{
  if (step.type == MotionStep::TYPE_COLLISION) {
    return executeCollisionStep(step, error);   // no group / motion
  }

  auto git = config_.groups.find(step.group);
  if (git == config_.groups.end()) {
    error = "unknown move group '" + step.group + "'";
    return false;
  }
  const GroupConfig & g = git->second;

  if (step.type == MotionStep::TYPE_GRIPPER) {
    return executeGripperStep(g, step, error);
  }
  if (step.type == MotionStep::TYPE_FOLLOW) {
    return executeFollow(g, step, error);
  }

  std::vector<double> start;
  if (!currentGroupValues(g, start, error)) {return false;}
  JointPath path;
  const auto t0 = std::chrono::steady_clock::now();
  if (!computeStepPath(g, step, start, currentState(), path, error)) {return false;}
  RCLCPP_INFO(
    get_logger(), "[%s] trajectory generated in %.0f ms (%zu waypoints), executing...",
    g.name.c_str(),
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(),
    path.size());

  double vscale, ascale;
  stepScaling(g, step, vscale, ascale);
  return runTrajectory(g, path, vscale, ascale, error);
}

// --- Move action -----------------------------------------------------------

rclcpp_action::GoalResponse ManipulationServer::moveGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal>)
{
  if (busy_.load()) {
    RCLCPP_WARN(get_logger(), "Rejecting Move goal: server busy.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationServer::moveCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationServer::moveAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> gh)
{
  std::thread(
    [this, gh]() {
      busy_.store(true);
      cancel_requested_.store(false);
      auto result = std::make_shared<Move::Result>();
      std::string err;
      const bool ok = executeStep(gh->get_goal()->step, err);
      result->success = ok;
      result->message = ok ? "completed" : err;
      if (ok) {
        gh->succeed(result);
      } else {
        RCLCPP_ERROR(get_logger(), "Move failed: %s", err.c_str());
        gh->abort(result);
      }
      busy_.store(false);
    }).detach();
}

// --- ExecuteSequence action ------------------------------------------------

rclcpp_action::GoalResponse ManipulationServer::seqGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteSequence::Goal>)
{
  if (busy_.load()) {
    RCLCPP_WARN(get_logger(), "Rejecting ExecuteSequence goal: server busy.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationServer::seqCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>>)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationServer::seqAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> gh)
{
  std::thread(
    [this, gh]() {
      busy_.store(true);
      cancel_requested_.store(false);
      const auto goal = gh->get_goal();

      // Resolve the full step list: named sequence first, then inline steps.
      std::vector<MotionStep> steps;
      if (!goal->sequence_name.empty()) {
        auto sit = config_.sequences.find(goal->sequence_name);
        if (sit == config_.sequences.end()) {
          auto result = std::make_shared<ExecuteSequence::Result>();
          result->success = false;
          result->message = "unknown sequence '" + goal->sequence_name + "'";
          gh->abort(result);
          busy_.store(false);
          return;
        }
        steps = sit->second;
      }
      steps.insert(steps.end(), goal->steps.begin(), goal->steps.end());

      auto result = std::make_shared<ExecuteSequence::Result>();
      auto feedback = std::make_shared<ExecuteSequence::Feedback>();
      feedback->total_steps = static_cast<int>(steps.size());
      result->completed_steps = 0;
      bool ok = true;
      std::string err;

      // Consecutive motion steps on the same group are concatenated into one
      // continuous trajectory (no stop between them) and only blended/executed
      // at a barrier: a gripper step, a group change, or the end. This is what
      // gives fluid, human-like chaining.
      const GroupConfig * batch_g = nullptr;
      JointPath batch_path;
      std::vector<size_t> batch_junctions;
      std::vector<double> batch_radii;
      std::vector<double> batch_start_q;
      std::map<std::string, double> batch_base_state;
      double batch_vmin = 1.0, batch_amin = 1.0, prev_blend = 0.0;
      int batch_count = 0;

      auto flush = [&](std::string & e) -> bool {
          if (!batch_g || batch_path.size() < 2) {
            batch_g = nullptr; batch_path.clear(); batch_junctions.clear();
            batch_radii.clear(); batch_count = 0;
            return true;
          }
          const std::vector<std::string> jn = batch_g->joints;
          const std::set<std::string> active(jn.begin(), jn.end());
          const auto base = batch_base_state;
          StateValidator valid = [this, jn, active, base](const std::vector<double> & q) {
              std::map<std::string, double> full = base;
              for (size_t i = 0; i < jn.size(); ++i) {full[jn[i]] = q[i];}
              return collision_.checkState(full, active);
            };
          const size_t nb = TrajectoryGenerator::blendJunctions(
            batch_path, batch_junctions, batch_radii, valid);
          RCLCPP_INFO(
            get_logger(), "[%s] %zu step(s), %zu/%zu junction(s) blended",
            batch_g->name.c_str(), static_cast<size_t>(batch_count), nb, batch_junctions.size());
          const bool r = runTrajectory(*batch_g, batch_path, batch_vmin, batch_amin, e);
          if (r) {result->completed_steps += batch_count;}
          batch_g = nullptr; batch_path.clear(); batch_junctions.clear();
          batch_radii.clear(); batch_count = 0;
          return r;
        };

      for (size_t i = 0; i < steps.size() && ok; ++i) {
        if (cancel_requested_.load()) {
          flush(err);  // finish what is already planned
          result->success = false;
          result->message = "canceled";
          gh->canceled(result);
          busy_.store(false);
          return;
        }
        const MotionStep & step = steps[i];
        feedback->current_step = static_cast<int>(i);
        feedback->current_action = step.group;
        gh->publish_feedback(feedback);

        // Group-less barrier steps (collision management): flush, then run.
        if (step.type == MotionStep::TYPE_COLLISION) {
          if (!flush(err)) {ok = false; break;}
          if (!executeStep(step, err)) {ok = false; break;}
          result->completed_steps++;
          continue;
        }

        auto git = config_.groups.find(step.group);
        if (git == config_.groups.end()) {
          flush(err);
          err = "unknown move group '" + step.group + "'";
          ok = false;
          break;
        }
        const GroupConfig & g = git->second;

        // Only joint-space / Cartesian motions are concatenated; gripper and
        // follow steps are barriers run on their own (and a group change also
        // closes the current batch).
        const bool batchable = step.type == MotionStep::TYPE_NAMED ||
          step.type == MotionStep::TYPE_JOINT || step.type == MotionStep::TYPE_CARTESIAN;
        if (!batchable || (batch_g && batch_g != &g)) {
          if (!flush(err)) {ok = false; break;}
        }

        if (!batchable) {
          if (!executeStep(step, err)) {ok = false; break;}
          result->completed_steps++;
          continue;
        }

        // Open a new batch if needed (capture the start configuration once).
        if (!batch_g) {
          batch_g = &g;
          batch_base_state = currentState();
          if (!currentGroupValues(g, batch_start_q, err)) {ok = false; break;}
          batch_vmin = 1.0;
          batch_amin = 1.0;
        }

        JointPath seg;
        if (!computeStepPath(g, step, batch_start_q, batch_base_state, seg, err)) {
          RCLCPP_ERROR(get_logger(), "Sequence step %zu failed: %s", i, err.c_str());
          flush(err);  // execute the valid prefix
          ok = false;
          break;
        }

        if (batch_path.empty()) {
          batch_path = seg;
        } else if (seg.size() >= 2) {
          batch_junctions.push_back(batch_path.size() - 1);
          batch_radii.push_back(prev_blend);
          batch_path.insert(batch_path.end(), seg.begin() + 1, seg.end());
        }
        batch_start_q = batch_path.back();
        prev_blend = step.blend_radius;
        double v, a;
        stepScaling(g, step, v, a);
        batch_vmin = std::min(batch_vmin, v);
        batch_amin = std::min(batch_amin, a);
        ++batch_count;
      }

      if (ok) {
        if (!flush(err)) {ok = false;}
      }

      result->success = ok;
      result->message = ok ? "sequence completed" : ("failed: " + err);
      if (ok) {
        gh->succeed(result);
      } else {
        gh->abort(result);
      }
      busy_.store(false);
    }).detach();
}

// --- collision object service ----------------------------------------------

void ManipulationServer::manageCollisionObject(
  const std::shared_ptr<ManageCollisionObject::Request> req,
  std::shared_ptr<ManageCollisionObject::Response> res)
{
  std::string err;
  const Eigen::Isometry3d pose = poseMsgToEigen(req->pose.pose);
  switch (req->operation) {
    case ManageCollisionObject::Request::ADD:
      res->success = collision_.addObject(req->id, req->primitive, pose, err);
      break;
    case ManageCollisionObject::Request::ATTACH:
      res->success = collision_.addAttachedObject(
        req->id, req->primitive, req->attach_link, pose, err);
      break;
    case ManageCollisionObject::Request::REMOVE:
    case ManageCollisionObject::Request::DETACH:
      res->success = collision_.removeObject(req->id);
      if (!res->success) {err = "object '" + req->id + "' not found";}
      break;
    case ManageCollisionObject::Request::CLEAR:
      collision_.clearObjects();
      res->success = true;
      break;
    default:
      res->success = false;
      err = "unknown operation";
      break;
  }
  res->message = res->success ? "ok" : err;
}

// --- visualization ---------------------------------------------------------

void ManipulationServer::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  using SP = shape_msgs::msg::SolidPrimitive;

  const auto objects = collision_.objects();
  visualization_msgs::msg::MarkerArray arr;

  Marker del;
  del.action = Marker::DELETEALL;
  arr.markers.push_back(del);

  int id = 0;
  for (const auto & o : objects) {
    Marker m;
    m.header.frame_id = o.attached_link.empty() ? collision_.rootFrame() : o.attached_link;
    m.header.stamp = now();
    m.ns = "collision_objects";
    m.id = id++;
    m.action = Marker::ADD;

    const Eigen::Quaterniond q(o.pose.linear());
    m.pose.position.x = o.pose.translation().x();
    m.pose.position.y = o.pose.translation().y();
    m.pose.position.z = o.pose.translation().z();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    const auto & d = o.primitive.dimensions;
    switch (o.primitive.type) {
      case SP::BOX:
        if (d.size() >= 3) {
          m.type = Marker::CUBE;
          m.scale.x = d[SP::BOX_X];
          m.scale.y = d[SP::BOX_Y];
          m.scale.z = d[SP::BOX_Z];
        }
        break;
      case SP::SPHERE:
        if (!d.empty()) {
          m.type = Marker::SPHERE;
          m.scale.x = m.scale.y = m.scale.z = 2.0 * d[SP::SPHERE_RADIUS];
        }
        break;
      case SP::CYLINDER:
        if (d.size() >= 2) {
          m.type = Marker::CYLINDER;
          m.scale.x = m.scale.y = 2.0 * d[SP::CYLINDER_RADIUS];
          m.scale.z = d[SP::CYLINDER_HEIGHT];
        }
        break;
      default:
        continue;
    }

    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.0f;
    m.color.a = 0.6f;
    arr.markers.push_back(m);
  }

  marker_pub_->publish(arr);
}

}  // namespace bimanual_manipulation
