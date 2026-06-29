#include "bimanual_manipulation/manipulation_server.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <thread>

#include <kdl_parser/kdl_parser.hpp>
#include <srdfdom/model.h>

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
    get_logger(), "Collision model: %zu shapes, %zu link pairs checked.",
    collision_.shapeCount(), collision_.checkPairCount());
  if (collision_.visualFallbackCount() > 0) {
    RCLCPP_INFO(
      get_logger(), "%zu link(s) had no <collision> and use <visual> geometry instead.",
      collision_.visualFallbackCount());
  }
  if (collision_.meshTotal() > 0) {
    RCLCPP_INFO(
      get_logger(), "Meshes: %zu loaded, %zu failed.",
      collision_.meshTotal() - collision_.meshFailed(), collision_.meshFailed());
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
  }

  // Build kinematics + controller handles per group.
  for (auto & kv : config_.groups) {
    GroupConfig & g = kv.second;
    move_groups_[g.name] = std::make_shared<MoveGroup>(this, g);
    if (g.cartesian) {
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

bool ManipulationServer::executeNamedOrJoint(
  const GroupConfig & g, const std::vector<double> & target, double vscale,
  double ascale, std::string & error)
{
  std::vector<double> start;
  if (!currentGroupValues(g, start, error)) {return false;}
  if (target.size() != g.joints.size()) {
    error = "target has " + std::to_string(target.size()) + " values, group '" +
      g.name + "' expects " + std::to_string(g.joints.size());
    return false;
  }

  Limits limits;
  makeLimits(g, limits);
  MotionLimits motion;
  motion.velocity_scaling = vscale;
  motion.acceleration_scaling = ascale;
  motion.joint_resolution = config_.collision.resolution;
  motion.cartesian_step = g.cartesian_step;

  const auto joints = g.joints;
  const std::set<std::string> active(g.joints.begin(), g.joints.end());
  auto base_state = currentState();
  StateValidator valid = [this, joints, active, base_state](const std::vector<double> & q) {
      std::map<std::string, double> full = base_state;
      for (size_t i = 0; i < joints.size(); ++i) {full[joints[i]] = q[i];}
      return collision_.checkState(full, active);
    };

  trajectory_msgs::msg::JointTrajectory traj;
  if (!TrajectoryGenerator::planJoint(g.joints, start, target, limits, motion, valid, traj, error)) {
    return false;
  }
  return move_groups_.at(g.name)->execute(traj, config_.defaults.execution_timeout, error);
}

bool ManipulationServer::executeCartesian(
  const GroupConfig & g, const MotionStep & step, std::string & error)
{
  if (!g.cartesian || !kinematics_.count(g.name)) {
    error = "group '" + g.name + "' is not Cartesian capable";
    return false;
  }
  auto kin = kinematics_.at(g.name);

  std::vector<double> start;
  if (!currentGroupValues(g, start, error)) {return false;}

  Eigen::Isometry3d start_pose;
  if (!kin->fkTip(start, start_pose)) {
    error = "forward kinematics failed for group '" + g.name + "'";
    return false;
  }

  Eigen::Isometry3d goal_pose;
  if (step.relative) {
    goal_pose = start_pose;
    Eigen::Vector3d off(step.offset.x, step.offset.y, step.offset.z);
    goal_pose.translation() += step.offset_in_tip_frame ? (start_pose.linear() * off) : off;
  } else {
    goal_pose = poseMsgToEigen(step.pose_target);
  }

  Limits limits;
  makeLimits(g, limits);
  MotionLimits motion;
  motion.velocity_scaling =
    step.velocity_scaling > 0 ? step.velocity_scaling : g.default_velocity_scaling;
  motion.acceleration_scaling =
    step.acceleration_scaling > 0 ? step.acceleration_scaling : g.default_acceleration_scaling;
  motion.cartesian_step = g.cartesian_step;
  motion.joint_resolution = config_.collision.resolution;

  const auto joints = g.joints;
  const std::set<std::string> active(g.joints.begin(), g.joints.end());
  auto base_state = currentState();
  StateValidator valid = [this, joints, active, base_state](const std::vector<double> & q) {
      std::map<std::string, double> full = base_state;
      for (size_t i = 0; i < joints.size(); ++i) {full[joints[i]] = q[i];}
      return collision_.checkState(full, active);
    };

  trajectory_msgs::msg::JointTrajectory traj;
  if (!TrajectoryGenerator::planCartesian(
      g.joints, *kin, start, start_pose, goal_pose, limits, motion, valid, traj, error))
  {
    return false;
  }
  return move_groups_.at(g.name)->execute(traj, config_.defaults.execution_timeout, error);
}

bool ManipulationServer::executeStep(const MotionStep & step, std::string & error)
{
  auto git = config_.groups.find(step.group);
  if (git == config_.groups.end()) {
    error = "unknown move group '" + step.group + "'";
    return false;
  }
  const GroupConfig & g = git->second;
  const double vscale =
    step.velocity_scaling > 0 ? step.velocity_scaling : g.default_velocity_scaling;
  const double ascale =
    step.acceleration_scaling > 0 ? step.acceleration_scaling : g.default_acceleration_scaling;

  switch (step.type) {
    case MotionStep::TYPE_NAMED: {
      auto pit = config_.named_poses.find(g.name);
      if (pit == config_.named_poses.end() || !pit->second.count(step.named_target)) {
        error = "unknown named pose '" + step.named_target + "' for group '" + g.name + "'";
        return false;
      }
      return executeNamedOrJoint(g, pit->second.at(step.named_target), vscale, ascale, error);
    }
    case MotionStep::TYPE_JOINT:
      return executeNamedOrJoint(g, step.joint_target, vscale, ascale, error);
    case MotionStep::TYPE_CARTESIAN:
      return executeCartesian(g, step, error);
    case MotionStep::TYPE_GRIPPER: {
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
    default:
      error = "unknown step type " + std::to_string(step.type);
      return false;
  }
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

      for (size_t i = 0; i < steps.size(); ++i) {
        if (cancel_requested_.load()) {
          result->success = false;
          result->message = "canceled";
          gh->canceled(result);
          busy_.store(false);
          return;
        }
        feedback->current_step = static_cast<int>(i);
        feedback->current_action = steps[i].group;
        gh->publish_feedback(feedback);

        if (!executeStep(steps[i], err)) {
          ok = false;
          RCLCPP_ERROR(get_logger(), "Sequence step %zu failed: %s", i, err.c_str());
          if (goal->stop_on_failure) {break;}
        } else {
          result->completed_steps++;
        }
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
