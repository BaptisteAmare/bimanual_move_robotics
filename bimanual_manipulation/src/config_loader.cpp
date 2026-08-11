#include "bimanual_manipulation/config_loader.hpp"

#include <yaml-cpp/yaml.h>

namespace bimanual_manipulation
{

namespace
{
using MotionStep = bimanual_msgs::msg::MotionStep;

std::vector<std::string> asStringList(const YAML::Node & n)
{
  std::vector<std::string> v;
  if (n && n.IsSequence()) {
    for (const auto & e : n) {v.push_back(e.as<std::string>());}
  }
  return v;
}

std::vector<double> asDoubleList(const YAML::Node & n)
{
  std::vector<double> v;
  if (n && n.IsSequence()) {
    for (const auto & e : n) {v.push_back(e.as<double>());}
  }
  return v;
}

bool parseGroups(const YAML::Node & root, ManipulationConfig & out, std::string & error)
{
  const YAML::Node planning = root["planning"];
  if (planning) {
    auto & d = out.defaults;
    d.velocity_scaling = planning["default_velocity_scaling"].as<double>(d.velocity_scaling);
    d.acceleration_scaling =
      planning["default_acceleration_scaling"].as<double>(d.acceleration_scaling);
    d.joint_velocity = planning["default_joint_velocity"].as<double>(d.joint_velocity);
    d.joint_acceleration =
      planning["default_joint_acceleration"].as<double>(d.joint_acceleration);
    d.execution_timeout = planning["execution_timeout"].as<double>(d.execution_timeout);
    d.acceleration_limiting =
      planning["acceleration_limiting"].as<bool>(d.acceleration_limiting);
    d.avoid_obstacles = planning["avoid_obstacles"].as<bool>(d.avoid_obstacles);
    d.rrt_max_iterations = planning["rrt_max_iterations"].as<int>(d.rrt_max_iterations);
    d.rrt_step = planning["rrt_step"].as<double>(d.rrt_step);
    d.rrt_edge_resolution =
      planning["rrt_edge_resolution"].as<double>(d.rrt_edge_resolution);
  }

  const YAML::Node groups = root["groups"];
  if (!groups || !groups.IsMap()) {
    error = "move_groups.yaml: missing 'groups' map";
    return false;
  }

  for (const auto & it : groups) {
    GroupConfig g;
    g.name = it.first.as<std::string>();
    const YAML::Node n = it.second;
    g.base_link = n["base_link"].as<std::string>("");
    g.tip_link = n["tip_link"].as<std::string>("");
    g.joints = asStringList(n["joints"]);
    g.cartesian = !g.base_link.empty() && !g.tip_link.empty();
    g.cartesian_step = n["cartesian_step"].as<double>(0.005);
    g.cartesian_subgroups = asStringList(n["cartesian_subgroups"]);
    g.locked_joints = asStringList(n["locked_joints"]);
    g.assist_joints = asStringList(n["assist_joints"]);
    g.driven_joints = asStringList(n["driven_joints"]);
    g.compensating_subgroup = n["compensating_subgroup"].as<std::string>("");
    g.default_velocity_scaling =
      n["velocity_scaling"].as<double>(out.defaults.velocity_scaling);
    g.default_acceleration_scaling =
      n["acceleration_scaling"].as<double>(out.defaults.acceleration_scaling);

    if (g.joints.empty()) {
      error = "group '" + g.name + "' declares no joints";
      return false;
    }

    const YAML::Node ctrls = n["controllers"];
    if (!ctrls || !ctrls.IsSequence() || ctrls.size() == 0) {
      error = "group '" + g.name + "' declares no controllers";
      return false;
    }
    for (const auto & c : ctrls) {
      ControllerConfig cc;
      cc.name = c["name"].as<std::string>();
      cc.type = c["type"].as<std::string>("follow_joint_trajectory");
      cc.joints = asStringList(c["joints"]);
      if (cc.joints.empty()) {cc.joints = g.joints;}  // default: all group joints
      g.controllers.push_back(std::move(cc));
    }
    out.groups[g.name] = std::move(g);
  }
  return true;
}

void parseNamedPoses(const YAML::Node & root, ManipulationConfig & out)
{
  const YAML::Node poses = root["named_poses"];
  if (!poses || !poses.IsMap()) {return;}
  for (const auto & grp : poses) {
    const std::string group = grp.first.as<std::string>();
    for (const auto & pose : grp.second) {
      out.named_poses[group][pose.first.as<std::string>()] = asDoubleList(pose.second);
    }
  }
}

void parseCollision(const YAML::Node & root, ManipulationConfig & out)
{
  const YAML::Node col = root["collision"];
  if (!col) {return;}
  auto & c = out.collision;
  c.enabled = col["enabled"].as<bool>(c.enabled);
  c.padding = col["padding"].as<double>(c.padding);
  c.margin = col["margin"].as<double>(c.margin);
  c.resolution = col["resolution"].as<double>(c.resolution);
  c.mesh_decimation = col["mesh_decimation"].as<double>(c.mesh_decimation);
  c.mode = col["mode"].as<std::string>(c.mode);
  c.sphere_voxel = col["sphere_voxel"].as<double>(c.sphere_voxel);
  c.sphere_radius_scale = col["sphere_radius_scale"].as<double>(c.sphere_radius_scale);
  c.auto_disable = col["auto_disable"].as<bool>(c.auto_disable);
  c.auto_disable_samples = col["auto_disable_samples"].as<int>(c.auto_disable_samples);
  c.self_chain_distance = col["self_chain_distance"].as<int>(c.self_chain_distance);
  const YAML::Node pairs = col["disabled_pairs"];
  if (pairs && pairs.IsSequence()) {
    for (const auto & p : pairs) {
      if (p.IsSequence() && p.size() == 2) {
        c.disabled_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
      }
    }
  }
}

MotionStep parseStep(const YAML::Node & s)
{
  MotionStep step;
  const std::string type = s["type"].as<std::string>("named");
  step.group = s["group"].as<std::string>("");
  // Shared, type-agnostic fields (parsed for every step type).
  step.velocity_scaling = s["velocity_scaling"].as<double>(0.0);
  step.acceleration_scaling = s["acceleration_scaling"].as<double>(0.0);
  step.blend_radius = s["blend_radius"].as<double>(0.0);
  step.free_orientation = s["free_orientation"].as<bool>(false);

  if (type == "named") {
    step.type = MotionStep::TYPE_NAMED;
    step.named_target = s["named"].as<std::string>(s["named_target"].as<std::string>(""));
  } else if (type == "joint") {
    step.type = MotionStep::TYPE_JOINT;
    step.joint_target = asDoubleList(s["joint_target"] ? s["joint_target"] : s["values"]);
  } else if (type == "cartesian") {
    step.type = MotionStep::TYPE_CARTESIAN;
    step.relative = s["relative"].as<bool>(false);
    step.offset_in_tip_frame = s["offset_in_tip_frame"].as<bool>(false);
    step.reference_frame = s["reference_frame"].as<std::string>("");
    step.cartesian_path = s["cartesian_path"].as<bool>(false);
    auto off = asDoubleList(s["offset"]);
    if (off.size() == 3) {
      step.offset.x = off[0];
      step.offset.y = off[1];
      step.offset.z = off[2];
    }
    // Dual-arm: one offset per subgroup, e.g. offsets: [[0,0.05,0],[0,-0.05,0]].
    if (s["offsets"] && s["offsets"].IsSequence()) {
      for (const auto & o : s["offsets"]) {
        auto v = asDoubleList(o);
        geometry_msgs::msg::Vector3 vv;
        if (v.size() == 3) {vv.x = v[0]; vv.y = v[1]; vv.z = v[2];}
        step.offsets.push_back(vv);
      }
    }
    auto pos = asDoubleList(s["position"]);
    if (pos.size() == 3) {
      step.pose_target.position.x = pos[0];
      step.pose_target.position.y = pos[1];
      step.pose_target.position.z = pos[2];
    }
    auto quat = asDoubleList(s["orientation"]);  // [x, y, z, w]
    if (quat.size() == 4) {
      step.pose_target.orientation.x = quat[0];
      step.pose_target.orientation.y = quat[1];
      step.pose_target.orientation.z = quat[2];
      step.pose_target.orientation.w = quat[3];
    } else {
      step.pose_target.orientation.w = 1.0;
    }
  } else if (type == "gripper") {
    step.type = MotionStep::TYPE_GRIPPER;
    step.named_target = s["named"].as<std::string>("");
    step.gripper_position = s["position"].as<double>(0.0);
  } else if (type == "hold_tip") {
    step.type = MotionStep::TYPE_HOLD_TIP;
    // "targets" -> absolute driven-joint positions; else relative deltas
    // (accepted under "deltas", "joint_target" or "values").
    auto targets = asDoubleList(s["targets"]);
    if (!targets.empty()) {
      step.joint_target = targets;
      step.driven_absolute = true;
    } else {
      step.joint_target = asDoubleList(s["deltas"]);
      if (step.joint_target.empty()) {step.joint_target = asDoubleList(s["joint_target"]);}
      if (step.joint_target.empty()) {step.joint_target = asDoubleList(s["values"]);}
      // Accept either key name; "driven_absolute" mirrors the message field.
      step.driven_absolute =
        s["driven_absolute"].as<bool>(s["absolute"].as<bool>(false));
    }
  } else if (type == "collision") {
    step.type = MotionStep::TYPE_COLLISION;
    step.collision_op = s["op"].as<std::string>("");
    step.collision_id = s["id"].as<std::string>("");
    step.collision_attach_link = s["attach_link"].as<std::string>("");
    const YAML::Node prim = s["primitive"];
    if (prim) {
      const std::string t = prim["type"].as<std::string>("box");
      if (t == "box" || t == "1") {
        step.collision_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      } else if (t == "sphere" || t == "2") {
        step.collision_primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      } else if (t == "cylinder" || t == "3") {
        step.collision_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      }
      for (double d : asDoubleList(prim["dimensions"])) {
        step.collision_primitive.dimensions.push_back(d);
      }
    }
    auto pos = asDoubleList(s["position"]);
    if (pos.size() == 3) {
      step.pose_target.position.x = pos[0];
      step.pose_target.position.y = pos[1];
      step.pose_target.position.z = pos[2];
    }
    auto quat = asDoubleList(s["orientation"]);  // [x, y, z, w]
    if (quat.size() == 4) {
      step.pose_target.orientation.x = quat[0];
      step.pose_target.orientation.y = quat[1];
      step.pose_target.orientation.z = quat[2];
      step.pose_target.orientation.w = quat[3];
    } else {
      step.pose_target.orientation.w = 1.0;
    }
  } else if (type == "follow") {
    step.type = MotionStep::TYPE_FOLLOW;
    step.reference_frame = s["reference_frame"].as<std::string>("");
    auto pos = asDoubleList(s["position"]);
    if (pos.size() == 3) {
      step.pose_target.position.x = pos[0];
      step.pose_target.position.y = pos[1];
      step.pose_target.position.z = pos[2];
    }
    auto quat = asDoubleList(s["orientation"]);  // [x, y, z, w]
    if (quat.size() == 4) {
      step.pose_target.orientation.x = quat[0];
      step.pose_target.orientation.y = quat[1];
      step.pose_target.orientation.z = quat[2];
      step.pose_target.orientation.w = quat[3];
    } else {
      step.pose_target.orientation.w = 1.0;
    }
    step.follow_rate = s["follow_rate"].as<double>(0.0);
    step.follow_timeout = s["follow_timeout"].as<double>(0.0);
    step.follow_position_tolerance = s["follow_position_tolerance"].as<double>(0.0);
    step.follow_settle_time = s["follow_settle_time"].as<double>(0.0);
    step.follow_stop_topic = s["follow_stop_topic"].as<std::string>("");
  }
  return step;
}

void parseSequences(const YAML::Node & root, ManipulationConfig & out)
{
  const YAML::Node seqs = root["sequences"];
  if (!seqs || !seqs.IsMap()) {return;}
  for (const auto & seq : seqs) {
    std::vector<MotionStep> steps;
    for (const auto & s : seq.second) {
      steps.push_back(parseStep(s));
    }
    out.sequences[seq.first.as<std::string>()] = std::move(steps);
  }
}

}  // namespace

bool loadConfig(
  const std::string & move_groups_yaml,
  const std::string & named_poses_yaml,
  const std::string & collision_yaml,
  const std::string & sequences_yaml,
  ManipulationConfig & out, std::string & error)
{
  try {
    if (move_groups_yaml.empty()) {
      error = "move_groups yaml path is empty";
      return false;
    }
    if (!parseGroups(YAML::LoadFile(move_groups_yaml), out, error)) {
      return false;
    }
    if (!named_poses_yaml.empty()) {
      parseNamedPoses(YAML::LoadFile(named_poses_yaml), out);
    }
    if (!collision_yaml.empty()) {
      parseCollision(YAML::LoadFile(collision_yaml), out);
    }
    if (!sequences_yaml.empty()) {
      parseSequences(YAML::LoadFile(sequences_yaml), out);
    }
  } catch (const std::exception & e) {
    error = std::string("YAML parse error: ") + e.what();
    return false;
  }
  return true;
}

}  // namespace bimanual_manipulation
