#include "bimanual_manipulation/collision_model.hpp"

#include <algorithm>
#include <cstdio>
#include <deque>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <resource_retriever/retriever.hpp>

namespace bimanual_manipulation
{

namespace
{

// Load a mesh resource (package:// or file://) into an FCL BVH model.
std::shared_ptr<fcl::CollisionGeometryd> loadMesh(
  const std::string & uri, const urdf::Vector3 & scale)
{
  try {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res = retriever.get(uri);

    Assimp::Importer importer;
    const aiScene * scene = importer.ReadFileFromMemory(
      res.data.get(), res.size,
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices, nullptr);
    if (!scene || !scene->HasMeshes()) {
      return nullptr;
    }

    using BVH = fcl::BVHModel<fcl::OBBRSS<double>>;
    auto model = std::make_shared<BVH>();
    model->beginModel();
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
      const aiMesh * mesh = scene->mMeshes[m];
      std::vector<fcl::Vector3d> vertices;
      vertices.reserve(mesh->mNumVertices);
      for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
        const aiVector3D & p = mesh->mVertices[v];
        vertices.emplace_back(p.x * scale.x, p.y * scale.y, p.z * scale.z);
      }
      std::vector<fcl::Triangle> triangles;
      triangles.reserve(mesh->mNumFaces);
      for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
        const aiFace & face = mesh->mFaces[f];
        if (face.mNumIndices == 3) {
          triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }
      }
      model->addSubModel(vertices, triangles);
    }
    model->endModel();
    return model;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[collision_model] failed to load mesh '%s': %s\n",
      uri.c_str(), e.what());
    return nullptr;
  }
}

}  // namespace

std::shared_ptr<fcl::CollisionGeometryd> CollisionModel::makeGeometry(
  const shape_msgs::msg::SolidPrimitive & p) const
{
  using SP = shape_msgs::msg::SolidPrimitive;
  switch (p.type) {
    case SP::BOX:
      if (p.dimensions.size() >= 3) {
        return std::make_shared<fcl::Boxd>(
          p.dimensions[SP::BOX_X], p.dimensions[SP::BOX_Y], p.dimensions[SP::BOX_Z]);
      }
      break;
    case SP::SPHERE:
      if (!p.dimensions.empty()) {
        return std::make_shared<fcl::Sphered>(p.dimensions[SP::SPHERE_RADIUS]);
      }
      break;
    case SP::CYLINDER:
      if (p.dimensions.size() >= 2) {
        return std::make_shared<fcl::Cylinderd>(
          p.dimensions[SP::CYLINDER_RADIUS], p.dimensions[SP::CYLINDER_HEIGHT]);
      }
      break;
    default:
      break;
  }
  return nullptr;
}

bool CollisionModel::init(
  const urdf::Model & model, const CollisionSettings & settings, std::string & error)
{
  settings_ = settings;
  if (!model.getRoot()) {
    error = "URDF has no root link";
    return false;
  }
  root_frame_ = model.getRoot()->name;

  // --- breadth-first walk of the kinematic tree (parents before children) --
  std::deque<urdf::LinkConstSharedPtr> queue;
  queue.push_back(model.getRoot());
  while (!queue.empty()) {
    urdf::LinkConstSharedPtr link = queue.front();
    queue.pop_front();

    LinkNode node;
    node.name = link->name;
    node.parent = -1;
    if (link->parent_joint) {
      auto it = node_index_.find(link->parent_joint->parent_link_name);
      node.parent = (it != node_index_.end()) ? it->second : -1;
      const auto & j = link->parent_joint;
      node.origin = urdfToEigen(j->parent_to_joint_origin_transform);
      node.joint_type = j->type;
      node.joint_name = j->name;
      node.axis = Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
      if (node.axis.norm() > 1e-9) {
        node.axis.normalize();
      }
    }
    node_index_[node.name] = static_cast<int>(nodes_.size());
    nodes_.push_back(node);

    for (const auto & child : link->child_links) {
      queue.push_back(child);
    }
  }

  // --- collision shapes per link ------------------------------------------
  const double pad = settings_.padding;
  auto build = [&](const urdf::GeometrySharedPtr & g)
    -> std::shared_ptr<fcl::CollisionGeometryd> {
      switch (g->type) {
        case urdf::Geometry::BOX: {
          auto b = std::dynamic_pointer_cast<urdf::Box>(g);
          return std::make_shared<fcl::Boxd>(
            b->dim.x + 2 * pad, b->dim.y + 2 * pad, b->dim.z + 2 * pad);
        }
        case urdf::Geometry::SPHERE: {
          auto s = std::dynamic_pointer_cast<urdf::Sphere>(g);
          return std::make_shared<fcl::Sphered>(s->radius + pad);
        }
        case urdf::Geometry::CYLINDER: {
          auto c = std::dynamic_pointer_cast<urdf::Cylinder>(g);
          return std::make_shared<fcl::Cylinderd>(c->radius + pad, c->length + 2 * pad);
        }
        case urdf::Geometry::MESH: {
          auto m = std::dynamic_pointer_cast<urdf::Mesh>(g);
          return loadMesh(m->filename, m->scale);
        }
        default:
          return nullptr;
      }
    };

  for (const auto & node : nodes_) {
    urdf::LinkConstSharedPtr link = model.getLink(node.name);
    if (!link) {continue;}
    const int node_idx = node_index_[node.name];

    // Prefer <collision> geometry; fall back to <visual> when a link declares
    // none (common on robots that only model visuals).
    std::vector<std::pair<urdf::GeometrySharedPtr, urdf::Pose>> geoms;
    if (!link->collision_array.empty()) {
      for (const auto & c : link->collision_array) {
        if (c && c->geometry) {geoms.emplace_back(c->geometry, c->origin);}
      }
    } else {
      for (const auto & v : link->visual_array) {
        if (v && v->geometry) {geoms.emplace_back(v->geometry, v->origin);}
      }
      if (!geoms.empty()) {++visual_fallback_links_;}
    }

    for (const auto & [geometry, origin] : geoms) {
      auto geom = build(geometry);
      if (!geom) {continue;}
      shapes_.push_back({node_idx, geom, urdfToEigen(origin)});
    }
  }

  // Note which links ended up with no collision geometry (e.g. mesh failed to
  // load, or the link genuinely has none) so the server can warn about it.
  {
    std::vector<int> shapes_per_node(nodes_.size(), 0);
    for (const auto & s : shapes_) {++shapes_per_node[s.node];}
    for (size_t i = 0; i < nodes_.size(); ++i) {
      if (shapes_per_node[i] == 0) {links_without_collision_.push_back(nodes_[i].name);}
    }
  }

  // --- allowed (skipped) self-collision pairs ------------------------------
  auto same_or_adjacent = [&](int na, int nb) {
      if (na == nb) {return true;}
      return nodes_[na].parent == nb || nodes_[nb].parent == na;
    };

  std::set<std::pair<std::string, std::string>> disabled;
  for (const auto & d : settings_.disabled_pairs) {
    disabled.insert(std::minmax(d.first, d.second));
  }

  for (size_t i = 0; i < shapes_.size(); ++i) {
    for (size_t j = i + 1; j < shapes_.size(); ++j) {
      const int ni = shapes_[i].node;
      const int nj = shapes_[j].node;
      if (same_or_adjacent(ni, nj)) {continue;}
      auto names = std::minmax(nodes_[ni].name, nodes_[nj].name);
      if (disabled.count(names)) {continue;}
      check_pairs_.emplace_back(static_cast<int>(i), static_cast<int>(j));
    }
  }

  return true;
}

void CollisionModel::computeLinkTransforms(
  const std::map<std::string, double> & joint_values,
  std::vector<Eigen::Isometry3d> & out) const
{
  out.resize(nodes_.size());
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const LinkNode & n = nodes_[i];
    if (n.parent < 0) {
      out[i] = Eigen::Isometry3d::Identity();
      continue;
    }
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    double q = 0.0;
    auto it = joint_values.find(n.joint_name);
    if (it != joint_values.end()) {q = it->second;}
    switch (n.joint_type) {
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::CONTINUOUS:
        motion.linear() = Eigen::AngleAxisd(q, n.axis).toRotationMatrix();
        break;
      case urdf::Joint::PRISMATIC:
        motion.translation() = n.axis * q;
        break;
      default:
        break;  // fixed / unsupported -> identity
    }
    out[i] = out[n.parent] * n.origin * motion;
  }
}

bool CollisionModel::checkState(const std::map<std::string, double> & joint_values) const
{
  if (!settings_.enabled) {return true;}

  std::vector<Eigen::Isometry3d> tf;
  computeLinkTransforms(joint_values, tf);

  // Build collision objects once for every shape.
  std::vector<fcl::CollisionObjectd> objs;
  objs.reserve(shapes_.size());
  for (const auto & s : shapes_) {
    objs.emplace_back(s.geom, Eigen::Isometry3d(tf[s.node] * s.origin));
  }

  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;

  // --- self collision ------------------------------------------------------
  for (const auto & pair : check_pairs_) {
    fcl::CollisionObjectd & a = objs[pair.first];
    fcl::CollisionObjectd & b = objs[pair.second];
    if (!a.getAABB().overlap(b.getAABB())) {continue;}
    result.clear();
    fcl::collide(&a, &b, request, result);
    if (result.isCollision()) {return false;}
  }

  // --- world objects -------------------------------------------------------
  std::lock_guard<std::mutex> lock(world_mutex_);
  for (const auto & wo : world_) {
    fcl::CollisionObjectd * wobj = wo.obj.get();
    if (wo.attached_node >= 0) {
      // Attached objects ride along with their link.
      wobj->setTransform(Eigen::Isometry3d(tf[wo.attached_node] * wo.pose));
      wobj->computeAABB();
    }
    for (size_t i = 0; i < objs.size(); ++i) {
      // Skip the link the object is attached to and its direct parent.
      if (wo.attached_node >= 0) {
        const int sn = shapes_[i].node;
        if (sn == wo.attached_node || nodes_[sn].parent == wo.attached_node ||
          nodes_[wo.attached_node].parent == sn)
        {
          continue;
        }
      }
      if (!wobj->getAABB().overlap(objs[i].getAABB())) {continue;}
      result.clear();
      fcl::collide(wobj, &objs[i], request, result);
      if (result.isCollision()) {return false;}
    }
  }

  return true;
}

bool CollisionModel::addObject(
  const std::string & id, const shape_msgs::msg::SolidPrimitive & primitive,
  const Eigen::Isometry3d & pose, std::string & error)
{
  auto geom = makeGeometry(primitive);
  if (!geom) {
    error = "unsupported or malformed primitive for object '" + id + "'";
    return false;
  }
  std::lock_guard<std::mutex> lock(world_mutex_);
  world_.erase(
    std::remove_if(world_.begin(), world_.end(),
    [&](const WorldObject & o) {return o.id == id;}), world_.end());
  WorldObject wo;
  wo.id = id;
  wo.obj = std::make_shared<fcl::CollisionObjectd>(geom, pose);
  wo.pose = pose;
  wo.attached_node = -1;
  wo.primitive = primitive;
  world_.push_back(std::move(wo));
  return true;
}

bool CollisionModel::addAttachedObject(
  const std::string & id, const shape_msgs::msg::SolidPrimitive & primitive,
  const std::string & link, const Eigen::Isometry3d & pose_in_link, std::string & error)
{
  auto it = node_index_.find(link);
  if (it == node_index_.end()) {
    error = "unknown attach link '" + link + "'";
    return false;
  }
  auto geom = makeGeometry(primitive);
  if (!geom) {
    error = "unsupported or malformed primitive for object '" + id + "'";
    return false;
  }
  std::lock_guard<std::mutex> lock(world_mutex_);
  world_.erase(
    std::remove_if(world_.begin(), world_.end(),
    [&](const WorldObject & o) {return o.id == id;}), world_.end());
  WorldObject wo;
  wo.id = id;
  wo.obj = std::make_shared<fcl::CollisionObjectd>(geom, pose_in_link);
  wo.pose = pose_in_link;
  wo.attached_node = it->second;
  wo.primitive = primitive;
  wo.attached_link = link;
  world_.push_back(std::move(wo));
  return true;
}

bool CollisionModel::removeObject(const std::string & id)
{
  std::lock_guard<std::mutex> lock(world_mutex_);
  const size_t before = world_.size();
  world_.erase(
    std::remove_if(world_.begin(), world_.end(),
    [&](const WorldObject & o) {return o.id == id;}), world_.end());
  return world_.size() != before;
}

void CollisionModel::clearObjects()
{
  std::lock_guard<std::mutex> lock(world_mutex_);
  world_.clear();
}

std::vector<CollisionModel::ObjectInfo> CollisionModel::objects() const
{
  std::lock_guard<std::mutex> lock(world_mutex_);
  std::vector<ObjectInfo> out;
  out.reserve(world_.size());
  for (const auto & wo : world_) {
    out.push_back({wo.id, wo.primitive, wo.pose, wo.attached_link});
  }
  return out;
}

}  // namespace bimanual_manipulation
