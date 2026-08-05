#include "bimanual_manipulation/collision_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <random>
#include <tuple>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <resource_retriever/retriever.hpp>

namespace bimanual_manipulation
{

namespace
{

// Vertex-clustering decimation: snap vertices onto a grid of size `voxel`,
// merge co-located ones to their centroid, and drop the resulting degenerate
// triangles. Cheaply turns a detailed mesh into a coarse collision proxy.
void decimate(
  std::vector<fcl::Vector3d> & verts, std::vector<fcl::Triangle> & tris, double voxel)
{
  if (voxel <= 0.0 || verts.empty()) {return;}
  const double inv = 1.0 / voxel;
  std::map<std::tuple<int, int, int>, int> cell_to_new;
  std::vector<fcl::Vector3d> sum;
  std::vector<int> count;
  std::vector<int> remap(verts.size());

  for (size_t i = 0; i < verts.size(); ++i) {
    const auto key = std::make_tuple(
      static_cast<int>(std::floor(verts[i].x() * inv)),
      static_cast<int>(std::floor(verts[i].y() * inv)),
      static_cast<int>(std::floor(verts[i].z() * inv)));
    auto it = cell_to_new.find(key);
    int idx;
    if (it == cell_to_new.end()) {
      idx = static_cast<int>(sum.size());
      cell_to_new.emplace(key, idx);
      sum.push_back(verts[i]);
      count.push_back(1);
    } else {
      idx = it->second;
      sum[idx] += verts[i];
      ++count[idx];
    }
    remap[i] = idx;
  }

  std::vector<fcl::Vector3d> nv(sum.size());
  for (size_t i = 0; i < sum.size(); ++i) {nv[i] = sum[i] / count[i];}

  std::vector<fcl::Triangle> nt;
  nt.reserve(tris.size());
  for (const auto & t : tris) {
    const int a = remap[t[0]], b = remap[t[1]], c = remap[t[2]];
    if (a != b && b != c && a != c) {nt.emplace_back(a, b, c);}
  }
  verts.swap(nv);
  tris.swap(nt);
}

// Load a mesh resource (package:// or file://) into an FCL BVH model,
// optionally decimated. Adds the resulting triangle count to *tri_count.
std::shared_ptr<fcl::CollisionGeometryd> loadMesh(
  const std::string & uri, const urdf::Vector3 & scale, double voxel, size_t * tri_count)
{
  try {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res = retriever.get(uri);

    // assimp needs the file extension as a hint to pick the right importer
    // when reading from memory (otherwise STL/DAE detection often fails).
    std::string ext;
    const auto dot = uri.find_last_of('.');
    if (dot != std::string::npos) {
      ext = uri.substr(dot + 1);
      for (auto & c : ext) {c = static_cast<char>(::tolower(c));}
    }

    Assimp::Importer importer;
    const aiScene * scene = importer.ReadFileFromMemory(
      res.data.get(), res.size,
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices, ext.c_str());
    if (!scene || !scene->HasMeshes()) {
      std::fprintf(stderr, "[collision_model] mesh '%s' produced no usable geometry: %s\n",
        uri.c_str(), importer.GetErrorString());
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
      decimate(vertices, triangles, voxel);
      if (tri_count) {*tri_count += triangles.size();}
      if (!triangles.empty()) {model->addSubModel(vertices, triangles);}
    }
    model->endModel();
    return model;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[collision_model] failed to load mesh '%s': %s\n",
      uri.c_str(), e.what());
    return nullptr;
  }
}

// Load all mesh vertices (scaled) for the sphere approximation.
std::vector<Eigen::Vector3d> loadMeshVertices(
  const std::string & uri, const urdf::Vector3 & scale)
{
  std::vector<Eigen::Vector3d> out;
  try {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res = retriever.get(uri);
    std::string ext;
    const auto dot = uri.find_last_of('.');
    if (dot != std::string::npos) {
      ext = uri.substr(dot + 1);
      for (auto & c : ext) {c = static_cast<char>(::tolower(c));}
    }
    Assimp::Importer importer;
    const aiScene * scene = importer.ReadFileFromMemory(
      res.data.get(), res.size, aiProcess_Triangulate, ext.c_str());
    if (!scene || !scene->HasMeshes()) {return out;}
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
      const aiMesh * mesh = scene->mMeshes[m];
      for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
        const aiVector3D & p = mesh->mVertices[v];
        out.emplace_back(p.x * scale.x, p.y * scale.y, p.z * scale.z);
      }
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[collision_model] failed to load mesh '%s': %s\n",
      uri.c_str(), e.what());
  }
  return out;
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

  sphere_mode_ = (settings_.mode == "spheres");

  // Shared: adjacency predicate + user-disabled pairs.
  auto same_or_adjacent = [&](int na, int nb) {
      if (na == nb) {return true;}
      return nodes_[na].parent == nb || nodes_[nb].parent == na;
    };
  std::set<std::pair<std::string, std::string>> disabled;
  for (const auto & d : settings_.disabled_pairs) {
    disabled.insert(std::minmax(d.first, d.second));
  }

  // --- spheres mode: approximate each link with spheres, check analytically -
  if (sphere_mode_) {
    buildSpheres(model);

    spheres_by_node_.assign(nodes_.size(), {});
    for (size_t i = 0; i < spheres_.size(); ++i) {
      spheres_by_node_[spheres_[i].node].push_back(static_cast<int>(i));
    }
    for (size_t i = 0; i < nodes_.size(); ++i) {
      if (spheres_by_node_[i].empty()) {links_without_collision_.push_back(nodes_[i].name);}
    }

    // Per-node bounding sphere (link frame) for broad-phase rejection.
    node_bound_center_.assign(nodes_.size(), Eigen::Vector3d::Zero());
    node_bound_radius_.assign(nodes_.size(), 0.0);
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const auto & si = spheres_by_node_[i];
      if (si.empty()) {continue;}
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (int k : si) {c += spheres_[k].center;}
      c /= static_cast<double>(si.size());
      double r = 0.0;
      for (int k : si) {r = std::max(r, (spheres_[k].center - c).norm() + spheres_[k].radius);}
      node_bound_center_[i] = c;
      node_bound_radius_[i] = r;
    }
    for (size_t i = 0; i < nodes_.size(); ++i) {
      if (spheres_by_node_[i].empty()) {continue;}
      for (size_t j = i + 1; j < nodes_.size(); ++j) {
        if (spheres_by_node_[j].empty()) {continue;}
        if (same_or_adjacent(static_cast<int>(i), static_cast<int>(j))) {continue;}
        if (disabled.count(std::minmax(nodes_[i].name, nodes_[j].name))) {continue;}
        check_node_pairs_.emplace_back(static_cast<int>(i), static_cast<int>(j));
      }
    }
    autoDisableAlwaysColliding(model);
    return true;
  }

  // --- mesh mode: collision shapes per link (FCL) -------------------------
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
          ++meshes_total_;
          auto geom = loadMesh(m->filename, m->scale, settings_.mesh_decimation, &mesh_triangles_);
          if (!geom) {++meshes_failed_;}
          return geom;
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

  autoDisableAlwaysColliding(model);
  return true;
}

void CollisionModel::autoDisableAlwaysColliding(const urdf::Model & model)
{
  if (!settings_.auto_disable) {return;}
  const int N = std::max(1, settings_.auto_disable_samples);
  const double margin = settings_.margin;

  // Sampling bounds for the actuated joints.
  std::vector<std::tuple<std::string, double, double>> jb;
  for (const auto & n : nodes_) {
    if (n.joint_name.empty()) {continue;}
    auto j = model.getJoint(n.joint_name);
    if (!j || j->type == urdf::Joint::FIXED || j->type == urdf::Joint::UNKNOWN) {continue;}
    double lo = -M_PI, hi = M_PI;
    if (j->limits && j->type != urdf::Joint::CONTINUOUS) {lo = j->limits->lower; hi = j->limits->upper;}
    jb.emplace_back(n.joint_name, lo, hi);
  }

  std::mt19937 rng(20240517u);
  auto sample = [&](int s) {
      std::map<std::string, double> jv;
      if (s > 0) {
        for (const auto & [name, lo, hi] : jb) {
          std::uniform_real_distribution<double> d(lo, hi);
          jv[name] = d(rng);
        }
      }
      return jv;
    };

  std::vector<Eigen::Isometry3d> tf;

  if (sphere_mode_) {
    std::vector<int> cnt(check_node_pairs_.size(), 0);
    std::vector<char> at_default(check_node_pairs_.size(), 0);
    std::vector<Eigen::Vector3d> centers(spheres_.size());
    for (int s = 0; s < N; ++s) {
      computeLinkTransforms(sample(s), tf);
      for (size_t i = 0; i < spheres_.size(); ++i) {
        centers[i] = tf[spheres_[i].node] * spheres_[i].center;
      }
      for (size_t pi = 0; pi < check_node_pairs_.size(); ++pi) {
        const auto & pr = check_node_pairs_[pi];
        bool hit = false;
        for (int ia : spheres_by_node_[pr.first]) {
          for (int ib : spheres_by_node_[pr.second]) {
            if ((centers[ia] - centers[ib]).norm() <
              spheres_[ia].radius + spheres_[ib].radius + margin)
            {
              hit = true;
              break;
            }
          }
          if (hit) {break;}
        }
        if (hit) {++cnt[pi]; if (s == 0) {at_default[pi] = 1;}}
      }
    }
    std::vector<std::pair<int, int>> kept;
    for (size_t pi = 0; pi < check_node_pairs_.size(); ++pi) {
      const auto & pr = check_node_pairs_[pi];
      if (at_default[pi] || cnt[pi] >= N) {
        ++auto_disabled_;
        auto_disabled_names_.push_back(nodes_[pr.first].name + " <-> " + nodes_[pr.second].name);
      } else {
        kept.push_back(pr);
      }
    }
    check_node_pairs_.swap(kept);
    return;
  }

  // Mesh mode.
  if (shape_objs_.size() != shapes_.size()) {
    shape_objs_.clear();
    for (const auto & s : shapes_) {
      shape_objs_.push_back(
        std::make_shared<fcl::CollisionObjectd>(s.geom, Eigen::Isometry3d::Identity()));
    }
  }
  fcl::CollisionRequestd creq;
  fcl::CollisionResultd cres;
  std::vector<int> cnt(check_pairs_.size(), 0);
  std::vector<char> at_default(check_pairs_.size(), 0);
  for (int s = 0; s < N; ++s) {
    computeLinkTransforms(sample(s), tf);
    for (size_t i = 0; i < shapes_.size(); ++i) {
      shape_objs_[i]->setTransform(Eigen::Isometry3d(tf[shapes_[i].node] * shapes_[i].origin));
      shape_objs_[i]->computeAABB();
    }
    for (size_t pi = 0; pi < check_pairs_.size(); ++pi) {
      cres.clear();
      fcl::collide(
        shape_objs_[check_pairs_[pi].first].get(),
        shape_objs_[check_pairs_[pi].second].get(), creq, cres);
      if (cres.isCollision()) {++cnt[pi]; if (s == 0) {at_default[pi] = 1;}}
    }
  }
  std::vector<std::pair<int, int>> kept;
  for (size_t pi = 0; pi < check_pairs_.size(); ++pi) {
    const auto & pair = check_pairs_[pi];
    if (at_default[pi] || cnt[pi] >= N) {
      ++auto_disabled_;
      auto_disabled_names_.push_back(
        nodes_[shapes_[pair.first].node].name + " <-> " + nodes_[shapes_[pair.second].node].name);
    } else {
      kept.push_back(pair);
    }
  }
  check_pairs_.swap(kept);
}

void CollisionModel::buildSpheres(const urdf::Model & model)
{
  for (const auto & node : nodes_) {
    urdf::LinkConstSharedPtr link = model.getLink(node.name);
    if (!link) {continue;}
    const int node_idx = node_index_[node.name];

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
      appendSpheresForGeometry(node_idx, geometry, urdfToEigen(origin));
    }
  }
}

void CollisionModel::appendSpheresForGeometry(
  int node, const urdf::GeometrySharedPtr & g, const Eigen::Isometry3d & origin)
{
  const double voxel = std::max(settings_.sphere_voxel, 1e-3);
  const double rscale = settings_.sphere_radius_scale;
  auto add = [&](const Eigen::Vector3d & c_local, double r) {
      spheres_.push_back(LinkSphere{node, Eigen::Vector3d(origin * c_local), r * rscale});
    };

  switch (g->type) {
    case urdf::Geometry::SPHERE: {
      auto s = std::dynamic_pointer_cast<urdf::Sphere>(g);
      add(Eigen::Vector3d::Zero(), s->radius);
      break;
    }
    case urdf::Geometry::BOX: {
      auto b = std::dynamic_pointer_cast<urdf::Box>(g);
      const Eigen::Vector3d dim(b->dim.x, b->dim.y, b->dim.z);
      const int nx = std::max(1, static_cast<int>(std::ceil(dim.x() / voxel)));
      const int ny = std::max(1, static_cast<int>(std::ceil(dim.y() / voxel)));
      const int nz = std::max(1, static_cast<int>(std::ceil(dim.z() / voxel)));
      const Eigen::Vector3d s(dim.x() / nx, dim.y() / ny, dim.z() / nz);
      const double r = 0.5 * s.norm();  // covers a cell
      for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
          for (int iz = 0; iz < nz; ++iz) {
            add(Eigen::Vector3d(
                -dim.x() / 2 + (ix + 0.5) * s.x(),
                -dim.y() / 2 + (iy + 0.5) * s.y(),
                -dim.z() / 2 + (iz + 0.5) * s.z()), r);
          }
        }
      }
      break;
    }
    case urdf::Geometry::CYLINDER: {
      auto c = std::dynamic_pointer_cast<urdf::Cylinder>(g);
      const int n = std::max(1, static_cast<int>(std::ceil(c->length / voxel)));
      const double sz = c->length / n;
      const double r = std::sqrt(c->radius * c->radius + 0.25 * sz * sz);
      for (int k = 0; k < n; ++k) {
        add(Eigen::Vector3d(0, 0, -c->length / 2 + (k + 0.5) * sz), r);
      }
      break;
    }
    case urdf::Geometry::MESH: {
      auto m = std::dynamic_pointer_cast<urdf::Mesh>(g);
      ++meshes_total_;
      const auto verts = loadMeshVertices(m->filename, m->scale);
      if (verts.empty()) {++meshes_failed_; break;}

      // Cluster vertices on a voxel grid -> one sphere per occupied cell.
      const double inv = 1.0 / voxel;
      std::map<std::tuple<int, int, int>, int> cell;
      std::vector<Eigen::Vector3d> sum;
      std::vector<int> cnt;
      std::vector<int> vcell(verts.size());
      for (size_t i = 0; i < verts.size(); ++i) {
        const auto key = std::make_tuple(
          static_cast<int>(std::floor(verts[i].x() * inv)),
          static_cast<int>(std::floor(verts[i].y() * inv)),
          static_cast<int>(std::floor(verts[i].z() * inv)));
        auto it = cell.find(key);
        int idx;
        if (it == cell.end()) {
          idx = static_cast<int>(sum.size());
          cell.emplace(key, idx);
          sum.push_back(verts[i]);
          cnt.push_back(1);
        } else {
          idx = it->second;
          sum[idx] += verts[i];
          ++cnt[idx];
        }
        vcell[i] = idx;
      }
      std::vector<Eigen::Vector3d> cen(sum.size());
      for (size_t k = 0; k < sum.size(); ++k) {cen[k] = sum[k] / cnt[k];}
      std::vector<double> maxd(sum.size(), 0.0);
      for (size_t i = 0; i < verts.size(); ++i) {
        maxd[vcell[i]] = std::max(maxd[vcell[i]], (verts[i] - cen[vcell[i]]).norm());
      }
      for (size_t k = 0; k < cen.size(); ++k) {
        add(cen[k], std::max(maxd[k], voxel * 0.25));
      }
      break;
    }
    default:
      break;
  }
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
  return checkStateImpl(joint_values, nullptr);
}

bool CollisionModel::checkState(
  const std::map<std::string, double> & joint_values,
  const std::set<std::string> & active_joints) const
{
  return checkStateImpl(joint_values, &active_joints);
}

bool CollisionModel::checkStateImpl(
  const std::map<std::string, double> & joint_values,
  const std::set<std::string> * active_joints) const
{
  if (!settings_.enabled) {return true;}

  std::vector<Eigen::Isometry3d> tf;
  computeLinkTransforms(joint_values, tf);

  // A node is "active" when its transform depends on one of the active joints,
  // i.e. its parent joint is active or its parent node is active. With no
  // active set, everything is active (full check).
  std::vector<char> active(nodes_.size(), active_joints ? 0 : 1);
  if (active_joints) {
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const int p = nodes_[i].parent;
      if (p >= 0 && (active[p] || active_joints->count(nodes_[i].joint_name))) {
        active[i] = 1;
      }
    }
  }

  if (sphere_mode_) {
    return checkStateSpheres(tf, active, active_joints);
  }
  return checkStateMesh(tf, active, active_joints);
}

bool CollisionModel::checkStateMesh(
  const std::vector<Eigen::Isometry3d> & tf,
  const std::vector<char> & active, const std::set<std::string> * active_joints) const
{
  // Build the reusable FCL objects once, then just refresh their transforms.
  if (shape_objs_.size() != shapes_.size()) {
    shape_objs_.clear();
    shape_objs_.reserve(shapes_.size());
    for (const auto & s : shapes_) {
      shape_objs_.push_back(
        std::make_shared<fcl::CollisionObjectd>(s.geom, Eigen::Isometry3d::Identity()));
    }
  }
  for (size_t i = 0; i < shapes_.size(); ++i) {
    shape_objs_[i]->setTransform(Eigen::Isometry3d(tf[shapes_[i].node] * shapes_[i].origin));
    shape_objs_[i]->computeAABB();
  }

  const double margin = settings_.margin;

  // True when the two objects touch (margin == 0) or are closer than `margin`.
  // Uses an exact distance query when a margin is requested, so the separation
  // applies to meshes as well as primitives.
  auto tooClose = [&](fcl::CollisionObjectd * a, fcl::CollisionObjectd * b) -> bool {
      if (margin > 0.0) {
        if (a->getAABB().distance(b->getAABB()) > margin) {return false;}
        fcl::DistanceRequestd dreq;
        dreq.enable_nearest_points = false;
        fcl::DistanceResultd dres;
        fcl::distance(a, b, dreq, dres);
        return dres.min_distance < margin;
      }
      if (!a->getAABB().overlap(b->getAABB())) {return false;}
      fcl::CollisionRequestd creq;
      fcl::CollisionResultd cres;
      fcl::collide(a, b, creq, cres);
      return cres.isCollision();
    };

  // --- self collision ------------------------------------------------------
  for (const auto & pair : check_pairs_) {
    // Skip pairs that cannot have changed (both links static for this motion).
    if (active_joints && !active[shapes_[pair.first].node] &&
      !active[shapes_[pair.second].node])
    {
      continue;
    }
    if (tooClose(shape_objs_[pair.first].get(), shape_objs_[pair.second].get())) {return false;}
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
    for (size_t i = 0; i < shape_objs_.size(); ++i) {
      const int sn = shapes_[i].node;
      if (active_joints && !active[sn]) {continue;}  // static link, unchanged
      // Skip the link the object is attached to and its direct parent.
      if (wo.attached_node >= 0) {
        if (sn == wo.attached_node || nodes_[sn].parent == wo.attached_node ||
          nodes_[wo.attached_node].parent == sn)
        {
          continue;
        }
      }
      if (tooClose(wobj, shape_objs_[i].get())) {return false;}
    }
  }

  return true;
}

namespace
{
// Signed distance from a point (already in the primitive's local frame) to the
// surface of a solid primitive. Negative inside. Only the sign/magnitude near 0
// matters for our threshold test.
double pointToPrimitive(
  const Eigen::Vector3d & p, const shape_msgs::msg::SolidPrimitive & prim)
{
  using SP = shape_msgs::msg::SolidPrimitive;
  const auto & d = prim.dimensions;
  switch (prim.type) {
    case SP::BOX: {
      if (d.size() < 3) {return 1e9;}
      const Eigen::Vector3d h(d[SP::BOX_X] / 2, d[SP::BOX_Y] / 2, d[SP::BOX_Z] / 2);
      const Eigen::Vector3d q = p.cwiseAbs() - h;
      const double outside = q.cwiseMax(0.0).norm();
      const double inside = std::min(std::max({q.x(), q.y(), q.z()}), 0.0);
      return outside + inside;   // >0 outside, <0 inside
    }
    case SP::SPHERE:
      return d.empty() ? 1e9 : p.norm() - d[SP::SPHERE_RADIUS];
    case SP::CYLINDER: {
      if (d.size() < 2) {return 1e9;}
      const double radial = std::hypot(p.x(), p.y()) - d[SP::CYLINDER_RADIUS];
      const double axial = std::abs(p.z()) - d[SP::CYLINDER_HEIGHT] / 2;
      if (radial <= 0 && axial <= 0) {return std::max(radial, axial);}   // inside
      const double ro = std::max(radial, 0.0), ao = std::max(axial, 0.0);
      return std::hypot(ro, ao);
    }
    default:
      return 1e9;
  }
}
}  // namespace

bool CollisionModel::checkStateSpheres(
  const std::vector<Eigen::Isometry3d> & tf,
  const std::vector<char> & active, const std::set<std::string> * active_joints) const
{
  const double margin = settings_.margin;

  // World-frame centers of every sphere for this configuration.
  std::vector<Eigen::Vector3d> centers(spheres_.size());
  for (size_t i = 0; i < spheres_.size(); ++i) {
    centers[i] = tf[spheres_[i].node] * spheres_[i].center;
  }
  // World-frame node bounding-sphere centers (for broad-phase reject).
  std::vector<Eigen::Vector3d> wbc(nodes_.size());
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (!spheres_by_node_[i].empty()) {wbc[i] = tf[i] * node_bound_center_[i];}
  }

  // --- self collision: analytic sphere-sphere over the link pairs ----------
  for (const auto & pr : check_node_pairs_) {
    if (active_joints && !active[pr.first] && !active[pr.second]) {continue;}
    // Broad-phase: skip the pair when their bounding spheres are far apart.
    if ((wbc[pr.first] - wbc[pr.second]).norm() >
      node_bound_radius_[pr.first] + node_bound_radius_[pr.second] + margin)
    {
      continue;
    }
    for (int ia : spheres_by_node_[pr.first]) {
      for (int ib : spheres_by_node_[pr.second]) {
        const double d = (centers[ia] - centers[ib]).norm();
        if (d < spheres_[ia].radius + spheres_[ib].radius + margin) {return false;}
      }
    }
  }

  // --- world objects: sphere vs primitive ----------------------------------
  std::lock_guard<std::mutex> lock(world_mutex_);
  for (const auto & wo : world_) {
    const Eigen::Isometry3d obj_tf =
      (wo.attached_node >= 0) ? Eigen::Isometry3d(tf[wo.attached_node] * wo.pose) : wo.pose;
    const Eigen::Isometry3d obj_inv = obj_tf.inverse();
    for (size_t i = 0; i < spheres_.size(); ++i) {
      const int sn = spheres_[i].node;
      if (active_joints && !active[sn]) {continue;}
      if (wo.attached_node >= 0) {
        if (sn == wo.attached_node || nodes_[sn].parent == wo.attached_node ||
          nodes_[wo.attached_node].parent == sn)
        {
          continue;
        }
      }
      const double d = pointToPrimitive(obj_inv * centers[i], wo.primitive);
      if (d < spheres_[i].radius + margin) {return false;}
    }
  }

  return true;
}

std::vector<std::string> CollisionModel::describeCollisions(
  const std::map<std::string, double> & joint_values,
  const std::set<std::string> * active_joints) const
{
  std::vector<std::string> out;
  if (!settings_.enabled) {return out;}
  std::vector<Eigen::Isometry3d> tf;
  computeLinkTransforms(joint_values, tf);
  const double margin = settings_.margin;
  const size_t kMax = 40;

  // Same active-link filter as checkState, so only the relevant pairs show.
  std::vector<char> active(nodes_.size(), active_joints ? 0 : 1);
  if (active_joints) {
    for (size_t i = 0; i < nodes_.size(); ++i) {
      const int p = nodes_[i].parent;
      if (p >= 0 && (active[p] || active_joints->count(nodes_[i].joint_name))) {
        active[i] = 1;
      }
    }
  }

  auto fmt = [](double m) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.3f", m);
      return std::string(buf);
    };

  if (sphere_mode_) {
    std::vector<Eigen::Vector3d> centers(spheres_.size());
    for (size_t i = 0; i < spheres_.size(); ++i) {
      centers[i] = tf[spheres_[i].node] * spheres_[i].center;
    }
    for (const auto & pr : check_node_pairs_) {
      if (active_joints && !active[pr.first] && !active[pr.second]) {continue;}
      double worst = 1e9;
      for (int ia : spheres_by_node_[pr.first]) {
        for (int ib : spheres_by_node_[pr.second]) {
          const double gap = (centers[ia] - centers[ib]).norm() -
            (spheres_[ia].radius + spheres_[ib].radius);
          worst = std::min(worst, gap);
        }
      }
      if (worst < margin) {
        out.push_back(
          "self: " + nodes_[pr.first].name + " <-> " + nodes_[pr.second].name +
          " (overlap " + fmt(-worst) + " m)");
        if (out.size() >= kMax) {return out;}
      }
    }
    std::lock_guard<std::mutex> lock(world_mutex_);
    for (const auto & wo : world_) {
      const Eigen::Isometry3d obj_inv =
        ((wo.attached_node >= 0) ?
        Eigen::Isometry3d(tf[wo.attached_node] * wo.pose) : wo.pose).inverse();
      for (size_t i = 0; i < spheres_.size(); ++i) {
        if (active_joints && !active[spheres_[i].node]) {continue;}
        if (pointToPrimitive(obj_inv * centers[i], wo.primitive) < spheres_[i].radius + margin) {
          out.push_back("object '" + wo.id + "' <-> " + nodes_[spheres_[i].node].name);
          if (out.size() >= kMax) {return out;}
          break;  // one report per object/link is enough
        }
      }
    }
    return out;
  }

  // Mesh mode.
  if (shape_objs_.size() != shapes_.size()) {
    shape_objs_.clear();
    for (const auto & s : shapes_) {
      shape_objs_.push_back(
        std::make_shared<fcl::CollisionObjectd>(s.geom, Eigen::Isometry3d::Identity()));
    }
  }
  for (size_t i = 0; i < shapes_.size(); ++i) {
    shape_objs_[i]->setTransform(Eigen::Isometry3d(tf[shapes_[i].node] * shapes_[i].origin));
    shape_objs_[i]->computeAABB();
  }
  fcl::CollisionRequestd creq;
  fcl::CollisionResultd cres;
  for (const auto & pair : check_pairs_) {
    if (active_joints && !active[shapes_[pair.first].node] &&
      !active[shapes_[pair.second].node])
    {
      continue;
    }
    cres.clear();
    fcl::collide(shape_objs_[pair.first].get(), shape_objs_[pair.second].get(), creq, cres);
    if (cres.isCollision()) {
      out.push_back(
        "self: " + nodes_[shapes_[pair.first].node].name + " <-> " +
        nodes_[shapes_[pair.second].node].name);
      if (out.size() >= kMax) {return out;}
    }
  }
  std::lock_guard<std::mutex> lock(world_mutex_);
  for (const auto & wo : world_) {
    fcl::CollisionObjectd * wobj = wo.obj.get();
    if (wo.attached_node >= 0) {
      wobj->setTransform(Eigen::Isometry3d(tf[wo.attached_node] * wo.pose));
      wobj->computeAABB();
    }
    for (size_t i = 0; i < shape_objs_.size(); ++i) {
      if (active_joints && !active[shapes_[i].node]) {continue;}
      cres.clear();
      fcl::collide(wobj, shape_objs_[i].get(), creq, cres);
      if (cres.isCollision()) {
        out.push_back("object '" + wo.id + "' <-> " + nodes_[shapes_[i].node].name);
        if (out.size() >= kMax) {return out;}
      }
    }
  }
  return out;
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
