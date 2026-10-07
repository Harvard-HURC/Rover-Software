// The world's terrain heightmap for the rover's Gazebo systems (the fly camera's ground clearance, the
// drivetrain's ground lookup): found in the entity-component manager and sampled exactly as Gazebo does.
//
// Gazebo scales an image heightmap by the image's own highest pixel, height = pixel / max_pixel * size.z,
// above the heightmap's world position (its model's pose plus <pos>; the URC worlds keep both at the
// origin, sim/README.md "Gazebo lessons"). Row 0 of the image is north (+y), column 0 west (-x), and the
// samples span size.x by size.y. Ported from the fly-camera prototype
// (sim/data/research/flycam/prototype/flycam_proto.cpp), which matched urc_autonomy's sheet within
// 6e-5 m; with the rows flipped it was 19 m off, without the max-pixel scaling up to 56 m.
//
// FindFarFieldApron() reads the far field's heights round the terrain (sim/urc/farfield.py's apron.json), so a
// camera beyond the terrain's edge keeps clear of the landscape drawn there.
//
// Header-only: a plugin includes it and links gz-common5::geospatial (sim/CMakeLists.txt does).
#pragma once

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <sdf/Geometry.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Mesh.hh>

namespace rover_sim {

/// One heightmap's samples in world coordinates.
struct TerrainHeightmap {
  std::string path;            ///< the image on disk (a world's ground.png and ground.json lie next to it)
  gz::math::Vector3d size;     ///< SDF <size>: x and y extent, z of the highest pixel [m]
  gz::math::Vector3d origin;   ///< world position of the heightmap's centre at height 0
  unsigned int samples = 0;    ///< per side
  std::vector<float> heights;  ///< row-major, row 0 north, column 0 west [m above origin.Z()]

  /// The sample nearest to world (x, y), as (row, column); false outside the heightmap.
  bool Nearest(double x, double y, unsigned int& row, unsigned int& col) const {
    double fc, fr;
    if (!Fraction(x, y, fc, fr)) return false;
    col = static_cast<unsigned int>(std::lround(fc));
    row = static_cast<unsigned int>(std::lround(fr));
    return true;
  }

  /// World height at (x, y), bilinear between samples; -infinity outside the heightmap, so that
  /// std::max with it keeps the other value.
  double Height(double x, double y) const {
    double fc, fr;
    if (!Fraction(x, y, fc, fr)) return -std::numeric_limits<double>::infinity();
    const unsigned int c = std::min(samples - 2, static_cast<unsigned int>(fc));
    const unsigned int r = std::min(samples - 2, static_cast<unsigned int>(fr));
    const double ac = fc - c, ar = fr - r;
    auto h = [&](unsigned int row, unsigned int column) { return double(heights[row * samples + column]); };
    return origin.Z() + (1 - ar) * ((1 - ac) * h(r, c) + ac * h(r, c + 1)) +
           ar * ((1 - ac) * h(r + 1, c) + ac * h(r + 1, c + 1));
  }

 private:
  /// Fractional (column, row) of world (x, y); false outside the heightmap.
  bool Fraction(double x, double y, double& col, double& row) const {
    const double u = (x - origin.X()) / size.X() + 0.5;  // 0 west .. 1 east
    const double v = 0.5 - (y - origin.Y()) / size.Y();  // 0 north .. 1 south
    if (samples < 2 || !(u >= 0 && u <= 1 && v >= 0 && v <= 1)) return false;
    col = u * (samples - 1);
    row = v * (samples - 1);
    return true;
  }
};

/// Which of the terrain's two heightmap geometries to read: they may differ in size.z (each image is
/// scaled by its own highest pixel), and only the collision one is what wheels touch.
enum class HeightmapGeometry { kVisual, kCollision };

/// A file path for an SDF URI (file://, model:// through Gazebo's resource paths, or a path); empty if
/// it is not found.
inline std::string ResolveUri(const std::string& uri) {
  const std::string scheme = "file://";
  const std::string path = uri.rfind(scheme, 0) == 0 ? uri.substr(scheme.size()) : uri;
  if (!path.empty() && path.front() == '/' && gz::common::isFile(path)) return path;
  return gz::common::findFile(uri);
}

/// The world's first heightmap of `kind`, loaded; std::nullopt (and a gzerr line) if there is none or
/// its image cannot be read.
inline std::optional<TerrainHeightmap> FindTerrainHeightmap(const gz::sim::EntityComponentManager& ecm,
                                                            HeightmapGeometry kind) {
  std::optional<TerrainHeightmap> out;
  auto load = [&](const gz::sim::Entity& entity, const gz::sim::components::Geometry* geometry) {
    if (geometry->Data().Type() != sdf::GeometryType::HEIGHTMAP) return true;
    const sdf::Heightmap* shape = geometry->Data().HeightmapShape();
    TerrainHeightmap map;
    map.path = ResolveUri(shape->Uri());
    gz::common::ImageHeightmap image;
    if (map.path.empty() || image.Load(map.path) != 0 || image.Width() != image.Height() || image.Width() < 2) {
      gzerr << "cannot read the heightmap " << shape->Uri() << " as a square image\n";
      return false;
    }
    map.size = shape->Size();
    map.origin = gz::sim::worldPose(entity, ecm).Pos() + shape->Position();
    map.samples = image.Width();
    // FillHeightMap gives each pixel over the format's maximum, times size.z; Gazebo stretches the
    // image's own highest pixel to size.z.
    image.FillHeightMap(1, map.samples, map.size, gz::math::Vector3d::One, false, map.heights);
    const float top = *std::max_element(map.heights.begin(), map.heights.end());
    for (auto& h : map.heights) h = top > 0 ? float(h / top * map.size.Z()) : 0.0f;
    out = std::move(map);
    return false;
  };
  if (kind == HeightmapGeometry::kVisual) {
    ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
        [&](const gz::sim::Entity& e, const gz::sim::components::Visual*,
            const gz::sim::components::Geometry* g) { return load(e, g); });
  } else {
    ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
        [&](const gz::sim::Entity& e, const gz::sim::components::Collision*,
            const gz::sim::components::Geometry* g) { return load(e, g); });
  }
  return out;
}

/// The far field's surface round the terrain: its vertex heights on a regular grid (row 0 north, column 0
/// west), each cell split from its south-west to its north-east corner as the mesh is (apron.json,
/// "rover-apron/1", sim/urc/farfield.py).
struct FarFieldApron {
  double spacing = 0.0, x0 = 0.0, y0 = 0.0, z0 = 0.0;  // [m] grid step, world x, y of vertex (0, 0), model z
  int rows = 0, cols = 0;
  std::vector<double> z;  // [m] above z0, row-major

  /// Height of the far-field surface at world (x, y); -infinity off the grid.
  double Height(double x, double y) const {
    const double fc = (x - x0) / spacing, fr = (y0 - y) / spacing;
    if (rows < 2 || cols < 2 || !(fc >= 0 && fr >= 0 && fc <= cols - 1 && fr <= rows - 1)) {
      return -std::numeric_limits<double>::infinity();
    }
    const int c = std::min(cols - 2, static_cast<int>(fc)), r = std::min(rows - 2, static_cast<int>(fr));
    const double u = fc - c, v = fr - r;  // east and south within the cell
    auto at = [&](int row, int col) { return z[static_cast<size_t>(row) * cols + col]; };
    if (u + v <= 1) return z0 + at(r, c) + v * (at(r + 1, c) - at(r, c)) + u * (at(r, c + 1) - at(r, c));
    return z0 + at(r + 1, c + 1) + (1 - v) * (at(r, c + 1) - at(r + 1, c + 1)) +
           (1 - u) * (at(r + 1, c) - at(r + 1, c + 1));
  }
};

/// The far field's apron.json beside the mesh of the world's far-field visual (a mesh named farfield.glb, in
/// meshes/ of its model), placed by the visual's world pose; std::nullopt (no line) for a world without
/// one, a gzerr line for one that does not parse.
inline std::optional<FarFieldApron> FindFarFieldApron(const gz::sim::EntityComponentManager& ecm) {
  std::optional<FarFieldApron> out;
  ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity& entity, const gz::sim::components::Visual*,
          const gz::sim::components::Geometry* geometry) {
        const sdf::Mesh* mesh = geometry->Data().MeshShape();
        const std::string suffix = "/farfield.glb";
        if (geometry->Data().Type() != sdf::GeometryType::MESH || !mesh || mesh->Uri().size() < suffix.size() ||
            mesh->Uri().compare(mesh->Uri().size() - suffix.size(), suffix.size(), suffix) != 0) {
          return true;
        }
        const std::string glb = ResolveUri(mesh->Uri());
        const std::string path = gz::common::joinPaths(gz::common::parentPath(gz::common::parentPath(glb)),
                                                       "apron.json");
        std::ifstream in(path);
        if (glb.empty() || !in) return false;
        std::stringstream text;
        text << in.rdbuf();
        google::protobuf::Struct doc;
        const bool parsed = google::protobuf::util::JsonStringToMessage(text.str(), &doc).ok();
        const auto& f = doc.fields();
        if (!parsed || !f.count("format") || f.at("format").string_value() != "rover-apron/1" || !f.count("z")) {
          gzerr << "far-field apron " << path << " is not a rover-apron/1 document; it is not used\n";
          return false;
        }
        FarFieldApron apron;
        const auto pose = gz::sim::worldPose(entity, ecm);
        apron.spacing = f.at("spacing").number_value();
        apron.x0 = f.at("x0").number_value() + pose.Pos().X();
        apron.y0 = f.at("y0").number_value() + pose.Pos().Y();
        apron.z0 = pose.Pos().Z();
        apron.rows = static_cast<int>(f.at("rows").number_value());
        apron.cols = static_cast<int>(f.at("cols").number_value());
        for (const auto& v : f.at("z").list_value().values()) apron.z.push_back(v.number_value());
        if (apron.spacing <= 0 || apron.z.size() != static_cast<size_t>(apron.rows) * apron.cols) {
          gzerr << "far-field apron " << path << ": its grid does not match its size; it is not used\n";
          return false;
        }
        out = std::move(apron);
        return false;
      });
  return out;
}

}  // namespace rover_sim
