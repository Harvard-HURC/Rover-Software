// The world's ground map for the drivetrain (design spec docs/superpowers/specs/2026-10-06-urc-realism-design.md,
// section 9.1): what ground a wheel stands on, and how it grips and resists.
//
// A world's terrain model holds, next to its heightmap image:
// - ground.png: 8-bit, one index per heightmap sample, on exactly the heightmap's grid (n x n, centred on the
//   heightmap, row 0 north, column 0 west); lookup is nearest-sample;
// - ground.json ("rover-ground/2"): the traction table (one row per index: terrains.Traction's fields plus the
//   dust factor), the default index, the collision map of the terrain model's other shapes (exact names, then
//   prefixes, then terrain_default) and object_default, the surface of objects whose SDF sets no friction.
// sim/urc/world.py writes both. FindTerrainShape() finds the world's collision heightmap (its image through
// terrain_heightmap.hh's ResolveUri) and FindGroundMap() reads the two files beside it.
//
// JSON is parsed as a google.protobuf.Struct: protobuf ships with gz-msgs, so this adds no dependency.
// Header-only: a plugin includes it and links gz-sim (sim/CMakeLists.txt does).
#pragma once

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include <array>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Image.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <sdf/Geometry.hh>
#include <sdf/Heightmap.hh>

#include "terrain_heightmap.hh"

namespace rover_sim {

/// How a ground grips and resists a wheel (spec 5.6): one row of ground.json's traction table, with the names
/// of terrains.Traction. mu is the Coulomb limit of the gross tyre force (static below the Stribeck speed,
/// kinetic above); crr and bulldoze are hub forces as fractions of the wheel's load; slip is the
/// force-dependent slip (steady slip ratio = slip x traction / load); a wheel's dig factor grows by
/// dig_rate / sinkage_m per metre of slip, up to dig_max; dust scales the dust emitters.
struct Traction {
  std::string key = "coulomb";
  double mu_s = 1.0, mu_k = 1.0, crr = 0.0, bulldoze = 0.0, slip = 0.0, sinkage_m = 0.0;
  double dig_rate = 0.0, dig_max = 1.0, dust = 0.0;

  bool Digs() const { return dig_rate > 0.0 && sinkage_m > 0.0 && dig_max > 1.0; }
};

/// ground.json and ground.png of one world. Parse() and LoadRaster() fill it; Place() puts the raster on the
/// heightmap. Lookups return nullptr where the map says nothing.
class GroundMap {
 public:
  static constexpr const char* kFormat = "rover-ground/2";

  /// Read ground.json's text; false, with the reason in `error`, if it is not a valid rover-ground/2 document.
  bool Parse(const std::string& json, std::string& error) {
    google::protobuf::Struct doc;
    if (!google::protobuf::util::JsonStringToMessage(json, &doc).ok()) return Fail(error, "not JSON");
    const auto& f = doc.fields();
    if (String(f, "format") != kFormat) return Fail(error, std::string("format is not ") + kFormat);
    size_m_ = Number(f, "size_m", 0.0);
    samples_ = static_cast<unsigned int>(Number(f, "samples", 0.0));
    if (!(size_m_ > 0.0) || samples_ < 2) return Fail(error, "size_m and samples must be positive");
    by_index_.fill(-1);
    const auto types = f.find("types");
    if (types == f.end() || !types->second.has_list_value()) return Fail(error, "no types");
    for (const auto& value : types->second.list_value().values()) {
      const auto& t = value.struct_value().fields();
      const double index = Number(t, "index", -1.0);
      if (index < 0 || index > 255 || index != std::floor(index)) return Fail(error, "a type index is not 0-255");
      Traction row;
      row.key = String(t, "key");
      if (row.key.empty()) return Fail(error, "a type has no key");
      row.mu_s = Number(t, "mu_s", row.mu_s);
      row.mu_k = Number(t, "mu_k", row.mu_k);
      row.crr = Number(t, "crr", row.crr);
      row.bulldoze = Number(t, "bulldoze", row.bulldoze);
      row.slip = Number(t, "slip", row.slip);
      row.sinkage_m = Number(t, "sinkage_m", row.sinkage_m);
      row.dig_rate = Number(t, "dig_rate", row.dig_rate);
      row.dig_max = Number(t, "dig_max", row.dig_max);
      row.dust = Number(t, "dust", row.dust);
      by_index_[static_cast<size_t>(index)] = static_cast<int>(types_.size());
      by_key_[row.key] = static_cast<int>(types_.size());
      types_.push_back(row);
    }
    const double fallback = Number(f, "default", -1.0);
    if (fallback < 0 || fallback > 255 || by_index_[static_cast<size_t>(fallback)] < 0) {
      return Fail(error, "default is not one of the types' indices");
    }
    default_ = by_index_[static_cast<size_t>(fallback)];
    for (const auto& [name, key] : Strings(f, "collisions")) collisions_[name] = Index(key);
    for (const auto& [prefix, key] : Strings(f, "prefixes")) prefixes_.emplace_back(prefix, Index(key));
    terrain_default_ = Index(String(f, "terrain_default"));
    object_default_ = Index(String(f, "object_default"));
    for (const auto& [name, index] : collisions_) {
      if (index < 0) return Fail(error, "collision " + name + " names an unknown type");
    }
    for (const auto& [prefix, index] : prefixes_) {
      if (index < 0) return Fail(error, "prefix " + prefix + " names an unknown type");
    }
    return true;
  }

  /// Read ground.png; false, with the reason in `error`, unless it is samples x samples.
  bool LoadRaster(const std::string& path, std::string& error) {
    gz::common::Image image;
    if (image.Load(path) != 0 || !image.Valid()) return Fail(error, "cannot read " + path);
    if (image.Width() != samples_ || image.Height() != samples_) {
      return Fail(error, path + " is not " + std::to_string(samples_) + " x " + std::to_string(samples_));
    }
    // RGBData converts any 8-bit grey, palette or colour image to rows of R, G, B from the top row down; a
    // grey index comes back in all three channels.
    const std::vector<unsigned char> rgb = image.RGBData();
    raster_.resize(size_t(samples_) * samples_);
    for (size_t i = 0; i < raster_.size(); ++i) raster_[i] = rgb[3 * i];
    return true;
  }

  /// Centre the raster on the heightmap's world position (its model pose plus <pos>).
  void Place(const gz::math::Vector3d& centre) { centre_ = centre; }

  /// The ground at world (x, y): the type of the nearest sample, the default type for an index the table
  /// lacks; nullptr outside the raster or before LoadRaster().
  const Traction* At(double x, double y) const {
    const double col = ((x - centre_.X()) / size_m_ + 0.5) * (samples_ - 1);
    const double row = (0.5 - (y - centre_.Y()) / size_m_) * (samples_ - 1);
    if (raster_.empty() || !std::isfinite(col) || !std::isfinite(row)) return nullptr;
    const long c = std::lround(col), r = std::lround(row);
    if (c < 0 || r < 0 || c >= long(samples_) || r >= long(samples_)) return nullptr;
    const int index = by_index_[raster_[size_t(r) * samples_ + size_t(c)]];
    return &types_[size_t(index < 0 ? default_ : index)];
  }

  /// A shape of the terrain model other than the heightmap, by its collision name: the exact name, else the
  /// first matching prefix, else terrain_default; nullptr if none applies.
  const Traction* Collision(const std::string& name) const {
    if (const auto it = collisions_.find(name); it != collisions_.end()) return &types_[size_t(it->second)];
    for (const auto& [prefix, index] : prefixes_) {
      if (name.rfind(prefix, 0) == 0) return &types_[size_t(index)];
    }
    return Row(terrain_default_);
  }

  const Traction* Type(const std::string& key) const {
    const auto it = by_key_.find(key);
    return it == by_key_.end() ? nullptr : &types_[size_t(it->second)];
  }
  const Traction* Default() const { return Row(default_); }
  const Traction* ObjectDefault() const { return Row(object_default_); }
  double SizeM() const { return size_m_; }
  unsigned int Samples() const { return samples_; }

 private:
  using Fields = google::protobuf::Map<std::string, google::protobuf::Value>;

  static bool Fail(std::string& error, const std::string& why) {
    error = why;
    return false;
  }
  static std::string String(const Fields& f, const std::string& key) {
    const auto it = f.find(key);
    return it != f.end() && it->second.has_string_value() ? it->second.string_value() : std::string();
  }
  static double Number(const Fields& f, const std::string& key, double fallback) {
    const auto it = f.find(key);
    return it != f.end() && it->second.has_number_value() ? it->second.number_value() : fallback;
  }
  static std::vector<std::pair<std::string, std::string>> Strings(const Fields& f, const std::string& key) {
    std::vector<std::pair<std::string, std::string>> out;
    const auto it = f.find(key);
    if (it != f.end() && it->second.has_struct_value()) {
      for (const auto& [name, value] : it->second.struct_value().fields()) out.emplace_back(name, value.string_value());
    }
    return out;
  }
  int Index(const std::string& key) const {
    const auto it = by_key_.find(key);
    return it == by_key_.end() ? -1 : it->second;
  }
  const Traction* Row(int index) const { return index < 0 ? nullptr : &types_[size_t(index)]; }

  double size_m_ = 0.0;
  unsigned int samples_ = 0;
  gz::math::Vector3d centre_;
  std::vector<Traction> types_;
  std::array<int, 256> by_index_{};
  std::unordered_map<std::string, int> by_key_;
  std::unordered_map<std::string, int> collisions_;
  std::vector<std::pair<std::string, int>> prefixes_;
  int default_ = -1, terrain_default_ = -1, object_default_ = -1;
  std::vector<unsigned char> raster_;
};

/// The world's collision heightmap: its collision entity and the model it belongs to (kNullEntity if the world
/// has none), and the directory of its image.
struct TerrainShape {
  gz::sim::Entity collision = gz::sim::kNullEntity;
  gz::sim::Entity model = gz::sim::kNullEntity;
  std::string directory;
  gz::math::Vector3d centre;
  double size = 0.0;  ///< east-west extent [m]
};

inline TerrainShape FindTerrainShape(const gz::sim::EntityComponentManager& ecm) {
  TerrainShape out;
  ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity& entity, const gz::sim::components::Collision*,
          const gz::sim::components::Geometry* geometry) {
        if (geometry->Data().Type() != sdf::GeometryType::HEIGHTMAP) return true;
        const sdf::Heightmap* shape = geometry->Data().HeightmapShape();
        out.collision = entity;
        out.model = gz::sim::topLevelModel(entity, ecm);
        const std::string image = ResolveUri(shape->Uri());
        out.directory = image.empty() ? std::string() : gz::common::parentPath(image);
        out.centre = gz::sim::worldPose(entity, ecm).Pos() + shape->Position();
        out.size = shape->Size().X();
        return false;
      });
  return out;
}

/// The ground map next to the world's collision heightmap; std::nullopt without a heightmap or without
/// ground.json beside it (that world's ground is the drivetrain's default surface everywhere). A ground.json
/// that does not parse or match its raster is an error (gzerr), and also gives std::nullopt.
inline std::optional<GroundMap> FindGroundMap(const TerrainShape& terrain) {
  if (terrain.directory.empty()) return std::nullopt;
  const std::string json = gz::common::joinPaths(terrain.directory, "ground.json");
  std::ifstream in(json);
  if (!in) return std::nullopt;
  std::stringstream text;
  text << in.rdbuf();
  GroundMap map;
  std::string error;
  bool ok = map.Parse(text.str(), error) &&
            map.LoadRaster(gz::common::joinPaths(terrain.directory, "ground.png"), error);
  if (ok && std::abs(map.SizeM() - terrain.size) > 1e-6 * terrain.size) {
    ok = false;
    error = "size_m " + std::to_string(map.SizeM()) + " is not the heightmap's " + std::to_string(terrain.size);
  }
  if (!ok) {
    gzerr << "ground map " << json << ": " << error << "; using the default surface everywhere\n";
    return std::nullopt;
  }
  map.Place(terrain.centre);
  return map;
}

}  // namespace rover_sim
