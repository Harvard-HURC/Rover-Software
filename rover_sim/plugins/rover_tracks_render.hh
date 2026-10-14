// The ruts on screen (rover_tracks.hh has what they are): gz::rendering markers made on the Sensors system's
// rendering thread from an events::SceneUpdate hook, the pattern of the FlyCamera plugin. The drivetrain owns a
// TrackSystem: it feeds the layer at the end of every step and connects the hook. Camera sensors only (the
// station's views, the rover's RGB-D); the Gazebo GUI renders its own scene and shows no ruts (user decision
// 2026-10-07).
//
// Each frame, at the scene's sim time: TrackLayer::Take() (under the layer's mutex) hands over the records due and
// not drawn yet; outside the lock they become triangles appended to their chunk's markers (never recreated: a
// marker made anew every frame cost 0.5-1 ms per frame and -7 to -32 % throughput, measured in the research). A
// chunk is a visual <model>/tracks/<id> with a floor marker per darkness level (f1-f4: the shared transparent
// floor materials) and a berm marker per ground (b<ground>: an opaque material of its own, the ground's colour);
// a pit is a visual of its own, p<index>, removed when a deeper pit replaces it. Chunks dropped from the ring go
// with their materials. A frame builds at most <frame_budget> records, newest first: a station that connects
// late (no camera renders, so no hook runs, while nobody watches) catches up over a few frames instead of
// stalling the simulation, which waits for the frame's scene update.
//
// The visual heightmap (the heights; the collision one in worlds without a visual, the contact's height on a
// plane) and the albedo map (the berms' colour) are read on a background thread started by the first frame (the
// Sensors system draws one at start-up, when it makes the cameras, watched or not; measured): a process without
// the Sensors system reads nothing. A frame with records to draw before they are read waits for them, so every
// frame shows what is due at its time; the rover has laid nothing by then, so only a stress test waits (70-100 ms,
// once, measured on Delivery and the proving ground).
//
// gz-rendering 8.2.2 traps (sim/README.md, Gazebo lessons): a marker turns its material's shadows off (receiving
// is turned back on here; casting cannot be: berms show relief by shading only); a transparent material moves
// only the renderables linked when its transparency is set to render queue 200, so it is set again after each
// marker takes it, or Terra (queue 11) draws over the floor; dynamic renderables have no UVs or vertex colours,
// so darkness comes in levels, colour per chunk and ground. No rendering pointer is kept between frames (the
// ogre2 plugin is unloaded before systems: a stored pointer crashes gz at shutdown): visuals and materials are
// looked up by name.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Image.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/rendering/Marker.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>
#include <sdf/Element.hh>

#include "rover_tracks.hh"
#include "terrain_heightmap.hh"

namespace rover_sim::tracks {

/// Terra over PBS for the same albedo and light, linear RGB (M 2026-10-07: a flat PBS square of albedo.png's colour
/// just above the ground, seen from above in the mission sun, against the Terra round it, at 8 spots of sand,
/// clay, regolith, sand sheet and gravel on the proving ground and Delivery: 0.93-1.03; mean 0.99 / 1.00 / 1.00 of
/// the 5 spots whose red the colour map does not clip and whose square stayed above the slope). Terra draws the
/// colour map's own colour: the research's (1.05, 0.956, 1.147) made up for its estimate of that colour from
/// layer 0, and drew pink berms on albedo.png's.
constexpr std::array<double, 3> kTerraGain{0.99, 1.0, 1.0};
/// Berms are turned-over ground, a little darker than the surface (M: the research prototype's tuning).
constexpr double kBermShade = 0.92;
/// The floor overlay: dark, moist soil, linear (M).
constexpr std::array<float, 3> kFloorColour{0.06f, 0.045f, 0.03f};
/// The berms' albedo in a world without an albedo map or a terrain texture (a plane, test worlds), linear (A).
constexpr std::array<double, 3> kDefaultAlbedo{0.30, 0.22, 0.15};
/// The albedo is averaged over (2 k + 1)^2 texels round a point: a chunk's colour is not one mottle's.
constexpr int kAlbedoReach = 2;

inline double SrgbToLinear(double c) { return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4); }

/// Where the ground is drawn, as the server thread finds it (paths only; Load reads them).
struct Ground {
  TerrainHeightmap heightmap;  ///< the visual heightmap, else the collision one; path "": none (a plane)
  std::string albedo;          ///< albedo.png beside the visual heightmap's image (gen_worlds), else ""
  std::string texture;         ///< the visual heightmap's first diffuse texture (worlds without one), else ""
};

inline Ground FindGround(const gz::sim::EntityComponentManager& ecm) {
  Ground g;
  std::string texture;
  if (auto map = FindTerrainHeightmapShape(ecm, HeightmapGeometry::kVisual, nullptr, &texture)) {
    g.heightmap = *map;
    const std::string albedo =
        map->path.empty() ? std::string() : gz::common::joinPaths(gz::common::parentPath(map->path), "albedo.png");
    if (!albedo.empty() && gz::common::isFile(albedo)) g.albedo = albedo;
    if (!texture.empty()) g.texture = ResolveUri(texture);
  } else if (auto collision = FindTerrainHeightmapShape(ecm, HeightmapGeometry::kCollision)) {
    g.heightmap = *collision;
  }
  return g;
}

/// The ground's albedo, linear RGB: albedo.png (sRGB, row 0 north, over the heightmap's extent) or one colour.
struct Albedo {
  int n = 0;
  double size = 0.0, cx = 0.0, cy = 0.0;
  std::vector<unsigned char> srgb;  // n x n x 3
  std::array<double, 3> flat = kDefaultAlbedo;
  std::array<double, 256> lut{};

  std::array<double, 3> At(double x, double y) const {
    if (n < 1) return flat;
    const int col = int(std::lround(((x - cx) / size + 0.5) * n - 0.5));
    const int row = int(std::lround((0.5 - (y - cy) / size) * n - 0.5));
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    int count = 0;
    for (int r = std::max(0, row - kAlbedoReach); r <= std::min(n - 1, row + kAlbedoReach); ++r) {
      for (int c = std::max(0, col - kAlbedoReach); c <= std::min(n - 1, col + kAlbedoReach); ++c) {
        for (int k = 0; k < 3; ++k) sum[size_t(k)] += lut[srgb[(size_t(r) * n + c) * 3 + k]];
        ++count;
      }
    }
    if (!count) return flat;  // off the map
    for (auto& v : sum) v /= count;
    return sum;
  }
};

/// The heights and the albedo, read once.
struct Surface {
  std::optional<TerrainHeightmap> heights;
  Albedo albedo;

  double Height(double x, double y, double fallback) const {
    if (!heights) return fallback;
    const double h = heights->Height(x, y);
    return std::isfinite(h) ? h : fallback;
  }
};

inline std::shared_ptr<const Surface> LoadSurface(Ground g) {
  auto out = std::make_shared<Surface>();
  if (!g.heightmap.path.empty()) {
    if (LoadHeights(g.heightmap)) {
      out->heights = std::move(g.heightmap);
    } else {
      gzwarn << "RoverDrivetrain tracks: cannot read the heightmap " << g.heightmap.path
             << "; ruts lie at their contacts' height\n";
    }
  }
  Albedo& a = out->albedo;
  for (int i = 0; i < 256; ++i) a.lut[size_t(i)] = SrgbToLinear(i / 255.0);
  const auto read = [](const std::string& path, gz::common::Image& image) {
    return !path.empty() && image.Load(path) == 0 && image.Valid() && image.Width() > 0;
  };
  gz::common::Image image;
  if (read(g.albedo, image) && image.Width() == image.Height() && out->heights) {
    a.n = int(image.Width());
    a.srgb = image.RGBData();
    a.size = out->heights->size.X();
    a.cx = out->heights->origin.X();
    a.cy = out->heights->origin.Y();
  } else if (read(g.texture, image)) {  // a world without a colour map: its first texture's mean colour
    const std::vector<unsigned char> rgb = image.RGBData();
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    for (size_t i = 0; i < rgb.size(); ++i) sum[i % 3] += a.lut[rgb[i]];
    for (auto& v : sum) v /= std::max<size_t>(1, rgb.size() / 3);
    a.flat = sum;
  }
  return out;
}

/// The rendering side: draws what a TrackLayer hands over (the notes at the top). OnSceneUpdate runs on the
/// rendering thread only; SetGround once on the server thread; the statistics on any thread.
class Renderer {
 public:
  Renderer(std::string prefix, const Params& p) : prefix_(std::move(prefix)), p_(p) {}

  void SetGround(const Ground& g) {
    std::lock_guard<std::mutex> lock(ground_mutex_);
    ground_ = g;
  }

  void OnSceneUpdate(const TrackLayer& layer) {
    const auto start = std::chrono::steady_clock::now();
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene) return;
    if (!surface_ && !loading_.valid()) {
      std::lock_guard<std::mutex> lock(ground_mutex_);
      if (!ground_) return;  // before the first step: nothing laid yet
      loading_ = std::async(std::launch::async, LoadSurface, *ground_);
    }
    const Stamp time = std::chrono::duration_cast<std::chrono::nanoseconds>(scene->Time()).count();
    const Update u = layer.Take(time, drawn_, size_t(std::max(1, p_.frame_budget)));
    if (u.stale) return;
    if (u.reset) {
      for (const auto& [id, chunk] : built_) DestroyChunk(scene, id, chunk);
      built_.clear();
    }
    for (const uint64_t id : u.dropped) {
      if (const auto it = built_.find(id); it != built_.end()) {
        DestroyChunk(scene, id, it->second);
        built_.erase(it);
      }
    }
    for (const PitRef& pit : u.dead) {
      if (auto vis = scene->VisualByName(PitName(pit.chunk, pit.index))) scene->DestroyVisual(vis, true);
    }
    if (!u.batches.empty()) {
      if (!surface_) surface_ = loading_.get();  // still reading the maps: wait, once
      Prepare(scene);
      for (const auto& batch : u.batches) Draw(scene, batch);
    }
    const uint64_t took = uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::steady_clock::now() - start).count());
    frames_ += 1;
    visuals_ = scene->VisualCount();
    built_records_ += u.taken;
    if (u.held) held_frames_ += 1;
    nanoseconds_ += took;
    if (took > max_nanoseconds_.load()) max_nanoseconds_ = took;
  }

  struct Stats {
    uint64_t frames = 0, records = 0;
    uint64_t held = 0;     ///< frames that left records laid after their time to a later frame
    uint64_t visuals = 0;  ///< the scene's visuals after the last frame (all of them, not only the ruts')
    double seconds = 0.0;
    double max_seconds = 0.0;  ///< the longest frame since the last call
  };
  Stats stats() {
    return {frames_.load(), built_records_.load(), held_frames_.load(), visuals_.load(),
            1e-9 * double(nanoseconds_.load()), 1e-9 * double(max_nanoseconds_.exchange(0))};
  }

 private:
  struct Built {
    std::vector<std::string> materials;  // the chunk's own (berms)
  };

  std::string ChunkName(uint64_t id) const { return prefix_ + "/" + std::to_string(id); }
  std::string PitName(uint64_t id, uint32_t index) const { return ChunkName(id) + "/p" + std::to_string(index); }
  std::string FloorMaterial(int level) const { return prefix_ + "/floor" + std::to_string(level); }

  double Height(double x, double y, double fallback) const { return surface_->Height(x, y, fallback); }

  /// The root visual and the floor materials, once per scene.
  void Prepare(const gz::rendering::ScenePtr& scene) {
    if (!scene->VisualByName(prefix_)) scene->RootVisual()->AddChild(scene->CreateVisual(prefix_));
    for (int k = 1; k < kLevels; ++k) {
      if (scene->MaterialRegistered(FloorMaterial(k))) continue;
      auto m = scene->CreateMaterial(FloorMaterial(k));
      const auto [r, g, b] = kFloorColour;
      m->SetDiffuse(gz::math::Color(r, g, b, 1.0f));
      m->SetAmbient(gz::math::Color(r, g, b, 1.0f));
      m->SetSpecular(gz::math::Color(0, 0, 0, 1));
      m->SetRoughness(1.0f);
      m->SetMetalness(0.0f);
      m->SetTransparency(std::pow(1.0 - kLayerOpacity, k));
      m->SetDepthWriteEnabled(false);  // depth cameras and point clouds do not see the floor
      m->SetRenderOrder(2.0f);         // a depth bias: over the terrain it lies 4 mm above
    }
  }

  /// The chunk's berm material for `ground`, made the first time a record needs it, coloured by the albedo
  /// there (the first such record in the chunk's order: the same whatever the frames).
  std::string BermMaterial(const gz::rendering::ScenePtr& scene, uint64_t id, uint8_t ground, const Section& at) {
    const std::string name = ChunkName(id) + "/b" + std::to_string(int(ground));
    if (scene->MaterialRegistered(name)) return name;
    const auto c = surface_->albedo.At(at.x, at.y);
    const float r = float(c[0] * kTerraGain[0] * kBermShade), g = float(c[1] * kTerraGain[1] * kBermShade),
                b = float(c[2] * kTerraGain[2] * kBermShade);
    auto m = scene->CreateMaterial(name);
    m->SetDiffuse(gz::math::Color(r, g, b, 1.0f));
    m->SetAmbient(gz::math::Color(r, g, b, 1.0f));
    m->SetSpecular(gz::math::Color(0, 0, 0, 1));
    m->SetRoughness(1.0f);
    m->SetMetalness(0.0f);
    built_[id].materials.push_back(name);
    return name;
  }

  void Draw(const gz::rendering::ScenePtr& scene, const Update::Batch& batch) {
    const std::string name = ChunkName(batch.chunk);
    auto chunk = scene->VisualByName(name);
    if (!chunk) {
      chunk = scene->CreateVisual(name);
      scene->VisualByName(prefix_)->AddChild(chunk);
      built_[batch.chunk];
    }
    const HeightFn height = [this](double x, double y, double fallback) { return Height(x, y, fallback); };
    std::array<std::vector<Vec3>, kLevels> floor;
    std::map<uint8_t, std::vector<Vec3>> berms;
    Mesh mesh;
    for (size_t k = 0; k < batch.records.size(); ++k) {
      const Segment& s = batch.records[k];
      mesh.Clear();
      if (s.pit) {
        BuildPit(p_, s, height, mesh);
        DrawPit(scene, chunk, batch.chunk, batch.index[k], s, mesh);
        continue;
      }
      BuildTrack(p_, s, height, mesh);
      for (int level = 1; level < kLevels; ++level) {
        auto& f = mesh.floor[size_t(level)];
        floor[size_t(level)].insert(floor[size_t(level)].end(), f.begin(), f.end());
      }
      if (!mesh.berm.empty()) {
        BermMaterial(scene, batch.chunk, s.ground, s.b);
        auto& b = berms[s.ground];
        b.insert(b.end(), mesh.berm.begin(), mesh.berm.end());
      }
    }
    for (int level = 1; level < kLevels; ++level) {
      if (floor[size_t(level)].empty()) continue;
      Append(scene, chunk, name + "/f" + std::to_string(level), FloorMaterial(level), true, floor[size_t(level)]);
    }
    for (const auto& [ground, vertices] : berms) {
      const std::string berm = name + "/b" + std::to_string(int(ground));
      Append(scene, chunk, berm, berm, false, vertices);
    }
  }

  void DrawPit(const gz::rendering::ScenePtr& scene, const gz::rendering::VisualPtr& chunk, uint64_t id,
               uint32_t index, const Segment& s, const Mesh& mesh) {
    auto vis = scene->CreateVisual(PitName(id, index));
    for (int level = 1; level < kLevels; ++level) {
      if (!mesh.floor[size_t(level)].empty()) {
        vis->AddGeometry(Marker(scene, FloorMaterial(level), true, mesh.floor[size_t(level)]));
      }
    }
    if (!mesh.berm.empty()) vis->AddGeometry(Marker(scene, BermMaterial(scene, id, s.ground, s.b), false, mesh.berm));
    chunk->AddChild(vis);
  }

  static gz::rendering::MarkerPtr Marker(const gz::rendering::ScenePtr& scene, const std::string& material,
                                         bool transparent, const std::vector<Vec3>& vertices) {
    auto marker = scene->CreateMarker();
    marker->SetType(gz::rendering::MarkerType::MT_TRIANGLE_LIST);
    for (const auto& v : vertices) marker->AddPoint(gz::math::Vector3d(v.x, v.y, v.z), gz::math::Color::Black);
    auto mat = scene->Material(material);
    marker->SetMaterial(mat, false);  // turns the material's shadows off
    mat->SetReceiveShadows(true);
    if (transparent) mat->SetTransparency(mat->Transparency());  // after linking: render queue 200, after Terra
    return marker;
  }

  /// Append triangles to the marker of child visual `name` of `parent`, made with `material` if new.
  static void Append(const gz::rendering::ScenePtr& scene, const gz::rendering::VisualPtr& parent,
                     const std::string& name, const std::string& material, bool transparent,
                     const std::vector<Vec3>& vertices) {
    auto vis = scene->VisualByName(name);
    gz::rendering::MarkerPtr marker;
    if (vis && vis->GeometryCount() > 0) {
      marker = std::dynamic_pointer_cast<gz::rendering::Marker>(vis->GeometryByIndex(0));
    }
    if (marker) {
      for (const auto& v : vertices) marker->AddPoint(gz::math::Vector3d(v.x, v.y, v.z), gz::math::Color::Black);
      return;
    }
    if (vis) scene->DestroyVisual(vis, true);
    vis = scene->CreateVisual(name);
    vis->AddGeometry(Marker(scene, material, transparent, vertices));
    parent->AddChild(vis);
  }

  void DestroyChunk(const gz::rendering::ScenePtr& scene, uint64_t id, const Built& built) {
    if (auto vis = scene->VisualByName(ChunkName(id))) scene->DestroyVisual(vis, true);
    for (const auto& m : built.materials) {
      if (scene->MaterialRegistered(m)) scene->DestroyMaterial(scene->Material(m));
    }
  }

  std::string prefix_;
  Params p_;
  std::mutex ground_mutex_;
  std::optional<Ground> ground_;
  // Rendering thread only.
  std::future<std::shared_ptr<const Surface>> loading_;
  std::shared_ptr<const Surface> surface_;
  Drawn drawn_;
  std::map<uint64_t, Built> built_;
  std::atomic<uint64_t> frames_{0}, built_records_{0}, held_frames_{0}, visuals_{0}, nanoseconds_{0},
      max_nanoseconds_{0};
};

/// <tracks> of the drivetrain plugin as Params (missing elements keep the defaults).
inline Params ReadParams(const sdf::ElementConstPtr& e, double radius) {
  Params p;
  p.radius = radius;
  const auto get = [&](const char* key, double fallback) { return e->Get<double>(key, fallback).first; };
  p.spacing = get("spacing", p.spacing);
  p.chunk = int(get("chunk", p.chunk));
  p.max_segments = int(get("max_segments", p.max_segments));
  p.berm_base = get("berm_base", p.berm_base);
  p.berm_gain = get("berm_gain", p.berm_gain);
  p.berm_max = get("berm_max", p.berm_max);
  p.berm_min = get("berm_min", p.berm_min);
  p.opacity_base = get("opacity_base", p.opacity_base);
  p.opacity_dig = get("opacity_dig", p.opacity_dig);
  p.floor_offset = get("floor_offset", p.floor_offset);
  p.pit_dig = get("pit_dig", p.pit_dig);
  p.smear_slip = get("smear_slip", p.smear_slip);
  p.width = get("width", p.width);
  p.jump = get("jump", p.jump);
  p.frame_budget = int(get("frame_budget", p.frame_budget));
  p.stress = int(get("stress", p.stress));
  p.report = get("report", p.report);
  return p;
}

/// The ruts as the drivetrain runs them: the layer it feeds, the renderer and the hook. Server thread, except
/// the hook.
class TrackSystem {
 public:
  TrackSystem(const sdf::ElementConstPtr& tracks, double radius, const std::string& model, size_t wheels,
              gz::sim::EventManager& events)
      : layer_(ReadParams(tracks, radius), wheels), renderer_(model + "/tracks", layer_.params()),
        report_(layer_.params().report) {
    // Connected once, here (EventT::Connect is not thread-safe). SceneUpdate does not make the Sensors system
    // render every step (only PreRender, Render and PostRender connections do).
    connection_ = events.Connect<gz::sim::events::SceneUpdate>([this] { renderer_.OnSceneUpdate(layer_); });
  }

  /// At the first step (every model is in the ECM by then): where the ground is drawn.
  void FindGround(const gz::sim::EntityComponentManager& ecm) { renderer_.SetGround(tracks::FindGround(ecm)); }

  void Step(std::chrono::steady_clock::duration sim_time, const std::vector<WheelInput>& wheels) {
    const Stamp t = std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count();
    const auto start = report_ > 0 ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point();
    if (!stressed_) {
      stressed_ = true;
      if (layer_.params().stress > 0 && !wheels.empty()) {
        layer_.Stress(t, wheels[0].x, wheels[0].y, wheels[0].z, layer_.params().stress);
      }
    }
    layer_.Step(t, wheels.data(), wheels.size());
    if (report_ <= 0) return;
    const double spent = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    step_seconds_ += spent;
    step_max_ = std::max(step_max_, spent);
    ++steps_;
    if (1e-9 * double(t) - last_report_ >= report_) {
      last_report_ = 1e-9 * double(t);
      const auto r = renderer_.stats();
      std::fprintf(stderr,
                   "RoverDrivetrain tracks at %.1f s: %zu records in %zu chunks; step %.3f us (max %.1f since the "
                   "last line) over %llu; render %llu frames %.3f ms (max %.2f since the last line), %llu records "
                   "built, %llu frames held some back; %llu scene visuals\n",
                   1e-9 * double(t), layer_.Records(), layer_.Chunks(), 1e6 * step_seconds_ / double(steps_),
                   1e6 * step_max_, (unsigned long long)steps_, (unsigned long long)r.frames,
                   r.frames ? 1e3 * r.seconds / double(r.frames) : 0.0, 1e3 * r.max_seconds,
                   (unsigned long long)r.records, (unsigned long long)r.held, (unsigned long long)r.visuals);
      step_max_ = 0.0;
    }
  }

  void Reset(std::chrono::steady_clock::duration sim_time) {
    layer_.Reset(std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count());
    stressed_ = false;
    last_report_ = -1e300;
  }

 private:
  TrackLayer layer_;
  Renderer renderer_;
  gz::common::ConnectionPtr connection_;
  bool stressed_ = false;
  double report_ = 0.0, last_report_ = -1e300, step_seconds_ = 0.0, step_max_ = 0.0;
  uint64_t steps_ = 0;
};

}  // namespace rover_sim::tracks
