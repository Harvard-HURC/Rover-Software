// Ruts and pits behind the rover's wheels (dig-in made visible, user decisions 2026-10-07): the part that needs no
// Gazebo. The drivetrain (rover_drivetrain.cpp) feeds a TrackLayer from its own per-wheel state at the end of every
// step; the rendering side (rover_tracks_render.hh) takes what is due on the Sensors system's rendering thread and
// draws it with BuildTrack() and BuildPit(). Visual only: nothing here is read back by the physics.
//
// Laying (server thread, TrackLayer::Step). A wheel touching soft ground (the terrain heightmap or a plane, whose
// ground has a static sinkage s > 0) lays a cross-section every <spacing> of its travel: the ground point under its
// hub, the lateral direction of travel, the half-width it swept, its dig factor D, s, whether it slipped and its
// heading. Two cross-sections of one wheel make a track segment. A wheel that barely moves (less than <spacing>)
// while D grows by <pit_dig> digs a pit at its last cross-section. A pit dug inside a standing one (within its
// rim's crest: the same place deeper, or a wheel that scrubbed or crept while digging) replaces it, swept from
// where the old one's hole began: one hole, never rims across each other's floors. A track segment laid while a pit
// stands draws no berm on a side where it would lie over that pit (inside its rim's crest): a wheel backing out of
// its pit, or another wheel crossing or passing it, lays no berms across the hole; the floor stays (review
// 2026-10-07: on the proving ground's dig-and-back-out drive the berms of the tracks leaving the pits and a second,
// shallower pit's rim covered 23-66 % of the pits' floors from above, now 0-8 %: only tracks laid before the pit was
// dug keep theirs). Records (segments and pits) go into chunks of <chunk>, a ring of at most <max_segments> records:
// the oldest chunk goes first (8000: about 100 m of the rover's travel, 4 wheels x 20 cross-sections per metre).
//
// Determinism (the snapshot rule). The rendering thread runs a frame's SceneUpdate while the next step's PreUpdate lays
// more records (gz-sim's Sensors::PostUpdate starts the frame and returns; measured: one frame in ten, driving on soft
// ground, found records laid after its time). So every record carries the sim time of the step that laid it, a pit
// replaced by a deeper one the time it was replaced, a chunk dropped from the ring the time it was dropped, and a frame
// draws exactly the records visible at its own sim time (the scene's time): stamp <= time < until, from the chunks in
// the ring at that time (the last kRetired = 4 chunks dropped stay readable for frames of an earlier time).
// TrackLayer::Take() copies only those not drawn yet (plain data, under the layer's mutex); the geometry is built
// outside it. A reset starts a new generation: a frame of the old one still rendering then (its time beyond the
// reset's) draws nothing new; gz-sim 8.10 publishes no image of such a frame at its own time (the Sensors system, a
// world plugin, resets before the drivetrain and sets the time its frame is published at to the reset's).
//
// Looks (s: the ground's static sinkage, D: the dig factor; the wheel already sits s into the visual surface, the
// static carve of the collision heightmap):
// - floor: a transparent dark overlay <floor_offset> above the visual surface over the swept strip, opacity
//   (opacity_base + opacity_dig (D - 1)) clamp(s / 2 cm, 0.6, 1.5), quantised to 1 - 0.85^k (k <= 4: 15, 28, 39,
//   48 %); every other cross-section one level lighter (tread marks) unless the wheel slipped over <smear_slip> of
//   its surface speed. It writes no depth: depth cameras and point clouds do not see it.
// - berms: a ridge each side, crest h = berm_gain s (max(D - 1, 0) + berm_base), at most <berm_max>, drawn from
//   <berm_min>: sand 6 mm at D = 1 and 2.6 cm at D = 2, wash sand up to 3.9 cm, the sand sheet 4.5-8 mm (D up to
//   1.25); regolith, gravel and biocrust (s 5 mm), mudstone, bentonite and badland slopes (s 1 cm, D stays 1) only
//   the faint floor. Each berm is 4 cm + 3 h wide, its crest at a quarter of that, lumpy by +-35 % (smooth value
//   noise), its outer edge buried 1 cm: on the ground, never floating, so depth images only get closer. The
//   apparent depth, carve plus berm, is about s (D + 0.3), what D means.
// - width: half of W |cos t| + L |sin t|, t between the wheel's heading and its travel, L = 2 sqrt(2 r D s) the
//   tyre's chord at that depth: a wheel scrubbing sideways in a spin leaves a wide smear.
// - pits: a rim at crest height round the wheel's chord (a 16-sided superellipse), the dark floor inside; a pit
//   swept from an earlier one's place is that shape stretched back to it.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

namespace rover_sim::tracks {

/// Sim time in nanoseconds: what UpdateInfo::simTime and gz::rendering::Scene::Time() count.
using Stamp = int64_t;
constexpr Stamp kNever = std::numeric_limits<Stamp>::max();

/// The <tracks> group of the drivetrain plugin (gen_model.TrackParams writes it; units SI).
struct Params {
  double spacing = 0.05;        ///< [m] of a wheel's travel per cross-section
  int chunk = 128;              ///< records per chunk
  int max_segments = 8000;      ///< records in the ring
  double berm_base = 0.3;       ///< crest = berm_gain s (max(D - 1, 0) + berm_base)
  double berm_gain = 1.0;
  double berm_max = 0.06;       ///< [m]
  double berm_min = 0.004;      ///< [m] lower crests are not drawn
  double opacity_base = 0.14;   ///< floor opacity at D = 1 on 2 cm sinkage
  double opacity_dig = 0.30;    ///< per unit of D - 1
  double floor_offset = 0.004;  ///< [m] above the visual surface
  double pit_dig = 0.05;        ///< D grown in place that digs a pit
  double smear_slip = 0.3;      ///< slip over surface speed above which tread marks smear
  double radius = 0.15;         ///< [m] wheel
  double width = 0.10;          ///< [m] wheel
  double jump = 1.0;            ///< [m] a wheel farther than this from its last cross-section starts a new track
  int frame_budget = 256;       ///< records a frame builds at most (a late viewer catches up over frames)
  int stress = 0;               ///< synthetic records laid at the first step (cost measurements)
  double report = 0.0;          ///< [s] of sim time between cost lines on stderr; 0: none (measurements)
};

constexpr double kPi = 3.14159265358979323846;
constexpr int kLevels = 5;              ///< floor darkness levels; 0 draws nothing
constexpr double kLayerOpacity = 0.15;  ///< level k: opacity 1 - (1 - kLayerOpacity)^k
constexpr double kSharpTurn = 0.7071;   ///< cos 45 deg: a sharper turn between cross-sections starts a corner
/// Chunks dropped from the ring that frames of an earlier time still read (a frame lags its step by one step, rarely
/// a few; four chunks of 128 records are at least 128 steps of four wheels, seconds of driving in practice).
constexpr size_t kRetired = 4;

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

/// One cross-section of a wheel's track.
struct Section {
  double x = 0.0, y = 0.0;  ///< ground point under the hub [m]
  double z = 0.0;           ///< its height by the contact [m]: used only where no heightmap is drawn (a plane)
  double lx = 0.0, ly = 1.0;  ///< unit, horizontal, left of the travel
  double hx = 1.0, hy = 0.0;  ///< the wheel's heading, unit, horizontal
  double half = 0.05;       ///< swept half-width [m]
  double dig = 1.0;         ///< D
  double sink = 0.0;        ///< the ground's static sinkage s [m]
  uint32_t index = 0;       ///< count along the track (tread marks)
  bool smeared = false;     ///< the wheel slipped here: no tread marks
};

/// A record: a stretch of one wheel's track (a to b), or a pit where the wheel dug in place (at b; a is where its
/// hole began, b's place unless it swept an earlier pit's: a's shape is b's).
struct Segment {
  Section a, b;
  Stamp stamp = 0;       ///< the step that laid it
  Stamp until = kNever;  ///< a pit dug deeper since: the step that replaced it
  uint8_t wheel = 0;
  uint8_t ground = 0;    ///< the ground's id (the drivetrain's numbering): berms of one ground share a colour
  bool pit = false;
  uint8_t bermless = 0;  ///< a track segment's sides (1 left, 2 right) whose berm would lie over a standing pit

  bool Visible(Stamp t) const { return stamp <= t && t < until; }
};

/// What a wheel tells the layer each step.
struct WheelInput {
  bool soil = false;      ///< touching ground the ruts mark: the terrain heightmap or a plane, sinkage > 0
  double x = 0.0, y = 0.0, z = 0.0;  ///< ground point under the hub [m]
  double hx = 1.0, hy = 0.0;         ///< the wheel's heading (horizontal part; normalised here)
  double sink = 0.0;      ///< the ground's static sinkage [m]
  double dig = 1.0;       ///< D
  double slip = 0.0;      ///< [m/s] the contact's slip speed
  double surface = 0.0;   ///< [m/s] the tyre's surface speed, |w| r
  uint8_t ground = 0;
};

/// A drawn pit: chunk id and record index.
struct PitRef {
  uint64_t chunk = 0;
  uint32_t index = 0;
};

/// The rendering side's account of what it took from a layer (TrackLayer::Take keeps it).
struct Drawn {
  uint64_t generation = 0;
  std::map<uint64_t, size_t> taken;  ///< chunk id -> records taken (a prefix)
  std::vector<PitRef> pits;          ///< pits drawn and still standing
};

/// What a frame has to change (TrackLayer::Take).
struct Update {
  bool stale = false;               ///< a frame of an earlier generation: change nothing
  bool reset = false;               ///< a new generation: remove everything drawn first
  std::vector<uint64_t> dropped;    ///< chunks gone from the ring
  std::vector<PitRef> dead;         ///< drawn pits replaced by deeper ones
  struct Batch {
    uint64_t chunk = 0;
    std::vector<uint32_t> index;    ///< of each record in its chunk
    std::vector<Segment> records;   ///< in order: track segments, and the pits standing at the frame's time
  };
  std::vector<Batch> batches;       ///< new records, newest chunk first
  size_t taken = 0;                 ///< records taken (drawn or skipped)
  size_t held = 0;                  ///< records laid already but due after the frame's time (the next steps')
};

// --- Laying ----------------------------------------------------------------------------------------------------

/// A cross-section at (x, y, z): lateral from the travel (the heading if there is none), half-width from the
/// angle between the two (the notes at the top).
inline Section MakeSection(const Params& p, double x, double y, double z, double tx, double ty, double hx, double hy,
                           double dig, double sink) {
  Section s;
  s.x = x;
  s.y = y;
  s.z = z;
  double hl = std::hypot(hx, hy);
  if (hl < 1e-9) {
    hx = 1.0;
    hy = 0.0;
    hl = 1.0;
  }
  hx /= hl;
  hy /= hl;
  double tl = std::hypot(tx, ty);
  if (tl < 1e-9) {
    tx = hx;
    ty = hy;
    tl = 1.0;
  }
  tx /= tl;
  ty /= tl;
  s.lx = -ty;
  s.ly = tx;
  s.hx = hx;
  s.hy = hy;
  const double cos_t = std::min(1.0, std::abs(hx * tx + hy * ty));
  const double sin_t = std::sqrt(std::max(0.0, 1.0 - cos_t * cos_t));
  const double chord = 2.0 * std::sqrt(std::max(0.0, 2.0 * p.radius * dig * sink));
  s.half = 0.5 * (p.width * cos_t + chord * sin_t);
  s.dig = dig;
  s.sink = sink;
  return s;
}

/// The berm's crest above the visual surface [m].
inline double BermHeight(const Params& p, const Section& s) {
  return std::min(p.berm_max, p.berm_gain * s.sink * (std::max(0.0, s.dig - 1.0) + p.berm_base));
}

/// A pit's shape round its cross-section c (BuildPit): its floor reaches `along` the heading and `across` it, the
/// rim `rim` wide beyond, its crest at a quarter of that, `crest` high. The wheel, the static sinkage into the
/// visual surface, meets the crest at a half-chord along its heading.
struct PitShape {
  double along = 0.0, across = 0.0, rim = 0.0, crest = 0.0;
};
inline PitShape ShapeOfPit(const Params& p, const Section& c) {
  PitShape s;
  s.crest = BermHeight(p, c);
  const double y = c.sink + s.crest;  // the crest above the wheel's lowest point
  s.along = std::sqrt(std::max(0.0, 2 * p.radius * y - y * y)) + 0.01;
  s.across = std::max(c.half, p.width / 2) + 0.005;
  s.rim = 0.04 + 3.0 * s.crest;
  return s;
}

/// Whether (x, y) lies inside a pit's rim crest: the superellipse round the nearest point of its sweep (from a to
/// b), grown by a quarter of the rim.
inline bool InsidePit(const Params& p, const Segment& pit, double x, double y) {
  const Section& c = pit.b;
  const PitShape s = ShapeOfPit(p, c);
  const double sx = pit.a.x - c.x, sy = pit.a.y - c.y, l2 = sx * sx + sy * sy;
  const double f = l2 > 0 ? std::clamp(((x - c.x) * sx + (y - c.y) * sy) / l2, 0.0, 1.0) : 0.0;
  const double dx = x - c.x - f * sx, dy = y - c.y - f * sy;
  const double u = (dx * c.hx + dy * c.hy) / (s.along + s.rim / 4);
  const double v = (dy * c.hx - dx * c.hy) / (s.across + s.rim / 4);
  return u * u * u * u + v * v * v * v < 1.0;
}

/// The ruts' record of every wheel: laid by the server thread (Step, Reset, Stress), taken by the rendering
/// thread (Take). Every public call locks.
class TrackLayer {
 public:
  explicit TrackLayer(const Params& p = Params(), size_t wheels = 4) : p_(p), runs_(wheels) {
    p_.chunk = std::max(8, p_.chunk);
    p_.max_segments = std::max(2 * p_.chunk, p_.max_segments);
  }

  const Params& params() const { return p_; }

  /// One step at sim time t: each wheel lays a cross-section, deepens a pit or nothing.
  void Step(Stamp t, const WheelInput* wheels, size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    now_ = t;
    for (size_t k = 0; k < count && k < runs_.size(); ++k) Lay(t, k, wheels[k]);
  }

  /// A world reset at sim time t: everything goes; a new generation starts.
  void Reset(Stamp t) {
    std::lock_guard<std::mutex> lock(mutex_);
    chunks_.clear();
    retired_.clear();
    holes_.clear();
    total_ = 0;
    for (auto& r : runs_) r = Run();
    ++generation_;
    now_ = t;
  }

  /// `count` synthetic track segments in rows of 10 m round (x, y, z) on ground of 2 cm sinkage, D from 1 to 2
  /// (cost measurements; Params::stress).
  void Stress(Stamp t, double x, double y, double z, int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int per_row = 200;
    const int rows = (count + per_row - 1) / per_row;
    for (int i = 0; i < count; ++i) {
      const int row = i / per_row, col = i % per_row;
      const double x0 = x - 5.0 + col * p_.spacing, y0 = y - 0.4 - 0.175 * (rows - 1) + 0.35 * row;
      const double dig = 1.0 + (row % 5) * 0.25;
      Segment s;
      s.a = MakeSection(p_, x0, y0, z, 1, 0, 1, 0, dig, 0.02);
      s.b = MakeSection(p_, x0 + p_.spacing, y0, z, 1, 0, 1, 0, dig, 0.02);
      s.a.index = uint32_t(col);
      s.b.index = uint32_t(col + 1);
      s.stamp = t;
      Append(s);
    }
  }

  /// What a frame at sim time t draws that `drawn` has not: the records visible at t (stamp <= t < until) in the
  /// chunks of the ring at t, newest chunk first, at most `budget` of them; and what to remove. Updates `drawn`.
  Update Take(Stamp t, Drawn& drawn, size_t budget) const {
    std::lock_guard<std::mutex> lock(mutex_);
    Update u;
    if (drawn.generation != generation_) {
      if (t > now_) {
        u.stale = true;  // rendering the old generation while the layer was reset
        return u;
      }
      u.reset = true;
      drawn = Drawn();
      drawn.generation = generation_;
    }
    const Ring ring = At(t);
    const uint64_t first = ring.size() ? ring[0].id : next_id_;
    for (auto it = drawn.taken.begin(); it != drawn.taken.end() && it->first < first;) {
      u.dropped.push_back(it->first);
      it = drawn.taken.erase(it);
    }
    for (auto it = drawn.pits.begin(); it != drawn.pits.end();) {
      const Segment* s = ring.Find(it->chunk, it->index);
      if (!s) {
        it = drawn.pits.erase(it);  // its chunk is gone
      } else if (s->until <= t) {
        u.dead.push_back(*it);
        it = drawn.pits.erase(it);
      } else {
        ++it;
      }
    }
    size_t left = budget;
    for (size_t k = ring.size(); k-- > 0 && left > 0;) {
      const Chunk& c = ring[k];
      size_t& taken = drawn.taken[c.id];
      size_t n = c.records.size();
      while (n > 0 && c.records[n - 1].stamp > t) --n;  // stamps never decrease: only the newest can be due later
      u.held += c.records.size() - n;
      if (n <= taken) continue;
      const size_t end = taken + std::min(n - taken, left);
      Update::Batch batch;
      batch.chunk = c.id;
      for (size_t i = taken; i < end; ++i) {
        const Segment& s = c.records[i];
        if (s.pit) {
          if (!s.Visible(t)) continue;  // replaced already: never drawn
          drawn.pits.push_back({c.id, uint32_t(i)});
        }
        batch.index.push_back(uint32_t(i));
        batch.records.push_back(s);
      }
      u.taken += end - taken;
      left -= end - taken;
      taken = end;
      if (!batch.records.empty()) u.batches.push_back(std::move(batch));
    }
    return u;
  }

  size_t Records() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_;
  }
  size_t Chunks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return chunks_.size();
  }
  uint64_t Generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }
  /// Every record, oldest first (tests).
  std::vector<Segment> All() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Segment> out;
    for (const auto& c : chunks_) out.insert(out.end(), c.records.begin(), c.records.end());
    return out;
  }

 private:
  struct Chunk {
    uint64_t id = 0;
    std::vector<Segment> records;
  };
  /// A chunk dropped from the ring, and the step that dropped it.
  struct Retired {
    Stamp stamp = 0;
    Chunk chunk;
  };
  /// The chunks a frame sees, oldest first (ids consecutive): the ring's, after those dropped by later steps.
  struct Ring {
    const std::deque<Retired>& retired;
    const std::deque<Chunk>& chunks;
    size_t old = 0;  // retired chunks seen

    size_t size() const { return old + chunks.size(); }
    const Chunk& operator[](size_t i) const {
      return i < old ? retired[retired.size() - old + i].chunk : chunks[i - old];
    }
    const Segment* Find(uint64_t chunk, uint32_t index) const {
      if (!size() || chunk < (*this)[0].id || chunk > (*this)[size() - 1].id) return nullptr;
      const Chunk& c = (*this)[size_t(chunk - (*this)[0].id)];
      return index < c.records.size() ? &c.records[index] : nullptr;
    }
  };
  /// A wheel's track being laid.
  struct Run {
    bool active = false;
    Section last;            // the last cross-section (deepened in place by pits)
    double mark_dig = 1.0;   // D when the last cross-section or pit was made
  };
  /// A pit standing in the ring (not replaced), and how far from the middle of its sweep its rim's crest reaches.
  struct Hole {
    PitRef ref;
    Segment pit;
    double reach = 0.0;
  };

  /// The ring as a frame at time t sees it.
  Ring At(Stamp t) const {
    size_t old = 0;
    while (old < retired_.size() && retired_[retired_.size() - 1 - old].stamp > t) ++old;
    return {retired_, chunks_, old};
  }

  /// The sides of a track segment whose berm would lie over a standing pit: its foot, a point on its slope or its
  /// crest, at either end or in the middle, inside a pit's rim crest. Leaving a pit, or crossing one, a wheel draws
  /// no berm across the hole (the floor stays); passing beside one, only the berm on that side goes.
  uint8_t OverHoles(const Segment& s) const {
    uint8_t sides = 0;
    const double mx = (s.a.x + s.b.x) / 2, my = (s.a.y + s.b.y) / 2;
    const double span = 0.5 * std::hypot(s.b.x - s.a.x, s.b.y - s.a.y) + std::max(s.a.half, s.b.half) +
                        0.45 * (0.04 + 3.0 * p_.berm_max);  // how far its berms reach from its middle
    for (const Hole& h : holes_) {
      const double hx = (h.pit.a.x + h.pit.b.x) / 2, hy = (h.pit.a.y + h.pit.b.y) / 2;
      if (std::hypot(mx - hx, my - hy) > h.reach + span) continue;
      for (const auto& [bit, side] : {std::pair<uint8_t, double>{1, 1.0}, {2, -1.0}}) {
        for (const double f : {0.0, 0.5, 1.0}) {
          if (sides & bit) break;
          for (const double reach : {0.0, 0.25, 0.45}) {  // the foot, the crest, the slope beyond it
            double x = 0.0, y = 0.0;
            for (const auto& [c, weight] : {std::pair<const Section*, double>{&s.a, 1.0 - f}, {&s.b, f}}) {
              const double out = c->half + reach * (0.04 + 3.0 * BermHeight(p_, *c));
              x += weight * (c->x + side * c->lx * out);
              y += weight * (c->y + side * c->ly * out);
            }
            if (InsidePit(p_, h.pit, x, y)) {
              sides |= bit;
              break;
            }
          }
        }
      }
    }
    return sides;
  }

  void Lay(Stamp t, size_t wheel, const WheelInput& in) {
    Run& r = runs_[wheel];
    if (!in.soil || !(in.sink > 0.0)) {
      r.active = false;
      return;
    }
    if (!r.active) {
      r = Run();
      r.active = true;
      r.last = MakeSection(p_, in.x, in.y, in.z, in.hx, in.hy, in.hx, in.hy, in.dig, in.sink);
      r.mark_dig = in.dig;
      return;
    }
    const double dx = in.x - r.last.x, dy = in.y - r.last.y;
    const double d = std::hypot(dx, dy);
    if (d > p_.jump) {  // a respawn or a jump: a new track starts at the next step
      r.active = false;
      return;
    }
    if (d >= p_.spacing) {
      Section s = MakeSection(p_, in.x, in.y, in.z, dx, dy, in.hx, in.hy, in.dig, in.sink);
      s.smeared = in.slip > p_.smear_slip * std::max(0.05, in.surface);
      s.index = r.last.index + 1;
      Segment seg;
      seg.a = r.last;
      seg.b = s;
      // A sharp turn (a wheel reversing, or jinking as it swings round in a spin) would twist the strip between
      // cross-sections whose laterals differ by more than 45 deg into a bow-tie of slivers: the segment starts
      // square to its own travel instead (a corner, overlapping the last one).
      if (seg.a.lx * s.lx + seg.a.ly * s.ly < kSharpTurn) {
        seg.a.lx = s.lx;
        seg.a.ly = s.ly;
      }
      seg.stamp = t;
      seg.wheel = uint8_t(wheel);
      seg.ground = in.ground;
      seg.bermless = OverHoles(seg);
      Append(seg);
      r.last = s;
      r.mark_dig = in.dig;
      return;
    }
    if (in.dig >= r.mark_dig + p_.pit_dig) {  // digging in place: a pit at the last cross-section, deeper each time
      const Section here = MakeSection(p_, in.x, in.y, in.z, in.hx, in.hy, in.hx, in.hy, in.dig, in.sink);
      r.last.dig = std::max(r.last.dig, in.dig);
      r.last.half = std::max(r.last.half, here.half);
      r.last.hx = here.hx;
      r.last.hy = here.hy;
      r.last.sink = std::max(r.last.sink, in.sink);
      Segment seg;
      seg.b = r.last;
      Section start = r.last;
      double half = here.half;
      // Inside a standing pit (this one, dug deeper; one the wheel scrubbed or crept out of while digging; another
      // wheel's): that one goes, and this one is swept from where its hole began, at least as deep and as wide. A
      // swept pit is the wheel's own width (the sweep is the scrub that widened the last cross-section).
      for (auto h = holes_.begin(); h != holes_.end();) {
        if (!InsidePit(p_, h->pit, r.last.x, r.last.y)) {
          ++h;
          continue;
        }
        if (Segment* old = Find(h->ref.chunk, h->ref.index)) old->until = t;
        start = h->pit.a;
        half = std::max(half, h->pit.b.half);
        seg.b.dig = std::max(seg.b.dig, h->pit.b.dig);
        seg.b.sink = std::max(seg.b.sink, h->pit.b.sink);
        h = holes_.erase(h);
      }
      if (start.x != r.last.x || start.y != r.last.y) seg.b.half = half;
      seg.a = seg.b;
      seg.a.x = start.x;
      seg.a.y = start.y;
      seg.a.z = start.z;
      seg.stamp = t;
      seg.wheel = uint8_t(wheel);
      seg.ground = in.ground;
      seg.pit = true;
      const PitRef ref = Append(seg);
      const PitShape shape = ShapeOfPit(p_, seg.b);
      const double reach = 0.5 * std::hypot(seg.a.x - seg.b.x, seg.a.y - seg.b.y) +
                           std::hypot(shape.along + shape.rim / 4, shape.across + shape.rim / 4);
      holes_.push_back({ref, seg, reach});
      r.mark_dig = in.dig;
    }
  }

  PitRef Append(const Segment& s) {
    if (chunks_.empty() || chunks_.back().records.size() >= size_t(p_.chunk)) {
      while (!chunks_.empty() && total_ + size_t(p_.chunk) > size_t(p_.max_segments)) {
        const uint64_t id = chunks_.front().id;
        holes_.erase(std::remove_if(holes_.begin(), holes_.end(), [&](const Hole& h) { return h.ref.chunk == id; }),
                     holes_.end());
        total_ -= chunks_.front().records.size();
        retired_.push_back({s.stamp, std::move(chunks_.front())});  // frames before this step still see it
        chunks_.pop_front();
        if (retired_.size() > kRetired) retired_.pop_front();
      }
      Chunk c;
      c.id = next_id_++;
      c.records.reserve(size_t(p_.chunk));
      chunks_.push_back(std::move(c));
    }
    Chunk& c = chunks_.back();
    c.records.push_back(s);
    ++total_;
    return {c.id, uint32_t(c.records.size() - 1)};
  }

  /// A record in the ring as it stands now.
  Segment* Find(uint64_t chunk, uint32_t index) {
    return const_cast<Segment*>(Ring{retired_, chunks_, 0}.Find(chunk, index));
  }

  Params p_;
  mutable std::mutex mutex_;
  std::deque<Chunk> chunks_;  // ids consecutive; they keep counting across resets, so names never clash
  std::deque<Retired> retired_;  // the last kRetired chunks dropped, oldest first
  std::vector<Hole> holes_;      // the pits standing, oldest first
  size_t total_ = 0;
  uint64_t next_id_ = 1;
  uint64_t generation_ = 0;
  Stamp now_ = 0;             // the last step's time
  std::vector<Run> runs_;
};

// --- Geometry ------------------------------------------------------------------------------------------------

/// The darkness level of the floor at a cross-section (0: none).
inline int FloorLevel(const Params& p, const Section& s) {
  if (!(s.sink > 0.0)) return 0;
  const double o = std::clamp(
      (p.opacity_base + p.opacity_dig * (s.dig - 1.0)) * std::clamp(s.sink / 0.02, 0.6, 1.5), 0.0, 0.62);
  return std::clamp(int(std::lround(std::log(1.0 - o) / std::log(1.0 - kLayerOpacity))), 0, kLevels - 1);
}

/// A deterministic number in [-1, 1] for a grid cell and a channel.
inline double Hash(int64_t i, int64_t j, double channel) {
  uint64_t z = uint64_t(i) * 0x9E3779B97F4A7C15ULL ^ uint64_t(j) * 0xC2B2AE3D27D4EB4FULL ^
               uint64_t(std::llround(channel * 16 + 64)) * 0x165667B19E3779F9ULL;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  z ^= z >> 31;
  return double(z >> 11) / double(1ULL << 53) * 2.0 - 1.0;
}

/// Smooth deterministic noise in [-1, 1]: value noise on a 0.25 m grid (white noise per cross-section made a
/// sawtooth crest at grazing angles).
inline double Lumps(double x, double y, double channel) {
  constexpr double kCell = 0.25;
  const double u = x / kCell, v = y / kCell;
  const double fu = std::floor(u), fv = std::floor(v), su = u - fu, sv = v - fv;
  const auto at = [&](double i, double j) { return Hash(int64_t(fu + i), int64_t(fv + j), channel); };
  const double wu = su * su * (3 - 2 * su), wv = sv * sv * (3 - 2 * sv);
  return (1 - wv) * ((1 - wu) * at(0, 0) + wu * at(1, 0)) + wv * ((1 - wu) * at(0, 1) + wu * at(1, 1));
}

/// Height of the visual surface at (x, y); `fallback` where there is none (a plane: the contact's height).
using HeightFn = std::function<double(double x, double y, double fallback)>;

/// Triangle lists, counter-clockwise seen from above (the normals point up).
struct Mesh {
  std::array<std::vector<Vec3>, kLevels> floor;  ///< per darkness level (index 0 unused)
  std::vector<Vec3> berm;

  void Clear() {
    for (auto& f : floor) f.clear();
    berm.clear();
  }
  size_t Vertices() const {
    size_t n = berm.size();
    for (const auto& f : floor) n += f.size();
    return n;
  }
};

namespace detail {

inline void Quad(std::vector<Vec3>& out, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
  out.insert(out.end(), {a, b, c, a, c, d});
}

/// The point `along` the cross-section's lateral, `dz` above the surface.
inline Vec3 Across(const HeightFn& h, const Section& s, double along, double dz) {
  const double x = s.x + s.lx * along, y = s.y + s.ly * along;
  return {x, y, h(x, y, s.z) + dz};
}

}  // namespace detail

/// A track segment's floor and berms, appended to `out` (the notes at the top).
inline void BuildTrack(const Params& p, const Segment& s, const HeightFn& h, Mesh& out) {
  using detail::Across;
  using detail::Quad;
  const Section& a = s.a;
  const Section& b = s.b;
  int level = std::max(FloorLevel(p, a), FloorLevel(p, b));
  if (!b.smeared && (b.index & 1u) && level > 1) --level;  // tread marks
  if (level > 0) {
    auto& f = out.floor[size_t(level)];
    Quad(f, Across(h, a, -a.half, p.floor_offset), Across(h, b, -b.half, p.floor_offset),
         Across(h, b, b.half, p.floor_offset), Across(h, a, a.half, p.floor_offset));
  }
  const double ha = BermHeight(p, a), hb = BermHeight(p, b);
  if (std::max(ha, hb) < p.berm_min) return;
  const double wa = 0.04 + 3.0 * ha, wb = 0.04 + 3.0 * hb;  // crest at a quarter, gentle outside
  for (const double side : {1.0, -1.0}) {
    if (s.bermless & (side > 0 ? 1 : 2)) continue;  // it would lie over a standing pit
    // Lumpy, not extruded: crest height and place, and the outer edge, vary with smooth noise over the ground (a
    // cross-section shared by two segments gets the same vertices).
    const double na = Lumps(a.x, a.y, side), nb = Lumps(b.x, b.y, side);
    const Vec3 ai = Across(h, a, side * a.half, p.floor_offset);
    const Vec3 bi = Across(h, b, side * b.half, p.floor_offset);
    const Vec3 ak = Across(h, a, side * (a.half + wa * (0.25 + 0.08 * na)), ha * (1 + 0.35 * na));
    const Vec3 bk = Across(h, b, side * (b.half + wb * (0.25 + 0.08 * nb)), hb * (1 + 0.35 * nb));
    const Vec3 ao = Across(h, a, side * (a.half + wa * (1 + 0.3 * Lumps(a.x, a.y, 3 * side))), -0.01);
    const Vec3 bo = Across(h, b, side * (b.half + wb * (1 + 0.3 * Lumps(b.x, b.y, 3 * side))), -0.01);
    if (side > 0) {
      Quad(out.berm, ai, bi, bk, ak);
      Quad(out.berm, ak, bk, bo, ao);
    } else {
      Quad(out.berm, ak, bk, bi, ai);
      Quad(out.berm, ao, bo, bk, ak);
    }
  }
}

/// A pit's rim and dark floor, appended to `out` (ShapeOfPit). A pit swept from an earlier one's place (s.a) is the
/// shape round b stretched back there: each point whose outward normal faces s.a moves by b to s.a.
inline void BuildPit(const Params& p, const Segment& s, const HeightFn& h, Mesh& out) {
  using detail::Quad;
  const Section& c = s.b;
  const PitShape shape = ShapeOfPit(p, c);
  const double a = shape.along, wi = shape.across, b = shape.rim, crest = shape.crest;
  const int level = FloorLevel(p, c);
  const double tx = c.hx, ty = c.hy, nx = -c.hy, ny = c.hx;
  const double sx = s.a.x - c.x, sy = s.a.y - c.y;              // the sweep, 0 for a pit dug where it stands
  const double su = sx * tx + sy * ty, sv = sx * nx + sy * ny;  // in the pit's frame
  constexpr int kSides = 16;
  // A rounded rectangle (superellipse, exponent 4) round the wheel's footprint, counter-clockwise from above.
  const auto at = [&](int k, double along, double across, double dz) {
    const double th = 2 * kPi * (k % kSides) / kSides, co = std::cos(th), si = std::sin(th);
    const double u = std::copysign(std::sqrt(std::abs(co)), co), v = std::copysign(std::sqrt(std::abs(si)), si);
    const bool back = (u * u * u / a) * su + (v * v * v / wi) * sv > 0;  // the floor's normal there faces s.a
    const double x = c.x + (back ? sx : 0.0) + tx * (u * along) + nx * (v * across);
    const double yy = c.y + (back ? sy : 0.0) + ty * (u * along) + ny * (v * across);
    return Vec3{x, yy, h(x, yy, back ? s.a.z : c.z) + dz};
  };
  if (crest >= p.berm_min) {
    for (int k = 0; k < kSides; ++k) {
      const Vec3 mk = at(k, a, wi, 0), mj = at(k + 1, a, wi, 0);
      const double nk = Lumps(mk.x, mk.y, 5.0), nj = Lumps(mj.x, mj.y, 5.0);
      const Vec3 ik = at(k, a, wi, p.floor_offset), ij = at(k + 1, a, wi, p.floor_offset);
      const Vec3 kk = at(k, a + b / 4, wi + b / 4, crest * (1 + 0.35 * nk));
      const Vec3 kj = at(k + 1, a + b / 4, wi + b / 4, crest * (1 + 0.35 * nj));
      const Vec3 ok = at(k, a + b, wi + b, -0.01), oj = at(k + 1, a + b, wi + b, -0.01);
      Quad(out.berm, kk, kj, ij, ik);
      Quad(out.berm, ok, oj, kj, kk);
    }
  }
  if (level > 0) {
    const double mx = c.x + sx / 2, my = c.y + sy / 2;
    const Vec3 centre{mx, my, h(mx, my, (c.z + s.a.z) / 2) + p.floor_offset};
    auto& f = out.floor[size_t(level)];
    for (int k = 0; k < kSides; ++k) {
      f.insert(f.end(), {centre, at(k, a, wi, p.floor_offset), at(k + 1, a, wi, p.floor_offset)});
    }
  }
}

}  // namespace rover_sim::tracks
