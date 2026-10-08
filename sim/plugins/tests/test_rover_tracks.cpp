// Unit tests of the ruts' layer and geometry (rover_tracks.hh): cross-section spacing, sharp turns, the pit rule,
// the ring, reset, the time-stamp snapshot a frame draws, and the floor, berm and pit shapes against the design's
// numbers.
#include <cmath>
#include <set>
#include <vector>

#include "check.hh"
#include "rover_tracks.hh"

using namespace rover_sim::tracks;

namespace {

constexpr Stamp kMs = 1000000;  // 1 ms in ns
constexpr double kSand = 0.02, kWashSand = 0.03, kSandSheet = 0.015, kRegolith = 0.005;

// The layer's tests use binary fractions, so its thresholds are met exactly: a cross-section every 1/16 m, a
// wheel moving 1/256 m per 1 ms step (one cross-section per 16 steps), a pit every 1/16 of D.
constexpr double kStep = 1.0 / 256;

Params Exact() {
  Params p;
  p.spacing = 1.0 / 16;
  p.pit_dig = 1.0 / 16;
  return p;
}

WheelInput At(double x, double y, double dig = 1.0, double sink = kSand, double hx = 1.0, double hy = 0.0) {
  WheelInput in;
  in.soil = true;
  in.x = x;
  in.y = y;
  in.hx = hx;
  in.hy = hy;
  in.sink = sink;
  in.dig = dig;
  in.surface = 0.5;
  return in;
}

/// One wheel driving along +x `kStep` per step for `steps` steps from x0, the first step at `from`; returns the
/// next step's time.
Stamp Drive(TrackLayer& layer, Stamp from, int steps, double x0, double dig = 1.0, double sink = kSand) {
  for (int k = 0; k < steps; ++k) {
    const WheelInput in = At(x0 + k * kStep, 0.0, dig, sink);
    layer.Step(from + k * kMs, &in, 1);
  }
  return from + steps * kMs;
}

/// A wheel standing at x digging in: D up by 1/64 per step for `steps` steps; returns the next step's time.
Stamp Dig(TrackLayer& layer, Stamp from, int steps, double x, double& dig) {
  for (int k = 0; k < steps; ++k) {
    dig = std::min(2.0, dig + 1.0 / 64);
    const WheelInput in = At(x, 0.0, dig);
    layer.Step(from + k * kMs, &in, 1);
  }
  return from + steps * kMs;
}

std::vector<Segment> Tracks(const TrackLayer& layer) {
  std::vector<Segment> out;
  for (const auto& s : layer.All())
    if (!s.pit) out.push_back(s);
  return out;
}
std::vector<Segment> Pits(const TrackLayer& layer) {
  std::vector<Segment> out;
  for (const auto& s : layer.All())
    if (s.pit) out.push_back(s);
  return out;
}
size_t PitsIn(const Update& u) {
  size_t n = 0;
  for (const auto& b : u.batches)
    for (const auto& s : b.records) n += s.pit;
  return n;
}

double Flat(double, double, double) { return 1.0; }  // the visual surface at z = 1

void TestSpacing() {
  CHECK_NEAR(Params().spacing, 0.05, 0);  // the default: 20 cross-sections per metre
  TrackLayer layer(Exact());
  Drive(layer, kMs, 101, 0.0);  // to 100/256 m
  const auto segs = Tracks(layer);
  CHECK(segs.size() == 6);  // a cross-section at the start, then one every 16 steps
  for (size_t i = 0; i < segs.size(); ++i) {
    CHECK_NEAR(segs[i].a.x, i / 16.0, 0);
    CHECK_NEAR(segs[i].b.x, (i + 1) / 16.0, 0);
    CHECK_NEAR(segs[i].b.ly, 1.0, 1e-12);  // left of +x travel
    CHECK(segs[i].b.index == segs[i].a.index + 1);
    CHECK(segs[i].stamp == (1 + 16 * Stamp(i + 1)) * kMs);  // laid by the step that reached it
    CHECK(segs[i].until == kNever && !segs[i].pit);
  }
  CHECK(Pits(layer).empty());
  if (!segs.empty()) CHECK_NEAR(segs[0].b.half, 0.05, 1e-12);  // rolling straight: the wheel's width
}

void TestOnlySoftGround() {
  TrackLayer rock(Exact());
  Drive(rock, kMs, 100, 0.0, 1.0, 0.0);  // sinkage 0
  CHECK(rock.Records() == 0);
  TrackLayer object(Exact());
  for (int k = 0; k < 100; ++k) {  // soft ground but not the soil (an object, a mesh of the terrain model)
    WheelInput in = At(k * kStep, 0.0);
    in.soil = false;
    object.Step(k * kMs, &in, 1);
  }
  CHECK(object.Records() == 0);
  // Leaving soft ground ends a track; coming back starts another, with no segment across the gap.
  TrackLayer layer(Exact());
  Stamp t = Drive(layer, kMs, 20, 0.0);           // 0 .. 19/256: one segment
  t = Drive(layer, t, 50, 20 * kStep, 1.0, 0.0);  // rock to 69/256
  Drive(layer, t, 20, 70 * kStep);                 // sand again from 70/256: one segment
  const auto segs = Tracks(layer);
  CHECK(segs.size() == 2);
  for (const auto& seg : segs) CHECK_NEAR(seg.b.x - seg.a.x, 1.0 / 16, 0);
  if (segs.size() == 2) CHECK_NEAR(segs[1].a.x, 70 * kStep, 0);
  // A jump of more than a metre (a respawn) starts a new track too.
  TrackLayer jump(Exact());
  t = Drive(jump, kMs, 30, 0.0);
  Drive(jump, t, 30, 5.0);
  for (const auto& seg : Tracks(jump)) CHECK_NEAR(seg.b.x - seg.a.x, 1.0 / 16, 0);
  CHECK(Tracks(jump).size() == 2);
}

void TestWidthOfASpin() {
  // A wheel heading +x scrubbing along +y (a turn in place) at D = 2 in sand sweeps its chord across:
  // half = (W |cos| + 2 sqrt(2 r D s) |sin|) / 2.
  TrackLayer layer(Exact());
  for (int k = 0; k < 40; ++k) {
    const WheelInput in = At(0.0, k * kStep, 2.0, kSand, 1.0, 0.0);
    layer.Step(k * kMs, &in, 1);
  }
  const auto segs = Tracks(layer);
  CHECK(!segs.empty());
  if (segs.empty()) return;
  const double chord = 2 * std::sqrt(2 * 0.15 * 2.0 * kSand);
  CHECK_NEAR(segs.front().b.half, chord / 2, 1e-9);
  CHECK_NEAR(segs.front().b.lx, -1.0, 1e-12);  // left of +y travel
  CHECK(segs.front().b.half > 2 * 0.05);
}

/// The floor quad between two cross-sections does not twist: its sides a-b on each edge do not cross.
bool Untwisted(const Segment& s) {
  const double ax = s.a.x + s.a.lx * s.a.half, ay = s.a.y + s.a.ly * s.a.half;  // left edge
  const double bx = s.b.x + s.b.lx * s.b.half, by = s.b.y + s.b.ly * s.b.half;
  const double cx = s.a.x - s.a.lx * s.a.half, cy = s.a.y - s.a.ly * s.a.half;  // right edge
  const double dx = s.b.x - s.b.lx * s.b.half, dy = s.b.y - s.b.ly * s.b.half;
  // The left edge stays left of the right one: (b - a) x (left - right) keeps its sign at both ends.
  const double tx = s.b.x - s.a.x, ty = s.b.y - s.a.y;
  return tx * (ay - cy) - ty * (ax - cx) > 0 && tx * (by - dy) - ty * (bx - dx) > 0;
}

void TestSharpTurn() {
  // Forward along +x, then straight back: the segment after the reversal starts square to its own travel
  // (the reversed lateral), not with the forward lateral, which would twist it into a bow-tie.
  TrackLayer layer(Exact());
  Stamp t = Drive(layer, kMs, 33, 0.0);  // two segments to 1/8
  for (int k = 1; k <= 40; ++k, t += kMs) {
    const WheelInput in = At(1.0 / 8 - k * kStep, 0.0);
    layer.Step(t, &in, 1);
  }
  const auto segs = Tracks(layer);
  CHECK(segs.size() >= 4);
  for (const auto& s : segs) CHECK(Untwisted(s));
  if (segs.size() >= 3) {
    CHECK_NEAR(segs[2].a.ly, -1.0, 1e-12);  // left of -x travel
    CHECK_NEAR(segs[1].b.ly, 1.0, 1e-12);   // the forward segment keeps its own
  }
  // A gentle arc keeps the shared cross-sections: a continuous strip.
  TrackLayer arc(Exact());
  for (int k = 0; k < 200; ++k) {
    const double a = k * kStep / 0.6;  // round a 0.6 m circle, a wheel of a spin in place
    const WheelInput in = At(0.6 * std::sin(a), 0.6 - 0.6 * std::cos(a), 1.0, kSand, 1.0, 0.0);
    arc.Step(k * kMs, &in, 1);
  }
  const auto bends = Tracks(arc);
  CHECK(bends.size() > 8);
  for (size_t i = 1; i < bends.size(); ++i) {
    CHECK(bends[i].a.lx == bends[i - 1].b.lx && bends[i].a.ly == bends[i - 1].b.ly);
    CHECK(Untwisted(bends[i]));
  }
}

void TestPitRule() {
  TrackLayer layer(Exact());
  Stamp t = Drive(layer, kMs, 17, 0.0);  // one segment, ending at 1/16
  // Digging in place: D up by 1/64 per step (a pit every 4 steps) while the wheel creeps under the spacing.
  double dig = 1.0;
  for (int k = 0; k < 64; ++k) {
    dig = std::min(2.0, dig + 1.0 / 64);
    const WheelInput in = At(1.0 / 16 + k / 4096.0, 0.0, dig);
    layer.Step(t, &in, 1);
    t += kMs;
  }
  const auto pits = Pits(layer);
  CHECK(pits.size() == 16);  // D 1 1/16 .. 2
  CHECK(Tracks(layer).size() == 1);
  size_t standing = 0;
  for (size_t i = 0; i < pits.size(); ++i) {
    CHECK_NEAR(pits[i].b.x, 1.0 / 16, 0);  // at the last cross-section
    CHECK(pits[i].a.x == pits[i].b.x && pits[i].a.dig == pits[i].b.dig);
    CHECK_NEAR(pits[i].b.dig, 1.0 + (i + 1) / 16.0, 0);
    if (i + 1 < pits.size()) CHECK(pits[i].until == pits[i + 1].stamp);  // replaced by the next, deeper one
    if (pits[i].until == kNever) ++standing;
  }
  CHECK(standing == 1);
  // Driving out: the pit stays; the next segment starts from the deepened cross-section.
  t = Drive(layer, t, 40, 1.0 / 16 + 64 / 4096.0, 1.0);
  CHECK(Pits(layer).back().until == kNever);
  const auto segs = Tracks(layer);
  CHECK(segs.size() >= 2);
  if (segs.size() >= 2) CHECK_NEAR(segs[1].a.dig, 2.0, 0);
  // Digging in again further on leaves a second pit; the first stays.
  dig = 1.0;
  t = Dig(layer, t, 8, 1.0 / 16 + 103 / 4096.0, dig);
  size_t standing_now = 0;
  for (const auto& pit : Pits(layer)) standing_now += pit.until == kNever;
  CHECK(standing_now == 2);
  // Normal driving with D steady never digs a pit.
  TrackLayer steady(Exact());
  Drive(steady, kMs, 500, 0.0, 1.25);
  CHECK(Pits(steady).empty());
}

void TestRing() {
  Params p = Exact();
  p.chunk = 8;
  p.max_segments = 32;
  TrackLayer layer(p);
  Drive(layer, kMs, 1601, 0.0);  // 100 segments
  CHECK(layer.Records() <= 32);
  CHECK(layer.Records() >= 32 - 8);
  CHECK(layer.Chunks() == 4);
  const auto segs = Tracks(layer);
  CHECK_NEAR(segs.back().b.x, 100.0 / 16, 0);    // the newest kept
  CHECK(segs.front().a.x >= (100.0 - 32) / 16);  // the oldest dropped
}

void TestReset() {
  TrackLayer layer(Exact());
  Drive(layer, kMs, 200, 0.0);
  CHECK(layer.Records() > 0);
  const uint64_t generation = layer.Generation();
  layer.Reset(0);
  CHECK(layer.Records() == 0);
  CHECK(layer.Chunks() == 0);
  CHECK(layer.Generation() == generation + 1);
  // The track starts afresh: no segment from where the wheel was before the reset.
  Drive(layer, kMs, 30, 3.0);
  CHECK(Tracks(layer).size() == 1);
  for (const auto& s : Tracks(layer)) CHECK(s.a.x == 3.0);
}

/// What frames every 20 ms draw of a drive with a dig in place, each taken `lag` steps after its own step: per
/// frame, the stamps of the records drawn and, negative, the indices of the pits removed.
std::vector<std::vector<int64_t>> Frames(int lag) {
  TrackLayer layer(Exact());
  Drawn drawn;
  std::vector<std::vector<int64_t>> out;
  double dig = 1.0, x = 0.0;
  std::vector<Stamp> due;
  for (int k = 1; k <= 900; ++k) {
    if (k > 300 && k <= 400) {
      dig = std::min(2.0, dig + 1.0 / 64);  // digging in place: a pit every 4 steps
    } else {
      x += kStep;
      dig = std::max(1.0, dig - 1.0 / 128);
    }
    const WheelInput in = At(x, 0.0, dig);
    layer.Step(k * kMs, &in, 1);
    if (k % 18 == 0) due.push_back(k * kMs);
    while (!due.empty() && (due.front() + lag * kMs <= k * kMs || k == 900)) {
      const Update u = layer.Take(due.front(), drawn, 100000);
      std::vector<int64_t> frame;
      for (const auto& b : u.batches)
        for (const auto& r : b.records) frame.push_back(r.stamp);
      for (const auto& d : u.dead) frame.push_back(-int64_t(d.index) - 1);
      out.push_back(frame);
      due.erase(due.begin());
    }
  }
  return out;
}

/// The snapshot rule: a frame at time T draws exactly the records visible at T, whatever the layer laid since.
void TestSnapshot() {
  TrackLayer layer(Exact());
  Drawn drawn;
  Stamp t = Drive(layer, kMs, 17, 0.0);  // the first segment at 17 ms
  // A frame at 16 ms (rendering while the step at 17 ms ran): nothing is due yet.
  Update u = layer.Take(16 * kMs, drawn, 1000);
  CHECK(u.batches.empty() && u.taken == 0);
  CHECK(!u.reset && !u.stale);
  // At 17 ms: the segment.
  u = layer.Take(17 * kMs, drawn, 1000);
  CHECK(u.batches.size() == 1 && u.taken == 1);
  if (u.batches.size() == 1) CHECK(u.batches[0].index[0] == 0 && !u.batches[0].records[0].pit);
  // Taken already: not again.
  CHECK(layer.Take(17 * kMs, drawn, 1000).taken == 0);
  // Digging in place from 18 ms: pits at 21 ms and 25 ms (D up 1/64 per step, one every 1/16).
  double dig = 1.0;
  t = Dig(layer, t, 8, 1.0 / 16, dig);
  const auto pits = Pits(layer);
  CHECK(pits.size() == 2);
  if (pits.size() != 2) return;
  CHECK(pits[0].stamp == 21 * kMs && pits[1].stamp == 25 * kMs && pits[0].until == 25 * kMs);
  // The frame at 24 ms (taken after the step at 25 ms deepened the pit): the first pit.
  u = layer.Take(24 * kMs, drawn, 1000);
  CHECK(PitsIn(u) == 1 && u.taken == 1 && u.dead.empty());
  if (u.batches.size() != 1) return;
  CHECK(u.batches[0].records[0].stamp == 21 * kMs);
  const uint32_t first = u.batches[0].index[0];
  // At 25 ms the first goes and the second comes.
  u = layer.Take(25 * kMs, drawn, 1000);
  CHECK(u.dead.size() == 1);
  if (u.dead.size() == 1) CHECK(u.dead[0].index == first);
  CHECK(PitsIn(u) == 1 && u.taken == 1);
  if (u.batches.size() == 1) CHECK(u.batches[0].records[0].stamp == 25 * kMs);
  CHECK(drawn.pits.size() == 1);
  // A pit replaced before any frame saw it is never drawn: a frame after four pits draws only the last.
  TrackLayer quick(Exact());
  Drawn seen;
  t = Drive(quick, kMs, 17, 0.0);
  quick.Take(17 * kMs, seen, 1000);
  dig = 1.0;
  t = Dig(quick, t, 16, 1.0 / 16, dig);
  CHECK(Pits(quick).size() == 4);
  u = quick.Take(t - kMs, seen, 1000);
  CHECK(PitsIn(u) == 1 && u.taken == 4 && u.dead.empty());
  // The same frames, taken the moment their step ends or while the next steps already lay more (the rendering
  // thread runs beside them): the same records and the same pits removed.
  const auto prompt = Frames(0);
  CHECK(prompt == Frames(1));
  CHECK(prompt == Frames(3));
  CHECK(prompt.size() > 40);
  size_t removed = 0;
  for (const auto& frame : prompt)
    for (const int64_t v : frame) removed += v < 0;
  CHECK(removed > 3);  // the dig in place deepened pits that frames had drawn
}

void TestSnapshotAcrossAReset() {
  TrackLayer layer(Exact());
  Drawn drawn;
  const Stamp t = Drive(layer, kMs, 2000, 0.0);  // to 2 s
  layer.Take(t - kMs, drawn, 100000);
  layer.Reset(0);
  const WheelInput in = At(9.0, 0.0);
  layer.Step(kMs, &in, 1);
  // A frame of the old generation (1.5 s) rendering while the world was reset: it changes nothing.
  Update u = layer.Take(1500 * kMs, drawn, 1000);
  CHECK(u.stale);
  CHECK(!u.reset && u.batches.empty());
  // The first frame of the new one: remove everything, draw what is due.
  Drive(layer, 2 * kMs, 40, 9.0 + kStep);
  u = layer.Take(30 * kMs, drawn, 1000);
  CHECK(u.reset && !u.stale);
  CHECK(u.taken == 1);
  for (const auto& b : u.batches)
    for (const auto& s : b.records) CHECK(s.stamp <= 30 * kMs && s.a.x >= 9.0);
  CHECK(drawn.generation == layer.Generation());
}

void TestBudgetAndDrops() {
  Params p = Exact();
  p.chunk = 8;
  p.max_segments = 64;
  TrackLayer layer(p);
  Drawn drawn;
  const Stamp t = Drive(layer, kMs, 801, 0.0);  // 50 segments: chunks of 8
  // A late viewer, 20 records per frame: the newest chunk first, all of them within three frames.
  Update u = layer.Take(t, drawn, 20);
  CHECK(u.taken == 20);
  if (!u.batches.empty()) CHECK(u.batches.front().chunk > u.batches.back().chunk);
  size_t total = 20;
  for (int frame = 0; frame < 2; ++frame) total += layer.Take(t, drawn, 20).taken;
  CHECK(total == 50);
  CHECK(layer.Take(t, drawn, 20).taken == 0);
  // Drive on until the ring drops chunks: the frame is told which.
  const Stamp later = Drive(layer, t, 800, 50.0 / 16 + kStep);
  u = layer.Take(later, drawn, 100000);
  CHECK(!u.dropped.empty());
  std::set<uint64_t> dropped(u.dropped.begin(), u.dropped.end());
  for (const auto& b : u.batches) CHECK(!dropped.count(b.chunk));
  CHECK(drawn.taken.size() == layer.Chunks());
}

void TestStress() {
  TrackLayer layer;
  layer.Stress(5 * kMs, 10.0, 20.0, 0.5, 1000);
  CHECK(layer.Records() == 1000);
  Drawn drawn;
  CHECK(layer.Take(4 * kMs, drawn, 100000).taken == 0);
  CHECK(layer.Take(5 * kMs, drawn, 100000).taken == 1000);
}

// --- Geometry ---------------------------------------------------------------------------------------------------

Section Cross(double dig, double sink, double x = 0.0) {
  Params p;
  return MakeSection(p, x, 0.0, 0.0, 1, 0, 1, 0, dig, sink);
}

void TestBermAndFloorNumbers() {
  Params p;
  CHECK_NEAR(BermHeight(p, Cross(1.0, kSand)), 0.006, 1e-12);        // 6 mm from normal driving
  CHECK_NEAR(BermHeight(p, Cross(2.0, kSand)), 0.026, 1e-12);        // 2.6 cm dug in
  CHECK_NEAR(BermHeight(p, Cross(2.0, kWashSand)), 0.039, 1e-12);    // wash sand: 3.9 cm
  CHECK_NEAR(BermHeight(p, Cross(1.0, kSandSheet)), 0.0045, 1e-12);  // the sand sheet: 4.5 mm
  CHECK_NEAR(BermHeight(p, Cross(1.0, kRegolith)), 0.0015, 1e-12);   // regolith: under berm_min
  CHECK_NEAR(BermHeight(p, Cross(5.0, 0.03)), 0.06, 1e-12);          // the cap
  CHECK(FloorLevel(p, Cross(1.0, 0.0)) == 0);        // rock: nothing
  CHECK(FloorLevel(p, Cross(1.0, kRegolith)) == 1);  // faint, 15 %
  CHECK(FloorLevel(p, Cross(1.0, kSand)) == 1);
  CHECK(FloorLevel(p, Cross(1.5, kSand)) == 2);
  CHECK(FloorLevel(p, Cross(2.0, kSand)) == 4);  // 48 %
  for (double d = 1.0; d < 2.0; d += 0.1) {
    CHECK(FloorLevel(p, Cross(d + 0.1, kSand)) >= FloorLevel(p, Cross(d, kSand)));
  }
}

Segment Track(double dig, double sink, uint32_t index = 2, bool smeared = false) {
  Segment s;
  s.a = Cross(dig, sink, 0.0);
  s.b = Cross(dig, sink, 0.05);
  s.a.index = index - 1;
  s.b.index = index;
  s.b.smeared = smeared;
  return s;
}

void TestTrackGeometry() {
  Params p;
  Mesh m;
  // Regolith: a faint floor only, at floor_offset above the surface.
  BuildTrack(p, Track(1.0, kRegolith), Flat, m);
  CHECK(m.berm.empty());
  CHECK(m.floor[1].size() == 6);
  for (const auto& v : m.floor[1]) CHECK_NEAR(v.z, 1.0 + p.floor_offset, 1e-12);
  // Sand dug in: the darkest floor (an even cross-section: no tread mark) and berms.
  m.Clear();
  BuildTrack(p, Track(2.0, kSand), Flat, m);
  CHECK(m.floor[4].size() == 6);
  CHECK(m.berm.size() == 24);
  double top = -1e9, bottom = 1e9, outer = 0.0;
  for (const auto& v : m.berm) {
    top = std::max(top, v.z);
    bottom = std::min(bottom, v.z);
    outer = std::max(outer, std::abs(v.y));
  }
  CHECK(top <= 1.0 + 0.026 * 1.35 + 1e-12);  // the crest, lumpy by +-35 %
  CHECK(top >= 1.0 + 0.026 * 0.65 - 1e-12);
  CHECK_NEAR(bottom, 1.0 - 0.01, 1e-12);  // the outer edge buried 1 cm
  const double half = Track(2.0, kSand).a.half;
  CHECK(outer <= half + (0.04 + 3 * 0.026) * 1.3 + 1e-12);
  // Every triangle faces up: (v1 - v2) x (v1 - v3), gz-rendering's normal, has z > 0.
  for (const auto* list : {&m.berm, &m.floor[4]}) {
    for (size_t i = 0; i + 2 < list->size(); i += 3) {
      const Vec3 &a = (*list)[i], &b = (*list)[i + 1], &c = (*list)[i + 2];
      const double nz = (a.x - b.x) * (a.y - c.y) - (a.y - b.y) * (a.x - c.x);
      CHECK(nz >= -1e-15);
    }
  }
  // Tread marks: odd cross-sections a level lighter unless smeared.
  m.Clear();
  BuildTrack(p, Track(2.0, kSand, 3), Flat, m);
  CHECK(m.floor[3].size() == 6 && m.floor[4].empty());
  m.Clear();
  BuildTrack(p, Track(2.0, kSand, 3, true), Flat, m);
  CHECK(m.floor[4].size() == 6);
  // Rock: nothing at all.
  m.Clear();
  BuildTrack(p, Track(2.0, 0.0), Flat, m);
  CHECK(m.Vertices() == 0);
  // Without a heightmap the contact's height is the surface.
  m.Clear();
  Segment plane = Track(1.0, kSand);
  plane.a.z = plane.b.z = 0.25;
  BuildTrack(p, plane, [](double, double, double fallback) { return fallback; }, m);
  for (const auto& v : m.floor[1]) CHECK_NEAR(v.z, 0.25 + p.floor_offset, 1e-12);
}

void TestPitGeometry() {
  Params p;
  Segment pit;
  pit.a = pit.b = Cross(2.0, kSand);
  pit.pit = true;
  Mesh m;
  BuildPit(p, pit, Flat, m);
  CHECK(m.berm.size() == 16 * 12);
  CHECK(m.floor[4].size() == 16 * 3);
  // The rim's crest meets the wheel at the static sinkage plus the crest: half-chord sqrt(2 r y - y^2).
  const double y = kSand + 0.026, a = std::sqrt(2 * 0.15 * y - y * y) + 0.01;
  double reach = 0.0;
  for (const auto& v : m.floor[4]) reach = std::max(reach, std::abs(v.x));
  CHECK_NEAR(reach, a, 1e-9);
  for (const auto& v : m.berm) CHECK(v.z >= 1.0 - 0.01 - 1e-12 && v.z <= 1.0 + 0.026 * 1.35 + 1e-12);
  // On regolith a pit's crest (1.5 mm at D = 1) is under berm_min: a faint floor, no rim.
  m.Clear();
  pit.a = pit.b = Cross(1.0, kRegolith);
  BuildPit(p, pit, Flat, m);
  CHECK(m.berm.empty());
  CHECK(m.floor[1].size() == 16 * 3);
}

}  // namespace

int main() {
  TestSpacing();
  TestOnlySoftGround();
  TestWidthOfASpin();
  TestSharpTurn();
  TestPitRule();
  TestRing();
  TestReset();
  TestSnapshot();
  TestSnapshotAcrossAReset();
  TestBudgetAndDrops();
  TestStress();
  TestBermAndFloorNumbers();
  TestTrackGeometry();
  TestPitGeometry();
  std::cout << (Failures() ? "FAILED" : "passed") << ": test_rover_tracks\n";
  return Failures();
}
