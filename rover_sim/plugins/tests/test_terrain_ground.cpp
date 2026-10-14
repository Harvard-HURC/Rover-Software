// Unit tests of the ground map reader (terrain_ground.hh): ground.json's schema (design spec 9.1) and the
// nearest-sample lookup on ground.png.
#include <cstdio>
#include <string>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/common/Image.hh>

#include "check.hh"
#include "terrain_ground.hh"

using rover_sim::GroundMap;
using rover_sim::Traction;

namespace {

const std::string kJson = R"({"format": "rover-ground/2", "size_m": 8.0, "samples": 5, "default": 3,
 "types": [{"index": 3, "key": "regolith", "title": "Packed regolith", "mu_s": 0.62, "mu_k": 0.52, "crr": 0.10,
            "bulldoze": 0.0, "slip": 0.3, "sinkage_m": 0.005, "dig_rate": 0.0, "dig_max": 1.0, "dust": 0.4,
            "dust_rgb": [0.80, 0.70, 0.56]},
           {"index": 7, "key": "sand", "mu_s": 0.52, "mu_k": 0.52, "crr": 0.2, "bulldoze": 0.06, "slip": 1.0,
            "sinkage_m": 0.02, "dig_rate": 0.01, "dig_max": 1.25, "dust": 0.8},
           {"index": 12, "key": "rock", "mu_s": 1.0, "mu_k": 0.85, "crr": 0.015, "slip": 0.05},
           {"index": 20, "key": "manmade", "mu_s": 0.8, "mu_k": 0.7, "crr": 0.015, "slip": 0.05}],
 "collisions": {"step_10cm_collision": "rock", "zone_sand_3": "sand"},
 "prefixes": {"rocks_": "rock", "slabs_": "rock"},
 "terrain_default": "rock", "object_default": "manmade"})";

std::string Replace(std::string text, const std::string& from, const std::string& to) {
  text.replace(text.find(from), from.size(), to);
  return text;
}

void Schema() {
  GroundMap map;
  std::string error;
  CHECK(map.Parse(kJson, error));
  CHECK(map.SizeM() == 8.0 && map.Samples() == 5);
  const Traction* sand = map.Type("sand");
  CHECK(sand && sand->mu_k == 0.52 && sand->crr == 0.2 && sand->bulldoze == 0.06 && sand->slip == 1.0);
  CHECK(sand && sand->sinkage_m == 0.02 && sand->dig_rate == 0.01 && sand->dig_max == 1.25 && sand->dust == 0.8);
  CHECK(sand && sand->Digs() && !map.Type("regolith")->Digs());
  CHECK(map.Default()->key == "regolith");
  CHECK(map.ObjectDefault()->key == "manmade");
  CHECK(map.Collision("step_10cm_collision")->key == "rock");
  CHECK(map.Collision("zone_sand_3")->key == "sand");
  CHECK(map.Collision("rocks_0_-1_collision")->key == "rock");
  CHECK(map.Collision("ledge_0_collision")->key == "rock");  // terrain_default
  CHECK(map.Type("gravel") == nullptr);

  for (const auto& [from, to] : std::vector<std::pair<std::string, std::string>>{
           {"rover-ground/2", "rover-ground/1"}, {"\"default\": 3", "\"default\": 4"},
           {"\"zone_sand_3\": \"sand\"", "\"zone_sand_3\": \"dune\""}, {"\"samples\": 5", "\"samples\": 0"},
           {"\"index\": 12", "\"index\": 300"}}) {
    GroundMap bad;
    std::string why;
    CHECK(!bad.Parse(Replace(kJson, from, to), why) && !why.empty());
  }
  GroundMap broken;
  CHECK(!broken.Parse("{\"format\": ", error));
}

/// A 5 x 5 raster over 8 m centred on (10, -20): sample (row, col) at x = 6 + 2 col, y = -16 - 2 row. Row 0
/// (north) is sand, column 4 (east) rock, one sample an index the table lacks (the default), the rest regolith.
void Raster() {
  std::vector<unsigned char> pixels(25, 3);
  for (int c = 0; c < 5; ++c) pixels[c] = 7;
  for (int r = 0; r < 5; ++r) pixels[r * 5 + 4] = 12;
  pixels[2 * 5 + 2] = 99;
  gz::common::Image image;
  image.SetFromData(pixels.data(), 5, 5, gz::common::Image::L_INT8);
  const std::string path = gz::common::joinPaths(gz::common::cwd(), "ground_test.png");
  image.SavePNG(path);

  GroundMap map;
  std::string error;
  CHECK(map.Parse(kJson, error));
  CHECK(map.At(10, -20) == nullptr);  // no raster yet
  CHECK(map.LoadRaster(path, error));
  map.Place(gz::math::Vector3d(10, -20, 3));
  CHECK(map.At(6, -16)->key == "sand");      // north-west corner
  CHECK(map.At(11.9, -16.9)->key == "sand");  // nearest is row 0, column 3
  CHECK(map.At(11.9, -17.1)->key == "regolith");
  CHECK(map.At(14, -24)->key == "rock");     // south-east corner
  CHECK(map.At(13.1, -20)->key == "rock");
  CHECK(map.At(10, -20)->key == "regolith");  // index 99: the default type
  CHECK(map.At(5.1, -20) != nullptr);         // within half a sample of the edge
  CHECK(map.At(4.9, -20) == nullptr);
  CHECK(map.At(10, -15.0) == nullptr);

  GroundMap wrong;
  CHECK(wrong.Parse(Replace(kJson, "\"samples\": 5", "\"samples\": 9"), error));
  CHECK(!wrong.LoadRaster(path, error));
  CHECK(!wrong.LoadRaster(path + ".missing", error));
  std::remove(path.c_str());
}

}  // namespace

int main() {
  Schema();
  Raster();
  if (Failures() == 0) std::cout << "test_terrain_ground: all checks passed\n";
  return Failures() == 0 ? 0 : 1;
}
