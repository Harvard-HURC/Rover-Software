// Scratch check of sim/plugins/terrain_heightmap.hh: writes heights at the points of <in> to <out>.
#include <fstream>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include "terrain_heightmap.hh"

namespace check {
class HmCheck : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity&, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager&, gz::sim::EventManager&) override {
    in_ = sdf->Get<std::string>("in");
    out_ = sdf->Get<std::string>("out");
  }
  void PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm) override {
    if (done_) return;
    done_ = true;
    auto vis = rover_sim::FindTerrainHeightmap(ecm, rover_sim::HeightmapGeometry::kVisual);
    auto col = rover_sim::FindTerrainHeightmap(ecm, rover_sim::HeightmapGeometry::kCollision);
    std::ofstream out(out_); out.precision(12);
    out << (vis ? vis->path : "none") << " " << (col ? col->path : "none") << "\n";
    if (!vis || !col) return;
    out << vis->samples << " " << vis->size << " " << vis->origin << "\n";
    std::ifstream in(in_);
    double x, y;
    while (in >> x >> y) {
      unsigned int r = 0, c = 0;
      const bool ok = vis->Nearest(x, y, r, c);
      out << x << " " << y << " " << vis->Height(x, y) << " " << col->Height(x, y) << " " << ok << " " << r << " " << c << "\n";
    }
  }
 private:
  std::string in_, out_;
  bool done_ = false;
};
}  // namespace check
GZ_ADD_PLUGIN(check::HmCheck, gz::sim::System, check::HmCheck::ISystemConfigure, check::HmCheck::ISystemPreUpdate)
