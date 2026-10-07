"""The drivetrain prototype as gates G5, G6 and G8 run it: sim/data/research/drive/prototype/cpp/
rover_drive.cpp plus <load_source> (contact | joint | both) and the joint-load estimate in its log.
Usage: python patch_proto.py <rover_drive.cpp> <patched copy>"""
import sys
from pathlib import Path

s = Path(sys.argv[1]).read_text()


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep('''    v_align_ = get("v_align", 0.005);''',
    '''    v_align_ = get("v_align", 0.005);
    // WS-0 gate G8: where the wheel loads come from. contact: ContactSensorData on the tyres (the
    // prototype's way); joint: JointTransmittedWrench of the wheel joints only; both: both, loads from
    // contacts, the joint estimate logged beside them.
    load_source_ = sdf->Get<std::string>("load_source", "both").first;
    joint_sign_ = get("joint_sign", 1.0);''')
rep('''      ecm.CreateComponent(joints_[k].Entity(), gz::sim::components::JointTransmittedWrench());
      for (auto c : links_[k].Collisions(ecm)) {
        collisions_[k] = c;
        ecm.CreateComponent(c, gz::sim::components::ContactSensorData());''',
    '''      if (load_source_ != "contact")
        ecm.CreateComponent(joints_[k].Entity(), gz::sim::components::JointTransmittedWrench());
      if (auto inertial = links_[k].WorldInertial(ecm)) wheel_mass_[k] = inertial->MassMatrix().Mass();
      for (auto c : links_[k].Collisions(ecm)) {
        collisions_[k] = c;
        if (load_source_ != "joint") ecm.CreateComponent(c, gz::sim::components::ContactSensorData());''')
rep('''    gz::math::Vector3d n = normal ? *normal : gz::math::Vector3d::UnitZ;
    n.Normalize();''',
    '''    gz::math::Vector3d n = normal ? *normal : gz::math::Vector3d::UnitZ;
    n.Normalize();
    normal_[k] = n.Z() < 0 ? -n : n;  // the ground's normal, pointing up into the wheel''')
rep('''    for (int k = 0; k < kWheels; ++k) {
      double tq = NAN;
      if (auto w = ecm.Component<gz::sim::components::JointTransmittedWrench>(joints_[k].Entity()))
        tq = w->Data().torque().y();''',
    '''    for (int k = 0; k < kWheels; ++k) {
      double tq = NAN, fnj = NAN;
      if (auto w = ecm.Component<gz::sim::components::JointTransmittedWrench>(joints_[k].Entity())) {
        tq = w->Data().torque().y();
        // The joint frame is the wheel's (identity joint pose): rotate the force into the world. Static
        // balance of the wheel: contact force = -(joint force + weight); its load is that along the normal.
        const auto& f = w->Data().force();
        const gz::math::Vector3d fw = joint_sign_ * rot_[k].RotateVector(gz::math::Vector3d(f.x(), f.y(), f.z()));
        const gz::math::Vector3d weight(0, 0, -9.81 * wheel_mass_[k]);
        fnj = -(fw + weight).Dot(normal_[k]);
      }''')
rep('''      fn_[k] = fn;
      rec.insert(rec.end(), {tq, applied_[k], ws_[k].i, ws_[k].d, ws_[k].w_o, ws_[k].sp, fn, ft,
                             static_cast<double>(nc), rr_[k]});''',
    '''      fn_[k] = load_source_ == "joint" ? std::max(0.0, fnj) : fn;
      rec.insert(rec.end(), {tq, applied_[k], ws_[k].i, ws_[k].d, ws_[k].w_o, ws_[k].sp, fn, ft,
                             static_cast<double>(nc), rr_[k], fnj});''')
rep('''        axle_[k] = p->Rot().RotateVector(gz::math::Vector3d::UnitY);''',
    '''        axle_[k] = p->Rot().RotateVector(gz::math::Vector3d::UnitY);
        rot_[k] = p->Rot();''')
rep('''  std::array<double, kWheels> w_{}, fn_{}, applied_{}, rr_{};''',
    '''  std::array<double, kWheels> w_{}, fn_{}, applied_{}, rr_{};
  std::array<double, kWheels> wheel_mass_{{2.5, 2.5, 2.5, 2.5}};
  std::array<gz::math::Quaterniond, kWheels> rot_;
  std::array<gz::math::Vector3d, kWheels> normal_{{gz::math::Vector3d::UnitZ, gz::math::Vector3d::UnitZ,
                                                   gz::math::Vector3d::UnitZ, gz::math::Vector3d::UnitZ}};
  std::string load_source_ = "both";
  double joint_sign_ = 1.0;''')
rep('''    gzmsg << "RoverDrive: drive=" << drive_''', '''    gzmsg << "RoverDrive: load_source=" << load_source_ << " drive=" << drive_''')
Path(sys.argv[2]).write_text(s)
print("patched", sys.argv[2])
