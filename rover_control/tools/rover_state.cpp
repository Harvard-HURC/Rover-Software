// Prints the rover kinematic state for one command as JSON (used by viz.py).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "rover_driver/kinematics_2d.hpp"
#include "rover_driver/kinematics_3d.hpp"

using namespace rover_driver;

namespace {

const char* const kWheelNames[kNumWheels] = {"FL", "FR", "RL", "RR"};

struct Args {
  bool mode_3d = true;
  BodyTwist2d cmd;
  double gx = 0.0;
  double gy = 0.0;
  SuspensionState suspension;
};

void usage(std::FILE* f) {
  std::fprintf(f,
               "usage: rover_state [--mode 2d|3d] [--vx V] [--vy V] [--wz W] [--gx W] [--gy W]\n"
               "                   [--ql Q] [--qr Q] [--dql DQ] [--dqr DQ]\n");
}

// The field a numeric option sets, or nullptr for any other key.
double* number_field(const std::string& key, Args& a) {
  if (key == "--vx") return &a.cmd.vx;
  if (key == "--vy") return &a.cmd.vy;
  if (key == "--wz") return &a.cmd.wz;
  if (key == "--gx") return &a.gx;
  if (key == "--gy") return &a.gy;
  if (key == "--ql") return &a.suspension.q_left;
  if (key == "--qr") return &a.suspension.q_right;
  if (key == "--dql") return &a.suspension.dq_left;
  if (key == "--dqr") return &a.suspension.dq_right;
  return nullptr;
}

// On failure, prints the reason to stderr and returns false.
bool parse(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "-h" || key == "--help") {
      usage(stdout);
      std::exit(0);
    }
    double* const field = number_field(key, a);
    if (field == nullptr && key != "--mode") {
      std::fprintf(stderr, "rover_state: unknown option '%s'\n", key.c_str());
      return false;
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "rover_state: missing value for %s\n", key.c_str());
      return false;
    }
    const std::string value = argv[++i];
    if (field == nullptr) {  // --mode
      if (value != "2d" && value != "3d") {
        std::fprintf(stderr, "rover_state: invalid mode '%s' (expected 2d or 3d)\n",
                     value.c_str());
        return false;
      }
      a.mode_3d = value == "3d";
      continue;
    }
    char* end = nullptr;
    const double d = std::strtod(value.c_str(), &end);
    // strtod also accepts "nan", "inf" and out-of-range values (as inf).
    if (end == value.c_str() || *end != '\0' || !std::isfinite(d)) {
      std::fprintf(stderr, "rover_state: invalid value '%s' for %s (expected a finite number)\n",
                   value.c_str(), key.c_str());
      return false;
    }
    *field = d;
  }
  return true;
}

// A number formatted for JSON, which has no nan or inf: such a value (a huge
// command that overflowed in the kinematics) prints as null.
struct Num {
  explicit Num(double x) {
    if (std::isfinite(x)) {
      std::snprintf(text, sizeof(text), "%.9g", x);
    } else {
      std::snprintf(text, sizeof(text), "null");
    }
  }
  char text[32];
};

void print_vec(const Eigen::Vector3d& v) {
  std::printf("[%s, %s, %s]", Num(v.x()).text, Num(v.y()).text, Num(v.z()).text);
}

void print_vec(const Vec3& v) { print_vec(to_eigen(v)); }

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse(argc, argv, a)) {
    usage(stderr);
    return 2;
  }
  const RoverGeometry& g = kRover;
  // 2D mode ignores the gyro's roll/pitch rates and the rockers.
  const SuspensionState s = a.mode_3d ? a.suspension : SuspensionState{};
  const Gyro gyro = a.mode_3d ? Gyro{a.gx, a.gy, a.cmd.wz} : Gyro{0.0, 0.0, a.cmd.wz};
  const Eigen::Vector3d v_body(a.cmd.vx, a.cmd.vy, 0.0);
  const Eigen::Vector3d w_body(gyro.wx, gyro.wy, gyro.wz);

  PerWheel<ModuleCommand3d> cmds{};
  Eigen::Vector3d fk_v;
  PerWheel<double> fk_contact_angle{};  // 3D mode only
  double fk_wz = 0.0;
  double fk_residual = 0.0;
  bool fk_converged = true;
  int fk_iterations = 0;
  if (a.mode_3d) {
    cmds = inverse_3d(a.cmd, gyro, s, g);
    const Fk3dResult fk = forward_3d(to_measurements(cmds), gyro, s, g);
    fk_v = fk.v_body;
    fk_contact_angle = fk.contact_angle;
    fk_wz = gyro.wz;  // forward_3d takes the yaw rate from the gyro
    fk_residual = fk.residual_rms;
    fk_converged = fk.converged;
    fk_iterations = fk.iterations;
  } else {
    const auto cmds_2d = inverse_2d(a.cmd, g);
    for (int i = 0; i < kNumWheels; ++i) {
      static_cast<ModuleCommand&>(cmds[i]) = cmds_2d[i];
    }
    const Fk2dResult fk = forward_2d(to_measurements(cmds_2d), g);
    fk_v = {fk.twist.vx, fk.twist.vy, 0.0};
    fk_wz = fk.twist.wz;
    fk_residual = fk.residual_rms;
  }

  std::printf("{\n  \"mode\": \"%s\",\n", a.mode_3d ? "3d" : "2d");
  std::printf("  \"geometry\": {\"wheel_radius\": %s, \"pivot\": [", Num(g.wheel_radius).text);
  print_vec(g.pivot[kLeft]);
  std::printf(", ");
  print_vec(g.pivot[kRight]);
  std::printf("], \"wheel_offset\": [");
  for (int i = 0; i < kNumWheels; ++i) {
    print_vec(g.wheel_offset[i]);
    std::printf("%s", i + 1 < kNumWheels ? ", " : "");
  }
  std::printf("]},\n");
  std::printf("  \"command\": {\"vx\": %s, \"vy\": %s, \"wz\": %s},\n", Num(a.cmd.vx).text,
              Num(a.cmd.vy).text, Num(a.cmd.wz).text);
  std::printf("  \"gyro\": {\"wx\": %s, \"wy\": %s, \"wz\": %s},\n", Num(gyro.wx).text,
              Num(gyro.wy).text, Num(gyro.wz).text);
  std::printf(
      "  \"suspension\": {\"q_left\": %s, \"q_right\": %s, \"dq_left\": %s, \"dq_right\": %s},\n",
      Num(s.q_left).text, Num(s.q_right).text, Num(s.dq_left).text, Num(s.dq_right).text);
  std::printf("  \"wheels\": [\n");
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelChain c = wheel_chain(i, v_body, w_body, s, g);
    const Eigen::Matrix3d module = c.rocker_rot * rot_z(cmds[i].steer_angle);
    std::printf("    {\"name\": \"%s\", \"center\": ", kWheelNames[i]);
    print_vec(c.center);
    std::printf(", \"velocity\": ");
    print_vec(c.velocity);
    std::printf(", \"forward\": ");
    print_vec(module.col(0));
    std::printf(", \"axle\": ");
    print_vec(module.col(1));
    std::printf(", \"up\": ");
    print_vec(module.col(2));
    std::printf(
        ", \"steer_angle\": %s, \"wheel_rate\": %s, \"contact_angle\": %s, "
        "\"steer_defined\": %s}%s\n",
        Num(cmds[i].steer_angle).text, Num(cmds[i].wheel_rate).text,
        Num(cmds[i].contact_angle).text, cmds[i].steer_defined ? "true" : "false",
        i + 1 < kNumWheels ? "," : "");
  }
  std::printf("  ],\n");
  std::printf(
      "  \"fk\": {\"vx\": %s, \"vy\": %s, \"vz\": %s, \"wz\": %s, \"residual_rms\": %s, "
      "\"converged\": %s, \"iterations\": %d",
      Num(fk_v.x()).text, Num(fk_v.y()).text, Num(fk_v.z()).text, Num(fk_wz).text,
      Num(fk_residual).text, fk_converged ? "true" : "false", fk_iterations);
  if (a.mode_3d) {  // forward_2d estimates no contact angles
    std::printf(", \"contact_angle\": [%s, %s, %s, %s]", Num(fk_contact_angle[0]).text,
                Num(fk_contact_angle[1]).text, Num(fk_contact_angle[2]).text,
                Num(fk_contact_angle[3]).text);
  }
  std::printf("}\n}\n");
  return 0;
}
