#include "rover_control/tank_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rover_control {

namespace {

void Positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("tank geometry: ") + name + " must be a positive number");
  }
}

double Number(const YAML::Node& root, const std::string& path, const char* section, const char* key) {
  const std::string name = std::string(section) + "." + key;
  const YAML::Node group = root[section];
  if (!group || !group.IsMap() || !group[key] || !group[key].IsScalar()) {
    throw std::runtime_error(path + ": no number " + name);
  }
  try {
    return group[key].as<double>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error(path + ": " + name + " is not a number");
  }
}

}  // namespace

void Validate(const TankGeometry& geometry) {
  Positive(geometry.wheel_radius, "wheel_radius");
  Positive(geometry.track, "track");
  Positive(geometry.wheelbase, "wheelbase");
  Positive(geometry.top_speed, "top_speed");
  Positive(geometry.track_multiplier, "track_multiplier");
}

WheelSpeeds ToWheels(const TankGeometry& geometry, const Twist& twist) {
  const double half = twist.wz * geometry.track * geometry.track_multiplier / 2.0;
  return {(twist.vx - half) / geometry.wheel_radius, (twist.vx + half) / geometry.wheel_radius};
}

Twist ToTwist(const TankGeometry& geometry, const WheelSpeeds& wheels) {
  return {geometry.wheel_radius * (wheels.left + wheels.right) / 2.0,
          geometry.wheel_radius * (wheels.right - wheels.left) / (geometry.track * geometry.track_multiplier)};
}

WheelSpeeds Saturate(const TankGeometry& geometry, const WheelSpeeds& wheels) {
  const double fastest = std::max(std::abs(wheels.left), std::abs(wheels.right));
  if (fastest <= geometry.top_speed) return wheels;
  const double scale = geometry.top_speed / fastest;
  return {wheels.left * scale, wheels.right * scale};
}

TankGeometry LoadRoverYaml(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  TankGeometry geometry;
  geometry.wheel_radius = Number(root, path, "wheel", "radius");
  geometry.track = 2.0 * Number(root, path, "rocker", "pivot_y");
  geometry.wheelbase = 2.0 * Number(root, path, "wheel", "dx");
  geometry.top_speed = Number(root, path, "wheel", "top_speed");
  Validate(geometry);
  return geometry;
}

}  // namespace rover_control
