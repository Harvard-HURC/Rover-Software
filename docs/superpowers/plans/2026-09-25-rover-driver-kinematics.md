# Rover Driver Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A C++ library in `driver/` with 2D and 3D (rocker-aware) inverse/forward kinematics for our 4-wheel swerve rocker rover, plus a CLI and a matplotlib visualizer.

**Architecture:** Pure `noexcept` functions over fixed-size structs; geometry is a `constexpr RoverGeometry kRover` passed by default argument so tests can inject their own. 3D kinematics propagates velocity body → rocker → wheel center with the transport theorem (Kelly & Seegmiller 2015); 3D IK follows Toupet et al. 2020 generalised to steered modules; 3D FK is a small Levenberg-Marquardt solve over body velocity and contact angles. A CLI prints the state as JSON; `viz.py` renders it.

**Tech Stack:** C++17, Eigen 5 (header-only), GoogleTest, CMake ≥ 3.20, Python 3.12 + matplotlib — all from the pixi environment. System Apple clang compiles.

**Spec:** `docs/superpowers/specs/2026-09-25-rover-driver-kinematics-design.md`

**Note:** the workspace is not a git repository, so there are no commit steps.

**Note:** Tasks 6-8 were extended in their own reviews and in the final review (listed under each task's **Files**). Where a task's code blocks differ from the files in `driver/`, they show the version first planned and the files are the reference.

**Conventions (used everywhere):** body frame x forward, y left, z up. Wheel indices FL=0, FR=1, RL=2, RR=3. Rocker angle `q` = rotation of the rocker relative to the body about +y (positive lowers the rocker's front). Steering angle `ψ` about the rocker z axis, 0 = rolling along rocker +x. Wheel rate positive = rolling toward module +x. Contact angle `η` measured from module +x toward module +z (positive = climbing).

---

## File map

| File | Responsibility |
|---|---|
| `pixi.toml` (modify) | `driver-build`, `driver-test`, `driver-viz` tasks |
| `.gitignore` (modify) | ignore `driver/build/` (and `__pycache__/`, added in the final review) |
| `driver/CMakeLists.txt` | library, tests, CLI targets |
| `driver/include/rover_driver/config.hpp` | `Vec3`, `RoverGeometry`, wheel/side enums, `kRover` constants |
| `driver/include/rover_driver/types.hpp` | command/measurement/result structs, `PerWheel`, `to_measurements` |
| `driver/include/rover_driver/kinematics_2d.hpp` / `src/kinematics_2d.cpp` | `nominal_wheel_xy`, `inverse_2d`, `forward_2d` |
| `driver/include/rover_driver/kinematics_3d.hpp` / `src/kinematics_3d.cpp` | `rot_y`, `rot_z`, `wheel_chain`, `inverse_3d`, `forward_3d` |
| `driver/tests/test_geometry.hpp` | `kExample` geometry for tests (wheels at ±0.5, ±0.4) |
| `driver/tests/test_kinematics_2d.cpp` | 2D tests |
| `driver/tests/test_kinematics_3d.cpp` | 3D tests |
| `driver/tests/test_viz.py` | `viz.py` and `rover_state` JSON tests (CTest `viz_py`, added in the Task 8 review) |
| `driver/tools/rover_state.cpp` | CLI: state → JSON |
| `driver/tools/viz.py` | interactive / snapshot visualization |

---

### Task 1: Scaffolding, configuration, types, `nominal_wheel_xy`

**Files:**
- Modify: `pixi.toml`, `.gitignore`
- Create: `driver/CMakeLists.txt`, `driver/include/rover_driver/config.hpp`, `driver/include/rover_driver/types.hpp`, `driver/include/rover_driver/kinematics_2d.hpp`, `driver/src/kinematics_2d.cpp`, `driver/tests/test_geometry.hpp`, `driver/tests/test_kinematics_2d.cpp`

- [ ] **Step 1: Add pixi tasks** — replace the empty `[tasks]` table in `pixi.toml` with:

```toml
[tasks]
driver-build = "cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release && cmake --build driver/build -j"
driver-test = { cmd = "ctest --test-dir driver/build --output-on-failure", depends-on = ["driver-build"] }
driver-viz = { cmd = "python driver/tools/viz.py", depends-on = ["driver-build"] }
```

- [ ] **Step 2: Ignore the build directory** — append to `.gitignore`:

```
# driver build output
driver/build/
```

- [ ] **Step 3: Create `driver/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(rover_driver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(rover_driver STATIC
  src/kinematics_2d.cpp
)
target_include_directories(rover_driver PUBLIC include)
target_link_libraries(rover_driver PUBLIC Eigen3::Eigen)
target_compile_options(rover_driver PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(rover_driver_tests
    tests/test_kinematics_2d.cpp
  )
  target_link_libraries(rover_driver_tests PRIVATE rover_driver GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(rover_driver_tests)
endif()
```

- [ ] **Step 4: Create `driver/include/rover_driver/config.hpp`**

```cpp
#pragma once

#include <array>

#include <Eigen/Core>

namespace rover_driver {

enum Wheel : int { kFL = 0, kFR = 1, kRL = 2, kRR = 3 };
enum Side : int { kLeft = 0, kRight = 1 };

constexpr int kNumWheels = 4;
constexpr Side kWheelSide[kNumWheels] = {kLeft, kRight, kLeft, kRight};

// constexpr-friendly 3-vector (Eigen types cannot be constexpr).
struct Vec3 {
  double x;
  double y;
  double z;
};

inline Eigen::Vector3d to_eigen(const Vec3& v) { return {v.x, v.y, v.z}; }

// Body frame: x forward, y left, z up. The body origin is the point whose
// velocity is commanded and estimated; all positions are relative to it.
struct RoverGeometry {
  double wheel_radius;
  // Rocker pivot positions in the body frame, indexed by Side.
  std::array<Vec3, 2> pivot;
  // Wheel-center position relative to its rocker pivot, in the rocker frame
  // (equal to the body-frame offset when the rocker angle is zero).
  std::array<Vec3, kNumWheels> wheel_offset;
  // Below this horizontal wheel speed [m/s] the steering angle is undefined.
  double steer_undefined_speed;
  // Weight [m/s per rad] pulling contact angles to zero in forward_3d.
  double fk3d_eta_prior_weight;
  // Weight [dimensionless] pulling body vertical velocity to zero in forward_3d:
  // the body is assumed to pitch and roll about its origin, as inverse_3d does.
  // Wheel odometry observes v_z only through rotation, so without it slip is
  // explained as vertical motion. In least-squares terms it is the wheel
  // velocity noise divided by the expected spread of v_z. The price: real v_z,
  // e.g. while pitching about the rear axle as the front wheels climb a step,
  // is underestimated and the rest lands in the contact angles and
  // residual_rms (see forward_3d). Lower weights shrink that bias but pass more
  // wheel noise into v_z.
  double fk3d_vz_prior_weight;
  // Largest |contact angle| [rad] forward_3d may report; keep it below pi/2.
  // At pi/2 the wheel center moves along module z, and beyond it opposite to
  // the way the wheel spins, so a wheel turning against its motion (reversed,
  // slipping, miswired) could be explained with no residual. A wheel meeting a
  // step of height h with its rocker level has contact angle acos(1 - h / R).
  double fk3d_max_contact_angle;
  int fk3d_max_iterations;
  double fk3d_tolerance;
};

// PLACEHOLDER numbers until the mechanical team provides the real geometry.
// Origin: on the ground, midway between the wheels.
inline constexpr RoverGeometry kRover{
    /*wheel_radius=*/0.15,
    /*pivot=*/{{{0.0, 0.40, 0.35}, {0.0, -0.40, 0.35}}},
    /*wheel_offset=*/
    {{{0.45, 0.0, -0.20}, {0.45, 0.0, -0.20}, {-0.45, 0.0, -0.20}, {-0.45, 0.0, -0.20}}},
    /*steer_undefined_speed=*/1e-3,
    /*fk3d_eta_prior_weight=*/1e-3,
    /*fk3d_vz_prior_weight=*/1.0,
    /*fk3d_max_contact_angle=*/1.4,  // 80 deg: a step of 0.83 R
    /*fk3d_max_iterations=*/50,
    /*fk3d_tolerance=*/1e-10,
};

}  // namespace rover_driver
```

- [ ] **Step 5: Create `driver/include/rover_driver/types.hpp`**

```cpp
#pragma once

#include <array>

#include <Eigen/Core>

#include "rover_driver/config.hpp"

namespace rover_driver {

template <class T>
using PerWheel = std::array<T, kNumWheels>;

// Planar body velocity command: linear [m/s] and yaw rate [rad/s].
struct BodyTwist2d {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
};

// Body angular velocity from the IMU gyro [rad/s], body frame.
struct Gyro {
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
};

// Rocker encoder readings: angles [rad] and rates [rad/s] relative to the body.
struct SuspensionState {
  double q_left = 0.0;
  double q_right = 0.0;
  double dq_left = 0.0;
  double dq_right = 0.0;
};

struct ModuleCommand {
  double steer_angle = 0.0;  // [rad]
  double wheel_rate = 0.0;   // [rad/s]
  bool steer_defined = false;
};

struct ModuleCommand3d : ModuleCommand {
  double contact_angle = 0.0;  // [rad]
};

struct ModuleMeasurement {
  double steer_angle = 0.0;  // [rad]
  double wheel_rate = 0.0;   // [rad/s]
};

struct Fk2dResult {
  BodyTwist2d twist;
  double residual_rms = 0.0;  // [m/s]
};

struct Fk3dResult {
  Eigen::Vector3d v_body = Eigen::Vector3d::Zero();  // [m/s], body frame
  PerWheel<double> contact_angle{};
  double residual_rms = 0.0;  // [m/s]
  bool converged = false;
  int iterations = 0;
};

// What the encoders would read if the modules tracked these commands exactly.
template <class Command>
PerWheel<ModuleMeasurement> to_measurements(const PerWheel<Command>& commands) noexcept {
  PerWheel<ModuleMeasurement> out{};
  for (int i = 0; i < kNumWheels; ++i) {
    out[i] = {commands[i].steer_angle, commands[i].wheel_rate};
  }
  return out;
}

}  // namespace rover_driver
```

- [ ] **Step 6: Create `driver/tests/test_geometry.hpp`**

```cpp
#pragma once

#include "rover_driver/config.hpp"

namespace rover_driver::test {

// Worked-example geometry from the design discussion: wheels at (±0.5, ±0.4).
inline constexpr RoverGeometry kExample{
    /*wheel_radius=*/0.15,
    /*pivot=*/{{{0.0, 0.4, 0.35}, {0.0, -0.4, 0.35}}},
    /*wheel_offset=*/
    {{{0.5, 0.0, -0.2}, {0.5, 0.0, -0.2}, {-0.5, 0.0, -0.2}, {-0.5, 0.0, -0.2}}},
    /*steer_undefined_speed=*/1e-6,
    /*fk3d_eta_prior_weight=*/1e-3,
    /*fk3d_vz_prior_weight=*/1.0,
    /*fk3d_max_contact_angle=*/1.4,
    /*fk3d_max_iterations=*/100,
    /*fk3d_tolerance=*/1e-12,
};

constexpr double kPi = 3.14159265358979323846;

}  // namespace rover_driver::test
```

- [ ] **Step 7: Write the failing test** — create `driver/tests/test_kinematics_2d.cpp`:

```cpp
#include <cmath>

#include <gtest/gtest.h>

#include "rover_driver/kinematics_2d.hpp"
#include "test_geometry.hpp"

using namespace rover_driver;
using rover_driver::test::kExample;
using rover_driver::test::kPi;

TEST(NominalWheelXy, IsPivotPlusOffset) {
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFL, kExample).x(), 0.5);
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kFL, kExample).y(), 0.4);
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRR, kExample).x(), -0.5);
  EXPECT_DOUBLE_EQ(nominal_wheel_xy(kRR, kExample).y(), -0.4);
}

TEST(NominalWheelXy, DefaultGeometryMatchesWheelNames) {
  EXPECT_GT(nominal_wheel_xy(kFL).x(), 0.0);
  EXPECT_GT(nominal_wheel_xy(kFL).y(), 0.0);
  EXPECT_GT(nominal_wheel_xy(kFR).x(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kFR).y(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kRL).x(), 0.0);
  EXPECT_GT(nominal_wheel_xy(kRL).y(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kRR).x(), 0.0);
  EXPECT_LT(nominal_wheel_xy(kRR).y(), 0.0);
}
```

- [ ] **Step 8: Create the header with the declaration only** — `driver/include/rover_driver/kinematics_2d.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "rover_driver/config.hpp"
#include "rover_driver/types.hpp"

namespace rover_driver {

// Wheel-center (x, y) in the body frame with both rockers at zero angle.
Eigen::Vector2d nominal_wheel_xy(int wheel, const RoverGeometry& g = kRover) noexcept;

}  // namespace rover_driver
```

and an empty `driver/src/kinematics_2d.cpp`:

```cpp
#include "rover_driver/kinematics_2d.hpp"
```

- [ ] **Step 9: Run to verify it fails**

Run: `pixi run driver-build`
Expected: link error `Undefined symbols ... nominal_wheel_xy`.

- [ ] **Step 10: Implement** — replace `driver/src/kinematics_2d.cpp` with:

```cpp
#include "rover_driver/kinematics_2d.hpp"

namespace rover_driver {

Eigen::Vector2d nominal_wheel_xy(int wheel, const RoverGeometry& g) noexcept {
  const Vec3& p = g.pivot[kWheelSide[wheel]];
  const Vec3& a = g.wheel_offset[wheel];
  return {p.x + a.x, p.y + a.y};
}

}  // namespace rover_driver
```

- [ ] **Step 11: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed, 0 tests failed out of 2`.

---

### Task 2: `inverse_2d`

**Files:**
- Modify: `driver/include/rover_driver/kinematics_2d.hpp`, `driver/src/kinematics_2d.cpp`
- Test: `driver/tests/test_kinematics_2d.cpp`

- [ ] **Step 1: Write the failing tests** — append to `driver/tests/test_kinematics_2d.cpp`:

```cpp
TEST(Inverse2d, StraightAheadDrivesAllWheelsForwardAtSameRate) {
  const auto cmd = inverse_2d({1.0, 0.0, 0.0}, kExample);
  for (const auto& m : cmd) {
    EXPECT_TRUE(m.steer_defined);
    EXPECT_NEAR(m.steer_angle, 0.0, 1e-12);
    EXPECT_NEAR(m.wheel_rate, 1.0 / 0.15, 1e-12);
  }
}

TEST(Inverse2d, StrafeLeftPointsAllWheelsLeft) {
  const auto cmd = inverse_2d({0.0, 1.0, 0.0}, kExample);
  for (const auto& m : cmd) {
    EXPECT_NEAR(m.steer_angle, kPi / 2.0, 1e-12);
    EXPECT_NEAR(m.wheel_rate, 1.0 / 0.15, 1e-12);
  }
}

TEST(Inverse2d, TurnInPlaceKeepsWheelsTangent) {
  const auto cmd = inverse_2d({0.0, 0.0, 1.0}, kExample);
  for (int i = 0; i < kNumWheels; ++i) {
    const Eigen::Vector2d r = nominal_wheel_xy(i, kExample);
    const Eigen::Vector2d dir(std::cos(cmd[i].steer_angle), std::sin(cmd[i].steer_angle));
    EXPECT_NEAR(dir.dot(r), 0.0, 1e-12) << "wheel " << i;
    EXPECT_GT(r.x() * dir.y() - r.y() * dir.x(), 0.0) << "wheel " << i << " not CCW";
    EXPECT_NEAR(cmd[i].wheel_rate, r.norm() / 0.15, 1e-12) << "wheel " << i;
  }
}

TEST(Inverse2d, WorkedExampleForwardWhileTurningLeft) {
  const auto cmd = inverse_2d({1.0, 0.0, 0.5}, kExample);
  EXPECT_NEAR(cmd[kFL].steer_angle, std::atan2(0.25, 0.8), 1e-12);
  EXPECT_NEAR(cmd[kFR].steer_angle, std::atan2(0.25, 1.2), 1e-12);
  EXPECT_NEAR(cmd[kRL].steer_angle, std::atan2(-0.25, 0.8), 1e-12);
  EXPECT_NEAR(cmd[kRR].steer_angle, std::atan2(-0.25, 1.2), 1e-12);
  EXPECT_NEAR(cmd[kFL].steer_angle * 180.0 / kPi, 17.354, 1e-3);
  EXPECT_NEAR(cmd[kFL].wheel_rate * 0.15, 0.8382, 1e-4);
  EXPECT_NEAR(cmd[kFR].wheel_rate * 0.15, 1.2258, 1e-4);
  EXPECT_NEAR(cmd[kRL].wheel_rate, cmd[kFL].wheel_rate, 1e-12);
  EXPECT_NEAR(cmd[kRR].wheel_rate, cmd[kFR].wheel_rate, 1e-12);
}

TEST(Inverse2d, ZeroCommandLeavesSteeringUndefined) {
  const auto cmd = inverse_2d({0.0, 0.0, 0.0}, kExample);
  for (const auto& m : cmd) {
    EXPECT_FALSE(m.steer_defined);
    EXPECT_EQ(m.steer_angle, 0.0);
    EXPECT_EQ(m.wheel_rate, 0.0);
  }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pixi run driver-build`
Expected: compile error `use of undeclared identifier 'inverse_2d'`.

- [ ] **Step 3: Declare** — add to `kinematics_2d.hpp` after `nominal_wheel_xy`:

```cpp
// Flat-ground swerve IK: body twist -> per-module steering angle and wheel rate.
// Modules slower than g.steer_undefined_speed get steer_defined = false and
// zero angle/rate.
PerWheel<ModuleCommand> inverse_2d(const BodyTwist2d& cmd,
                                   const RoverGeometry& g = kRover) noexcept;
```

- [ ] **Step 4: Implement** — in `kinematics_2d.cpp` add `#include <cmath>` below the first include, and inside the namespace after `nominal_wheel_xy`:

```cpp
PerWheel<ModuleCommand> inverse_2d(const BodyTwist2d& cmd, const RoverGeometry& g) noexcept {
  PerWheel<ModuleCommand> out{};
  for (int i = 0; i < kNumWheels; ++i) {
    const Eigen::Vector2d r = nominal_wheel_xy(i, g);
    const double vx = cmd.vx - cmd.wz * r.y();
    const double vy = cmd.vy + cmd.wz * r.x();
    const double speed = std::hypot(vx, vy);
    if (speed < g.steer_undefined_speed) {
      continue;
    }
    out[i].steer_angle = std::atan2(vy, vx);
    out[i].wheel_rate = speed / g.wheel_radius;
    out[i].steer_defined = true;
  }
  return out;
}
```

- [ ] **Step 5: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed, 0 tests failed out of 7`.

---

### Task 3: `forward_2d`

**Files:**
- Modify: `driver/include/rover_driver/kinematics_2d.hpp`, `driver/src/kinematics_2d.cpp`
- Test: `driver/tests/test_kinematics_2d.cpp`

- [ ] **Step 1: Write the failing tests** — append to `driver/tests/test_kinematics_2d.cpp`:

```cpp
TEST(Forward2d, RoundTripsInverse2d) {
  const BodyTwist2d twists[] = {
      {0.3, -0.2, 0.7}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.2}, {-0.5, 0.4, 0.1}};
  for (const auto& twist : twists) {
    const auto fk = forward_2d(to_measurements(inverse_2d(twist, kExample)), kExample);
    EXPECT_NEAR(fk.twist.vx, twist.vx, 1e-12);
    EXPECT_NEAR(fk.twist.vy, twist.vy, 1e-12);
    EXPECT_NEAR(fk.twist.wz, twist.wz, 1e-12);
    EXPECT_LT(fk.residual_rms, 1e-12);
  }
}

TEST(Forward2d, InconsistentWheelShowsResidual) {
  auto meas = to_measurements(inverse_2d({1.0, 0.0, 0.0}, kExample));
  meas[kFL].wheel_rate *= 1.5;  // front-left spins 50 % fast (slipping)
  const auto fk = forward_2d(meas, kExample);
  EXPECT_GT(fk.residual_rms, 0.05);
  EXPECT_GT(fk.twist.vx, 1.0);
  EXPECT_LT(fk.twist.vx, 1.5);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pixi run driver-build`
Expected: compile error `use of undeclared identifier 'forward_2d'`.

- [ ] **Step 3: Declare** — add to `kinematics_2d.hpp` after `inverse_2d`:

```cpp
// Flat-ground swerve FK: least-squares body twist from all modules
// (Kelly & Seegmiller 2015, eq. 75, zero steering offset). residual_rms is the
// RMS of the 8 velocity-equation residuals [m/s] - a slip indicator.
Fk2dResult forward_2d(const PerWheel<ModuleMeasurement>& meas,
                      const RoverGeometry& g = kRover) noexcept;
```

- [ ] **Step 4: Implement** — in `kinematics_2d.cpp` add `#include <Eigen/Dense>` below `<cmath>`, and inside the namespace after `inverse_2d`:

```cpp
Fk2dResult forward_2d(const PerWheel<ModuleMeasurement>& meas, const RoverGeometry& g) noexcept {
  // Each module: R * rate * [cos psi, sin psi] = [vx - wz*y, vy + wz*x].
  Eigen::Matrix<double, 2 * kNumWheels, 3> a;
  Eigen::Matrix<double, 2 * kNumWheels, 1> b;
  for (int i = 0; i < kNumWheels; ++i) {
    const Eigen::Vector2d r = nominal_wheel_xy(i, g);
    const double speed = g.wheel_radius * meas[i].wheel_rate;
    a.row(2 * i) << 1.0, 0.0, -r.y();
    a.row(2 * i + 1) << 0.0, 1.0, r.x();
    b(2 * i) = speed * std::cos(meas[i].steer_angle);
    b(2 * i + 1) = speed * std::sin(meas[i].steer_angle);
  }
  const Eigen::Vector3d x = a.colPivHouseholderQr().solve(b);

  Fk2dResult out;
  out.twist = {x(0), x(1), x(2)};
  out.residual_rms = std::sqrt((a * x - b).squaredNorm() / (2 * kNumWheels));
  return out;
}
```

- [ ] **Step 5: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed, 0 tests failed out of 9`.

---

### Task 4: 3D chain — `rot_y`, `rot_z`, `wheel_chain`

**Files:**
- Modify: `driver/CMakeLists.txt`
- Create: `driver/include/rover_driver/kinematics_3d.hpp`, `driver/src/kinematics_3d.cpp`, `driver/tests/test_kinematics_3d.cpp`

- [ ] **Step 1: Add the 3D sources to CMake** — in `driver/CMakeLists.txt` change the library and test source lists to:

```cmake
add_library(rover_driver STATIC
  src/kinematics_2d.cpp
  src/kinematics_3d.cpp
)
```

```cmake
  add_executable(rover_driver_tests
    tests/test_kinematics_2d.cpp
    tests/test_kinematics_3d.cpp
  )
```

- [ ] **Step 2: Write the failing tests** — create `driver/tests/test_kinematics_3d.cpp`:

```cpp
#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "rover_driver/kinematics_2d.hpp"
#include "rover_driver/kinematics_3d.hpp"
#include "test_geometry.hpp"

using namespace rover_driver;
using rover_driver::test::kExample;

namespace {

// World position of a wheel center at time t, for a body that starts at the
// world origin with identity attitude and moves with constant body twist
// (v_b, w_b) while the rockers move at constant rates.
Eigen::Vector3d world_wheel_center(int wheel, double t, const Eigen::Vector3d& v_b,
                                   const Eigen::Vector3d& w_b, const SuspensionState& s0) {
  SuspensionState s = s0;
  s.q_left += s0.dq_left * t;
  s.q_right += s0.dq_right * t;
  const Eigen::Matrix3d r =
      Eigen::AngleAxisd(w_b.norm() * t, w_b.normalized()).toRotationMatrix();
  const WheelChain chain = wheel_chain(wheel, v_b, w_b, s, kExample);
  return v_b * t + r * chain.center;
}

}  // namespace

TEST(WheelChain, CenterAtZeroRockerAngleIsPivotPlusOffset) {
  const WheelChain c = wheel_chain(kFR, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                   SuspensionState{}, kExample);
  EXPECT_TRUE(c.center.isApprox(Eigen::Vector3d(0.5, -0.4, 0.15)));
}

TEST(WheelChain, PositiveRockerAngleLowersFrontWheel) {
  SuspensionState s;
  s.q_left = 0.1;
  const WheelChain front = wheel_chain(kFL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                       s, kExample);
  const WheelChain rear = wheel_chain(kRL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                      s, kExample);
  EXPECT_LT(front.center.z(), 0.15);
  EXPECT_GT(rear.center.z(), 0.15);
}

TEST(WheelChain, VelocityMatchesFiniteDifferenceOfPosition) {
  struct Case {
    Eigen::Vector3d v_b;
    Eigen::Vector3d w_b;
    SuspensionState s;
  };
  const Case cases[] = {
      {{0.7, -0.2, 0.05}, {0.1, -0.3, 0.4}, {0.12, -0.12, 0.3, -0.3}},
      {{-0.3, 0.5, -0.1}, {-0.2, 0.25, -0.6}, {-0.2, 0.15, -0.5, 0.2}},
      {{0.0, 0.0, 0.0}, {0.4, 0.1, 0.0}, {0.05, -0.05, 0.0, 0.0}},
  };
  const double h = 1e-6;
  for (const auto& c : cases) {
    for (int i = 0; i < kNumWheels; ++i) {
      const Eigen::Vector3d fd = (world_wheel_center(i, h, c.v_b, c.w_b, c.s) -
                                  world_wheel_center(i, -h, c.v_b, c.w_b, c.s)) /
                                 (2.0 * h);
      const WheelChain chain = wheel_chain(i, c.v_b, c.w_b, c.s, kExample);
      EXPECT_NEAR(chain.velocity.x(), fd.x(), 1e-7) << "wheel " << i;
      EXPECT_NEAR(chain.velocity.y(), fd.y(), 1e-7) << "wheel " << i;
      EXPECT_NEAR(chain.velocity.z(), fd.z(), 1e-7) << "wheel " << i;
    }
  }
}
```

- [ ] **Step 3: Create the header with declarations** — `driver/include/rover_driver/kinematics_3d.hpp`:

```cpp
#pragma once

#include <Eigen/Core>

#include "rover_driver/config.hpp"
#include "rover_driver/types.hpp"

namespace rover_driver {

Eigen::Matrix3d rot_y(double angle) noexcept;
Eigen::Matrix3d rot_z(double angle) noexcept;

// Kinematic chain body -> rocker -> wheel center for one wheel, all in body
// coordinates (Kelly & Seegmiller 2015, transport theorem).
struct WheelChain {
  Eigen::Vector3d center;        // wheel-center position
  Eigen::Vector3d velocity;      // wheel-center velocity w.r.t. the world
  Eigen::Matrix3d rocker_rot;    // rocker frame -> body frame
  Eigen::Vector3d rocker_omega;  // rocker angular velocity w.r.t. the world
};

// v_body, w_body: body linear/angular velocity w.r.t. the world, body frame.
WheelChain wheel_chain(int wheel, const Eigen::Vector3d& v_body, const Eigen::Vector3d& w_body,
                       const SuspensionState& s, const RoverGeometry& g = kRover) noexcept;

}  // namespace rover_driver
```

and `driver/src/kinematics_3d.cpp`:

```cpp
#include "rover_driver/kinematics_3d.hpp"
```

- [ ] **Step 4: Run to verify it fails**

Run: `pixi run driver-build`
Expected: link error `Undefined symbols ... wheel_chain`.

- [ ] **Step 5: Implement** — replace `driver/src/kinematics_3d.cpp` with:

```cpp
#include "rover_driver/kinematics_3d.hpp"

#include <cmath>

namespace rover_driver {

Eigen::Matrix3d rot_y(double angle) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix3d r;
  r << c, 0.0, s,
       0.0, 1.0, 0.0,
       -s, 0.0, c;
  return r;
}

Eigen::Matrix3d rot_z(double angle) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix3d r;
  r << c, -s, 0.0,
       s, c, 0.0,
       0.0, 0.0, 1.0;
  return r;
}

WheelChain wheel_chain(int wheel, const Eigen::Vector3d& v_body, const Eigen::Vector3d& w_body,
                       const SuspensionState& s, const RoverGeometry& g) noexcept {
  const int side = kWheelSide[wheel];
  const double q = side == kLeft ? s.q_left : s.q_right;
  const double dq = side == kLeft ? s.dq_left : s.dq_right;
  const Eigen::Vector3d pivot = to_eigen(g.pivot[side]);

  WheelChain c;
  c.rocker_rot = rot_y(q);
  const Eigen::Vector3d arm = c.rocker_rot * to_eigen(g.wheel_offset[wheel]);
  c.center = pivot + arm;
  c.rocker_omega = w_body + dq * Eigen::Vector3d::UnitY();
  c.velocity = v_body + w_body.cross(pivot) + c.rocker_omega.cross(arm);
  return c;
}

}  // namespace rover_driver
```

- [ ] **Step 6: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed, 0 tests failed out of 12`.

---

### Task 5: `inverse_3d`

**Files:**
- Modify: `driver/include/rover_driver/kinematics_3d.hpp`, `driver/src/kinematics_3d.cpp`
- Test: `driver/tests/test_kinematics_3d.cpp`

- [ ] **Step 1: Write the failing tests** — append to `driver/tests/test_kinematics_3d.cpp`:

```cpp
TEST(Inverse3d, MatchesInverse2dWhenLevelAndStill) {
  const BodyTwist2d twists[] = {
      {1.0, 0.0, 0.5}, {0.3, -0.2, 0.7}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, 0.0}};
  for (const auto& geometry : {kRover, kExample}) {
    for (const auto& twist : twists) {
      const auto c2 = inverse_2d(twist, geometry);
      const auto c3 = inverse_3d(twist, Gyro{0.0, 0.0, twist.wz}, SuspensionState{}, geometry);
      for (int i = 0; i < kNumWheels; ++i) {
        EXPECT_EQ(c3[i].steer_defined, c2[i].steer_defined) << "wheel " << i;
        EXPECT_NEAR(c3[i].steer_angle, c2[i].steer_angle, 1e-12) << "wheel " << i;
        EXPECT_NEAR(c3[i].wheel_rate, c2[i].wheel_rate, 1e-12) << "wheel " << i;
        EXPECT_NEAR(c3[i].contact_angle, 0.0, 1e-12) << "wheel " << i;
      }
    }
  }
}

TEST(Inverse3d, FrontLeftClimbingSpeedsUpAndTiltsContact) {
  SuspensionState s;
  s.dq_left = -0.5;  // left rocker pitching nose-up: FL climbs, RL descends
  const auto cmd = inverse_3d({0.3, 0.0, 0.0}, Gyro{}, s, kExample);
  EXPECT_GT(cmd[kFL].contact_angle, 0.1);
  EXPECT_LT(cmd[kRL].contact_angle, -0.1);
  EXPECT_NEAR(cmd[kFR].contact_angle, 0.0, 1e-12);
  EXPECT_NEAR(cmd[kRR].contact_angle, 0.0, 1e-12);
  EXPECT_GT(cmd[kFL].wheel_rate, cmd[kFR].wheel_rate);
  EXPECT_NEAR(cmd[kFR].wheel_rate, 0.3 / 0.15, 1e-12);
}

TEST(Inverse3d, StaticEqualRockerAnglesTiltContactOnly) {
  SuspensionState s;
  s.q_left = 0.2;
  s.q_right = 0.2;
  const auto cmd = inverse_3d({0.4, 0.0, 0.0}, Gyro{}, s, kExample);
  for (int i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(cmd[i].steer_angle, 0.0, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].wheel_rate, 0.4 / 0.15, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].contact_angle, 0.2, 1e-12) << "wheel " << i;
  }
}

TEST(Inverse3d, StaticTwistTiltsContactPerSide) {
  SuspensionState s;
  s.q_left = 0.2;
  s.q_right = -0.2;
  const auto cmd = inverse_3d({0.4, 0.0, 0.0}, Gyro{}, s, kExample);
  for (int i = 0; i < kNumWheels; ++i) {
    const double expected = kWheelSide[i] == kLeft ? 0.2 : -0.2;
    EXPECT_NEAR(cmd[i].steer_angle, 0.0, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].wheel_rate, 0.4 / 0.15, 1e-12) << "wheel " << i;
    EXPECT_NEAR(cmd[i].contact_angle, expected, 1e-12) << "wheel " << i;
  }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pixi run driver-build`
Expected: compile error `use of undeclared identifier 'inverse_3d'`.

- [ ] **Step 3: Declare** — add to `kinematics_3d.hpp` after `wheel_chain`:

```cpp
// Terrain-aware IK (Toupet et al. 2020 generalised to swerve). Body velocity is
// (cmd.vx, cmd.vy, 0) - vertical assumed zero - and angular velocity is
// (gyro.wx, gyro.wy, cmd.wz). Steering follows the wheel-center velocity in
// the rocker frame; wheel rate removes the fork's own rotation about the axle.
PerWheel<ModuleCommand3d> inverse_3d(const BodyTwist2d& cmd, const Gyro& gyro,
                                     const SuspensionState& s,
                                     const RoverGeometry& g = kRover) noexcept;
```

- [ ] **Step 4: Implement** — add inside the namespace in `kinematics_3d.cpp`, after `wheel_chain`:

```cpp
PerWheel<ModuleCommand3d> inverse_3d(const BodyTwist2d& cmd, const Gyro& gyro,
                                     const SuspensionState& s, const RoverGeometry& g) noexcept {
  const Eigen::Vector3d v_body(cmd.vx, cmd.vy, 0.0);
  const Eigen::Vector3d w_body(gyro.wx, gyro.wy, cmd.wz);

  PerWheel<ModuleCommand3d> out{};
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelChain c = wheel_chain(i, v_body, w_body, s, g);
    const Eigen::Vector3d u = c.rocker_rot.transpose() * c.velocity;  // rocker frame
    const double horizontal = std::hypot(u.x(), u.y());
    if (horizontal < g.steer_undefined_speed) {
      continue;
    }
    const double psi = std::atan2(u.y(), u.x());
    const Eigen::Vector3d axle = c.rocker_rot * rot_z(psi) * Eigen::Vector3d::UnitY();
    const double fork_spin = axle.dot(c.rocker_omega);

    out[i].steer_angle = psi;
    out[i].wheel_rate = c.velocity.norm() / g.wheel_radius - fork_spin;
    out[i].steer_defined = true;
    out[i].contact_angle = std::atan2(u.z(), horizontal);
  }
  return out;
}
```

- [ ] **Step 5: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed, 0 tests failed out of 16`.

---

### Task 6: `forward_3d`

**Files:**
- Modify: `driver/include/rover_driver/kinematics_3d.hpp`, `driver/src/kinematics_3d.cpp`
- Modify (added in review): `driver/include/rover_driver/config.hpp` and
  `driver/tests/test_geometry.hpp` (`fk3d_vz_prior_weight`,
  `fk3d_max_contact_angle`, also in `kSkewed` in `driver/tests/test_kinematics_2d.cpp`),
  `driver/include/rover_driver/types.hpp` (`Fk3dResult::iterations` comment)
- Modify (final review): `driver/src/kinematics_3d.cpp` (LM damping capped
  relative to the Hessian's largest entry instead of at 1e12; a stall counts
  as converged only if the gradient is small next to `|J_k| |r|`; `v_body`,
  contact angles and `residual_rms` are NaN when the cost is not finite),
  `driver/include/rover_driver/kinematics_3d.hpp` (non-convexity, rocker-angle
  range, NaN outputs)
- Test: `driver/tests/test_kinematics_3d.cpp` (final review: `Forward3d.NonFiniteInputIsNotConverged`
  extended; `Forward3d.SolutionScalesWithHugeRates` and
  `Inverse3d.SteeringUndefinedBelowThresholdInTheRockerFrame` added)

- [ ] **Step 1: Write the failing tests** — append to `driver/tests/test_kinematics_3d.cpp`:

```cpp
TEST(Forward3d, RoundTripsInverse3d) {
  struct Case {
    BodyTwist2d twist;
    Gyro gyro;  // gyro.wz must equal twist.wz for a consistent round trip
    SuspensionState s;
  };
  const Case cases[] = {
      {{0.4, 0.1, 0.3}, {0.05, -0.08, 0.3}, {0.1, -0.1, 0.2, -0.2}},
      {{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, -0.5, 0.0}},
      {{-0.3, 0.6, -0.4}, {-0.1, 0.1, -0.4}, {-0.15, 0.15, 0.1, -0.1}},
  };
  for (const auto& c : cases) {
    const auto ik = inverse_3d(c.twist, c.gyro, c.s, kExample);
    const auto fk = forward_3d(to_measurements(ik), c.gyro, c.s, kExample);
    // The contact-angle prior biases the solution by ~(prior/speed)^2, so the
    // round trip is exact only to ~1e-6.
    EXPECT_TRUE(fk.converged);
    EXPECT_NEAR(fk.v_body.x(), c.twist.vx, 1e-5);
    EXPECT_NEAR(fk.v_body.y(), c.twist.vy, 1e-5);
    EXPECT_NEAR(fk.v_body.z(), 0.0, 1e-5);
    for (int i = 0; i < kNumWheels; ++i) {
      EXPECT_NEAR(fk.contact_angle[i], ik[i].contact_angle, 1e-4) << "wheel " << i;
    }
    EXPECT_LT(fk.residual_rms, 1e-5);
  }
}

TEST(Forward3d, StationaryRoverConvergesToZero) {
  const auto fk = forward_3d(PerWheel<ModuleMeasurement>{}, Gyro{}, SuspensionState{}, kExample);
  EXPECT_TRUE(fk.converged);
  EXPECT_NEAR(fk.v_body.norm(), 0.0, 1e-12);
  for (double eta : fk.contact_angle) {
    EXPECT_NEAR(eta, 0.0, 1e-12);
  }
}

TEST(Forward3d, InconsistentWheelShowsResidual) {
  auto meas = to_measurements(inverse_3d({1.0, 0.0, 0.0}, Gyro{}, SuspensionState{}, kExample));
  meas[kFL].wheel_rate *= 1.5;
  const auto fk = forward_3d(meas, Gyro{}, SuspensionState{}, kExample);
  EXPECT_GT(fk.residual_rms, 0.01);
}
```

Review added `SlipIsNotExplainedAsVerticalMotion`,
`PriorKeepsNearlyStoppedWheelContactAngleNearZero`,
`IterationLimitReportsNotConverged`, `NonFiniteInputIsNotConverged`,
`VzPriorUnderestimatesPitchAboutRearAxle`,
`WheelTurningAgainstItsMotionShowsResidual`,
`ConvergesWithinContactAngleLimitOnNoisyInput` and
`ReturnsStationaryPointOfItsCost`; see `driver/tests/test_kinematics_3d.cpp`.

- [ ] **Step 2: Run to verify it fails**

Run: `pixi run driver-build`
Expected: compile error `use of undeclared identifier 'forward_3d'`.

- [ ] **Step 3: Declare** — add to `kinematics_3d.hpp` after `inverse_3d`:

```cpp
// Terrain-aware FK: body linear velocity and per-wheel contact angles that best
// explain the measured steering angles and wheel rates (least squares), given
// the gyro's full angular velocity and the rocker encoders. Levenberg-Marquardt
// with the exact Hessian, initialised from forward_2d.
//
// Wheel speeds see v_z only through the front/rear and left/right differences
// that rotation causes, and not at all without rotation, so two priors apply:
// - v_z toward zero (g.fk3d_vz_prior_weight): the body pitches and rolls about
//   its origin, as inverse_3d assumes. Without it slip is explained as
//   vertical motion. Real v_z is underestimated, and the contact angles and
//   residual_rms absorb the rest. Example, kRover pitching nose-up at 0.3 rad/s
//   about the rear axle (front wheels climbing a step) while driving at
//   0.3 m/s: the origin moves at (0.3, 0, 0.135) m/s; on noise-free data
//   forward_3d reports (0.32, 0, 0.058), the level rear wheels as descending
//   at 0.27 rad, the front contact angles 0.20 rad low, and residual_rms
//   0.019 m/s.
// - contact angles toward zero (g.fk3d_eta_prior_weight). It only decides the
//   angle of a wheel whose rolling speed times velocity is below about the
//   weight squared, i.e. a wheel stopped to within about 1 mm/s by default.
//   A contact angle is meaningful only when its wheel moves well above encoder
//   noise; for a stopped or slow wheel it follows the noise.
// Contact angles are limited to +-g.fk3d_max_contact_angle. Otherwise a wheel
// turning against its motion (reversed, slipping, miswired) could be explained
// as rolling with an angle near pi, with no residual.
//
// converged: a stationary point was reached, possibly with contact angles at
// the limit (false at the iteration limit or for non-finite input).
// iterations: accepted LM steps. residual_rms: RMS of the 12 wheel-velocity
// rows [m/s], the consistency check. It includes what the priors and the limit
// leave unexplained, so it can be nonzero on consistent data (above). The
// rolling model omits the lateral wheel-center motion noted at inverse_3d, so
// under a body roll rate that motion shows up in v_body and residual_rms.
Fk3dResult forward_3d(const PerWheel<ModuleMeasurement>& meas, const Gyro& gyro,
                      const SuspensionState& s, const RoverGeometry& g = kRover) noexcept;
```

- [ ] **Step 4: Implement** — in `kinematics_3d.cpp` add these includes below `<cmath>`:

```cpp
#include <algorithm>

#include <Eigen/Dense>

#include "rover_driver/kinematics_2d.hpp"
```

add this block right after `namespace rover_driver {`:

```cpp
namespace {

constexpr int kUnknowns = 3 + kNumWheels;               // v_body, contact angles
constexpr int kKinematicRows = 3 * kNumWheels;          // wheel velocity equations
constexpr int kVzPriorRow = kKinematicRows + kNumWheels;  // after contact-angle priors
constexpr int kRows = kVzPriorRow + 1;
using State = Eigen::Matrix<double, kUnknowns, 1>;
using Hessian = Eigen::Matrix<double, kUnknowns, kUnknowns>;
using Residual = Eigen::Matrix<double, kRows, 1>;
using Jacobian = Eigen::Matrix<double, kRows, kUnknowns>;

// Direction of the wheel-center velocity in module coordinates.
Eigen::Vector3d rolling_direction(double contact_angle) noexcept {
  return {std::cos(contact_angle), 0.0, std::sin(contact_angle)};
}

// Per-wheel quantities that do not depend on the unknowns.
struct WheelTerms {
  Eigen::Matrix3d module_rot;  // module frame -> body frame
  Eigen::Vector3d rotation_velocity;  // wheel-center velocity from rotations only
  double rolling_speed;               // R * (wheel rate + fork spin)
};

PerWheel<WheelTerms> wheel_terms(const PerWheel<ModuleMeasurement>& meas,
                                 const Eigen::Vector3d& w_body, const SuspensionState& s,
                                 const RoverGeometry& g) noexcept {
  PerWheel<WheelTerms> terms{};
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelChain c = wheel_chain(i, Eigen::Vector3d::Zero(), w_body, s, g);
    terms[i].module_rot = c.rocker_rot * rot_z(meas[i].steer_angle);
    const double fork_spin = terms[i].module_rot.col(1).dot(c.rocker_omega);
    terms[i].rotation_velocity = c.velocity;
    terms[i].rolling_speed = g.wheel_radius * (meas[i].wheel_rate + fork_spin);
  }
  return terms;
}

// Residual per wheel (module coords): M^T (v + c) - s * [cos eta, 0, sin eta],
// then prior rows eta_weight * eta and vz_weight * v_z.
void evaluate(const State& x, const PerWheel<WheelTerms>& terms, double eta_weight,
              double vz_weight, Residual& r, Jacobian* jacobian) noexcept {
  const Eigen::Vector3d v = x.head<3>();
  if (jacobian != nullptr) {
    jacobian->setZero();
  }
  for (int i = 0; i < kNumWheels; ++i) {
    const WheelTerms& t = terms[i];
    const double eta = x(3 + i);
    r.segment<3>(3 * i) = t.module_rot.transpose() * (v + t.rotation_velocity) -
                          t.rolling_speed * rolling_direction(eta);
    r(kKinematicRows + i) = eta_weight * eta;
    if (jacobian != nullptr) {
      jacobian->block<3, 3>(3 * i, 0) = t.module_rot.transpose();
      jacobian->block<3, 1>(3 * i, 3 + i) =
          t.rolling_speed * Eigen::Vector3d(std::sin(eta), 0.0, -std::cos(eta));
      (*jacobian)(kKinematicRows + i, 3 + i) = eta_weight;
    }
  }
  r(kVzPriorRow) = vz_weight * v.z();
  if (jacobian != nullptr) {
    (*jacobian)(kVzPriorRow, 2) = vz_weight;
  }
}

}  // namespace
```

and after `inverse_3d`:

```cpp
Fk3dResult forward_3d(const PerWheel<ModuleMeasurement>& meas, const Gyro& gyro,
                      const SuspensionState& s, const RoverGeometry& g) noexcept {
  const Eigen::Vector3d w_body(gyro.wx, gyro.wy, gyro.wz);
  const PerWheel<WheelTerms> terms = wheel_terms(meas, w_body, s, g);
  const double eta_prior = g.fk3d_eta_prior_weight;
  const double vz_prior = g.fk3d_vz_prior_weight;
  const double eta_max = g.fk3d_max_contact_angle;

  const Fk2dResult initial = forward_2d(meas, g);
  State x = State::Zero();
  x(0) = initial.twist.vx;
  x(1) = initial.twist.vy;

  Fk3dResult out;
  Residual r;
  Jacobian jacobian;
  double lambda = 1e-3;
  // Each pass tests x for convergence, then takes at most one accepted LM step;
  // out.iterations counts accepted steps.
  while (true) {
    evaluate(x, terms, eta_prior, vz_prior, r, &jacobian);
    const double cost = r.squaredNorm();
    if (!std::isfinite(cost)) {
      break;  // non-finite input: report it, never as converged
    }
    State gradient = jacobian.transpose() * r;
    // Exact Hessian of cost / 2. The residual is linear in v_body, so J^T J
    // lacks only the curvature of each wheel's rolling term along its contact
    // angle. Without it, steps are poor whenever a wheel's rolling speed does
    // not match its motion (slip, or noise on a stopped rover) and the solve
    // crawls to the iteration limit.
    Hessian hessian = jacobian.transpose() * jacobian;
    for (int i = 0; i < kNumWheels; ++i) {
      hessian(3 + i, 3 + i) +=
          terms[i].rolling_speed * r.segment<3>(3 * i).dot(rolling_direction(x(3 + i)));
    }
    // Contact angles live in [-eta_max, eta_max]. One held at its bound by a
    // gradient pointing outward is fixed for this step (projected gradient).
    for (int i = 0; i < kNumWheels; ++i) {
      const int k = 3 + i;
      if ((x(k) >= eta_max && gradient(k) < 0.0) || (x(k) <= -eta_max && gradient(k) > 0.0)) {
        gradient(k) = 0.0;
        hessian.row(k).setZero();
        hessian.col(k).setZero();
        hessian(k, k) = 1.0;
      }
    }
    if (gradient.lpNorm<Eigen::Infinity>() < g.fk3d_tolerance) {
      out.converged = true;
      break;
    }
    if (out.iterations >= g.fk3d_max_iterations) {
      break;
    }

    bool improved = false;
    bool small_step = false;
    while (lambda < 1e12) {
      const Eigen::LLT<Hessian> damped(hessian + lambda * Hessian::Identity());
      if (damped.info() != Eigen::Success) {
        lambda *= 10.0;  // not positive definite yet: the step might not descend
        continue;
      }
      State candidate = x + damped.solve(-gradient);
      for (int i = 0; i < kNumWheels; ++i) {
        candidate(3 + i) = std::clamp(candidate(3 + i), -eta_max, eta_max);
      }
      Residual candidate_r;
      evaluate(candidate, terms, eta_prior, vz_prior, candidate_r, nullptr);
      if (candidate_r.squaredNorm() < cost) {
        small_step = (candidate - x).norm() < g.fk3d_tolerance;
        x = candidate;
        lambda = std::max(lambda * 0.3, 1e-12);
        improved = true;
        break;
      }
      lambda *= 10.0;
    }
    if (!improved) {
      out.converged = true;  // no descent direction left: numerical minimum
      break;
    }
    ++out.iterations;
    if (small_step) {
      out.converged = true;
      break;
    }
  }

  evaluate(x, terms, eta_prior, vz_prior, r, nullptr);
  out.v_body = x.head<3>();
  for (int i = 0; i < kNumWheels; ++i) {
    out.contact_angle[i] = x(3 + i);
  }
  out.residual_rms = std::sqrt(r.head<kKinematicRows>().squaredNorm() / kKinematicRows);
  return out;
}
```

- [ ] **Step 5: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed, 0 tests failed out of 19`.

---

### Task 7: `rover_state` CLI

**Files:**
- Modify: `driver/CMakeLists.txt`
- Create: `driver/tools/rover_state.cpp`
- Extended in review: option table (`number_field`), errors that name the
  rejected option or value, finite numbers only (`nan`, `inf` and out-of-range
  values rejected), `Num` (a value that overflows prints as JSON `null`); CTest
  cases `rover_state_rejects_nan`, `rover_state_rejects_out_of_range`,
  `rover_state_explains_rejection` and `rover_state_json_{3d,2d,overflow}`.
- Extended in the final review: `geometry.wheel_offset`, and `fk.contact_angle`
  in 3D mode.

- [ ] **Step 1: Add the target and a smoke test** — in `driver/CMakeLists.txt`, after the `target_compile_options(rover_driver ...)` line add:

```cmake
add_executable(rover_state tools/rover_state.cpp)
target_link_libraries(rover_state PRIVATE rover_driver)
target_compile_options(rover_state PRIVATE -Wall -Wextra -Wpedantic)
```

and inside `if(BUILD_TESTING)`, after `gtest_discover_tests(rover_driver_tests)`:

```cmake
  add_test(NAME rover_state_runs COMMAND rover_state --mode 3d --vx 0.5 --wz 0.2 --ql 0.1 --dqr -0.2)
  add_test(NAME rover_state_rejects_bad_flag COMMAND rover_state --bogus 1)
  set_tests_properties(rover_state_rejects_bad_flag PROPERTIES WILL_FAIL TRUE)
```

- [ ] **Step 2: Run to verify it fails**

Run: `pixi run driver-build`
Expected: CMake error `Cannot find source file: tools/rover_state.cpp`.

- [ ] **Step 3: Implement** — create `driver/tools/rover_state.cpp`:

```cpp
// Prints the rover kinematic state for one command as JSON (used by viz.py).
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

bool parse(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "-h" || key == "--help") {
      usage(stdout);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      return false;
    }
    const std::string value = argv[++i];
    if (key == "--mode") {
      if (value != "2d" && value != "3d") {
        return false;
      }
      a.mode_3d = value == "3d";
      continue;
    }
    char* end = nullptr;
    const double d = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0') {
      return false;
    }
    if (key == "--vx") a.cmd.vx = d;
    else if (key == "--vy") a.cmd.vy = d;
    else if (key == "--wz") a.cmd.wz = d;
    else if (key == "--gx") a.gx = d;
    else if (key == "--gy") a.gy = d;
    else if (key == "--ql") a.suspension.q_left = d;
    else if (key == "--qr") a.suspension.q_right = d;
    else if (key == "--dql") a.suspension.dq_left = d;
    else if (key == "--dqr") a.suspension.dq_right = d;
    else return false;
  }
  return true;
}

void print_vec(const Eigen::Vector3d& v) {
  std::printf("[%.9g, %.9g, %.9g]", v.x(), v.y(), v.z());
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
  double fk_wz = 0.0;
  double fk_residual = 0.0;
  bool fk_converged = true;
  int fk_iterations = 0;
  if (a.mode_3d) {
    cmds = inverse_3d(a.cmd, gyro, s, g);
    const Fk3dResult fk = forward_3d(to_measurements(cmds), gyro, s, g);
    fk_v = fk.v_body;
    fk_wz = gyro.wz;
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
  std::printf("  \"geometry\": {\"wheel_radius\": %.9g, \"pivot\": [", g.wheel_radius);
  print_vec(g.pivot[kLeft]);
  std::printf(", ");
  print_vec(g.pivot[kRight]);
  std::printf("]},\n");
  std::printf("  \"command\": {\"vx\": %.9g, \"vy\": %.9g, \"wz\": %.9g},\n", a.cmd.vx, a.cmd.vy,
              a.cmd.wz);
  std::printf("  \"gyro\": {\"wx\": %.9g, \"wy\": %.9g, \"wz\": %.9g},\n", gyro.wx, gyro.wy,
              gyro.wz);
  std::printf(
      "  \"suspension\": {\"q_left\": %.9g, \"q_right\": %.9g, \"dq_left\": %.9g, "
      "\"dq_right\": %.9g},\n",
      s.q_left, s.q_right, s.dq_left, s.dq_right);
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
        ", \"steer_angle\": %.9g, \"wheel_rate\": %.9g, \"contact_angle\": %.9g, "
        "\"steer_defined\": %s}%s\n",
        cmds[i].steer_angle, cmds[i].wheel_rate, cmds[i].contact_angle,
        cmds[i].steer_defined ? "true" : "false", i + 1 < kNumWheels ? "," : "");
  }
  std::printf("  ],\n");
  std::printf(
      "  \"fk\": {\"vx\": %.9g, \"vy\": %.9g, \"vz\": %.9g, \"wz\": %.9g, "
      "\"residual_rms\": %.9g, \"converged\": %s, \"iterations\": %d}\n}\n",
      fk_v.x(), fk_v.y(), fk_v.z(), fk_wz, fk_residual, fk_converged ? "true" : "false",
      fk_iterations);
  return 0;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `pixi run driver-test`
Expected: `100% tests passed out of 46` (38 library tests from Tasks 1-6 and their review fixes, 8 CLI tests; the final review adds 2 library tests).

- [ ] **Step 5: Check the JSON parses**

Run: `pixi run bash -c "driver/build/rover_state --mode 3d --vx 0.5 --wz 0.3 --ql 0.1 --dql -0.3 | python -m json.tool | head -20"`
Expected: pretty-printed JSON starting with `"mode": "3d"`, no parse error.

---

### Task 8: `viz.py`

**Files:**
- Create: `driver/tools/viz.py`
- Create (added in review): `driver/tests/test_viz.py`, run by the CTest
  `viz_py` in `driver/CMakeLists.txt` (47 tests after this task)
- Extended in review: `ONLY_3D` and `mark_ignored_sliders` (sliders that 2d
  mode ignores are greyed), `turning_center` and a top view that grows to show
  it, labels placed off the chassis, `rover_state`'s failure reason shown
  instead of a traceback.
- Extended in the final review: `query` stops with "command too large" when
  `rover_state` prints `null`; velocity arrows drawn above the outline,
  turning-center lines and labels; no contact marker where η is undefined;
  lower 3D viewing angle (`make_view_axes`); the text panel shows per mode only
  what FK estimates, marks 3D wz as the gyro's and compares IK and FK η.

- [ ] **Step 1: Implement** — create `driver/tools/viz.py`:

```python
#!/usr/bin/env python3
"""Interactive 2D/3D view of the rover driver kinematics.

Calls the C++ `rover_state` tool for every slider change and draws a top view
and a 3D view. `--snapshot out.png` renders once and exits (no window).
"""
import argparse
import json
import os
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RadioButtons, Slider

EXE = Path(
    os.environ.get(
        "ROVER_STATE_EXE", Path(__file__).resolve().parents[1] / "build" / "rover_state"
    )
)

# (flag, label, min, max, initial)
SLIDERS = [
    ("vx", "vx [m/s]", -1.5, 1.5, 0.5),
    ("vy", "vy [m/s]", -1.5, 1.5, 0.0),
    ("wz", "wz [rad/s]", -2.0, 2.0, 0.3),
    ("gx", "gyro wx [rad/s]", -1.0, 1.0, 0.0),
    ("gy", "gyro wy [rad/s]", -1.0, 1.0, 0.0),
    ("ql", "q left [rad]", -0.5, 0.5, 0.0),
    ("qr", "q right [rad]", -0.5, 0.5, 0.0),
    ("dql", "dq left [rad/s]", -1.0, 1.0, 0.0),
    ("dqr", "dq right [rad/s]", -1.0, 1.0, 0.0),
]
ARROW_SECONDS = 0.5  # velocity arrows show the displacement over this time
SIDE = {"FL": 0, "RL": 0, "FR": 1, "RR": 1}


def query(mode, values):
    cmd = [str(EXE), "--mode", mode]
    for flag, value in values.items():
        cmd += [f"--{flag}", f"{value:.6f}"]
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def draw_top(ax, s):
    ax.clear()
    ax.set_title(f"Top view ({s['mode']})")
    ax.set_aspect("equal")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.grid(alpha=0.3)
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")

    wheels = {w["name"]: w for w in s["wheels"]}
    outline = np.array([wheels[n]["center"][:2] for n in ("FL", "FR", "RR", "RL", "FL")])
    ax.plot(outline[:, 0], outline[:, 1], color="0.6")
    for p in s["geometry"]["pivot"]:
        ax.plot(p[0], p[1], "ks", ms=5)

    r = s["geometry"]["wheel_radius"]
    for w in s["wheels"]:
        c = np.array(w["center"][:2])
        f = np.array(w["forward"][:2])
        a = np.array(w["axle"][:2])
        corners = np.array(
            [c + r * f + 0.25 * r * a, c + r * f - 0.25 * r * a,
             c - r * f - 0.25 * r * a, c - r * f + 0.25 * r * a]
        )
        ax.fill(corners[:, 0], corners[:, 1],
                color="C0" if w["steer_defined"] else "0.7", alpha=0.8)
        v = np.array(w["velocity"][:2]) * ARROW_SECONDS
        if np.linalg.norm(v) > 1e-6:
            ax.arrow(c[0], c[1], v[0], v[1], width=0.01, color="C3",
                     length_includes_head=True)
        ax.annotate(
            f"{w['name']}\nψ={np.degrees(w['steer_angle']):+.1f}°\n"
            f"θ̇={w['wheel_rate']:+.2f} rad/s\nη={np.degrees(w['contact_angle']):+.1f}°",
            c + np.array([0.08, 0.08]), fontsize=7,
        )

    cmd = s["command"]
    if abs(cmd["wz"]) > 1e-6:
        icr = np.array([-cmd["vy"] / cmd["wz"], cmd["vx"] / cmd["wz"]])
        ax.plot(icr[0], icr[1], "mx", ms=10)
        for w in s["wheels"]:
            ax.plot([w["center"][0], icr[0]], [w["center"][1], icr[1]], "m:", lw=0.8)


def draw_3d(ax, s):
    ax.clear()
    ax.set_title("3D view (body level; rockers at q)")
    r = s["geometry"]["wheel_radius"]
    pivots = np.array(s["geometry"]["pivot"])
    centers = {w["name"]: np.array(w["center"]) for w in s["wheels"]}

    xs = [c[0] for c in centers.values()]
    x0, x1 = min(xs), max(xs)
    zb = pivots[:, 2].mean()
    yl, yr = pivots[0, 1], pivots[1, 1]
    body = np.array([[x1, yl, zb], [x1, yr, zb], [x0, yr, zb], [x0, yl, zb], [x1, yl, zb]])
    ax.plot(*body.T, color="0.4", lw=2)

    t = np.linspace(0.0, 2.0 * np.pi, 40)
    for w in s["wheels"]:
        c = centers[w["name"]]
        p = pivots[SIDE[w["name"]]]
        ax.plot(*np.array([p, c]).T, color="C1", lw=3)
        f = np.array(w["forward"])
        u = np.array(w["up"])
        rim = c[:, None] + r * (np.outer(f, np.cos(t)) + np.outer(u, np.sin(t)))
        ax.plot(*rim, color="C0" if w["steer_defined"] else "0.6")
        eta = w["contact_angle"]
        contact = c + r * (np.sin(eta) * f - np.cos(eta) * u)
        ax.plot(*np.array([c, contact]).T, color="C2")
        ax.scatter(*contact, color="C2", s=12)
        v = np.array(w["velocity"]) * ARROW_SECONDS
        if np.linalg.norm(v) > 1e-6:
            ax.quiver(*c, *v, color="C3")

    gx, gy = np.meshgrid([-1.0, 1.0], [-1.0, 1.0])
    ax.plot_surface(gx, gy, np.zeros_like(gx), alpha=0.08, color="0.5")
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(0.0, 1.0)
    ax.set_box_aspect((2, 2, 1))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")


def summary(s):
    cmd, fk = s["command"], s["fk"]
    return (
        f"command:  vx={cmd['vx']:+.3f}  vy={cmd['vy']:+.3f}  wz={cmd['wz']:+.3f}\n"
        f"FK est.:  vx={fk['vx']:+.3f}  vy={fk['vy']:+.3f}  vz={fk['vz']:+.3f}  "
        f"wz={fk['wz']:+.3f}\n"
        f"residual={fk['residual_rms']:.2e} m/s  converged={fk['converged']}  "
        f"iterations={fk['iterations']}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--snapshot", type=Path, help="render once to this PNG and exit")
    parser.add_argument("--mode", choices=["2d", "3d"], default="3d")
    for flag, _, _, _, initial in SLIDERS:
        parser.add_argument(f"--{flag}", type=float, default=initial)
    args = parser.parse_args()
    if not EXE.exists():
        raise SystemExit(f"{EXE} not found - run `pixi run driver-build` first")
    if args.snapshot:
        plt.switch_backend("Agg")

    fig = plt.figure(figsize=(13, 8))
    ax_top = fig.add_axes([0.04, 0.38, 0.42, 0.58])
    ax_3d = fig.add_axes([0.5, 0.38, 0.48, 0.58], projection="3d")
    text = fig.text(0.55, 0.30, "", family="monospace", fontsize=9, va="top")
    values = {flag: getattr(args, flag) for flag, *_ in SLIDERS}
    state = {"mode": args.mode}

    def redraw(*_):
        s = query(state["mode"], values)
        draw_top(ax_top, s)
        draw_3d(ax_3d, s)
        text.set_text(summary(s))
        fig.canvas.draw_idle()

    widgets = []  # keep references so the widgets stay alive
    if not args.snapshot:
        for k, (flag, label, lo, hi, _) in enumerate(SLIDERS):
            slider_ax = fig.add_axes([0.14, 0.30 - k * 0.03, 0.3, 0.02])
            slider = Slider(slider_ax, label, lo, hi, valinit=values[flag])

            def on_change(value, flag=flag):
                values[flag] = value
                redraw()

            slider.on_changed(on_change)
            widgets.append(slider)
        radio = RadioButtons(fig.add_axes([0.55, 0.05, 0.08, 0.12]), ("2d", "3d"),
                             active=0 if args.mode == "2d" else 1)

        def on_mode(label):
            state["mode"] = label
            redraw()

        radio.on_clicked(on_mode)
        widgets.append(radio)

    redraw()
    if args.snapshot:
        fig.savefig(args.snapshot, dpi=110)
        return
    plt.show()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Render snapshots headlessly**

Run:
```bash
pixi run bash -c "python driver/tools/viz.py --mode 2d --snapshot driver/build/viz_2d.png && python driver/tools/viz.py --mode 3d --vx 0.4 --wz 0.3 --ql 0.15 --qr -0.15 --dql -0.4 --snapshot driver/build/viz_3d.png"
```
Expected: exit code 0, two PNGs written.

- [ ] **Step 3: Inspect the snapshots** — open both PNGs. Check: in 2D, wheel headings point perpendicular to the dotted lines to the ICR (magenta ×); FK line equals the command; residual ≈ 0. In 3D, left rocker front is lowered (q_left > 0), FL shows a positive η (green contact marker ahead of the wheel's bottom), right rocker raised at the front.

---

### Task 9: Final verification

- [ ] **Step 1: Clean build and full test run**

Run: `rm -rf driver/build && pixi run driver-test`
Expected: `100% tests passed out of 49` (40 library tests, 8 CLI tests, `viz_py`), no compiler warnings in the build output.

- [ ] **Step 2: Launch the interactive viewer** (manual, needs a display)

Run: `pixi run driver-viz`
Expected: a window with top/3D views, sliders and a 2d/3d toggle; moving sliders updates the views.
