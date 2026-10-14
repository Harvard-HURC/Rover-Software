# Rover driver kinematics — design

Date: 2026-09-25
Status: approved design (approach A), pending spec review

## Goal

A C++ library in `driver/` that converts between body motion and swerve-module
motion for our 4-wheel swerve rover with a rocker differential, in two
fidelity levels:

- **2D** — flat-ground swerve kinematics in the body frame.
- **3D** — terrain-aware kinematics through the rocker chain, after
  Kelly & Seegmiller (2015) velocity propagation and Toupet et al. (2020)
  contact-angle wheel-speed correction.

Plus a simple interactive 2D (top view) and 3D visualization of the rover
state to sanity-check the math before real geometry is known.

Plain C++ functions only — no ROS node, no ros2_control controller yet.

## Out of scope (later layers)

Flip optimization, alignment gate / cosine scaling, wheel-speed
desaturation, command timeout, yaw-rate feedback, steering/drive coupling
compensation, slip-based traction control, motor-level loops (these live in
the motor controllers).

## Rover model

- Rigid body with two rockers (left, right) pivoting about the body **+y**
  axis at fixed pivot points. A differential links them, but each rocker has
  its own encoder and the kinematics uses each measured angle directly
  (so differential backlash does not corrupt the chain).
- Each rocker carries a front and a rear swerve module.
- Each module's steering axis is the rocker's **z** axis and passes through
  the wheel center (zero steering offset).
- Wheels: FL, FR, RL, RR (indices 0..3). Side: FL, RL → left; FR, RR → right.

## Conventions

- Body frame: ROS REP-103 — x forward, y left, z up. The body origin is the
  point whose velocity is commanded/estimated; it is implied by the config
  numbers (pivot positions are given relative to it).
- Rocker angle `q_s` (s ∈ {L, R}): rotation of rocker s relative to the body
  about +y (right-hand rule, so positive q lowers the front of that rocker).
  `R_s = Ry(q_s)`.
- Steering angle `ψ_i`: rotation of module i about the rocker z axis;
  0 = wheel rolls along the rocker x axis; positive counter-clockwise seen
  from above. Module frame `M_i = R_s · Rz(ψ_i)`; axle `y_i = M_i ŷ`.
- Wheel rate `θ̇_i` [rad/s]: wheel spin relative to its steering fork;
  positive = rolling toward module +x.
- Contact angle `η_i`: angle of the wheel-center velocity within the wheel
  plane, measured from module +x toward module +z (positive = climbing).
- Angles in radians, lengths in meters, time in seconds.

## Configuration (`config.hpp`)

A `RoverGeometry` aggregate and one `constexpr RoverGeometry kRover` holding
our rover's constants:

- `wheel_radius`
- `pivot[2]` — rocker pivot positions in the body frame (L, R).
- `wheel_offset[4]` — wheel-center position relative to its rocker pivot, in
  the rocker frame (equals the body-frame offset when q = 0).
- `steer_undefined_speed`, `fk3d_eta_prior_weight`, `fk3d_vz_prior_weight`,
  `fk3d_max_contact_angle`, `fk3d_max_iterations`, `fk3d_tolerance` —
  threshold/solver tuning.

Every kinematics function takes `const RoverGeometry& g = kRover`, so
production code uses the constant configuration and tests can supply their
own geometry. Numbers in `kRover` are placeholders until the mechanical team
provides geometry. Nominal 2D wheel positions `(x_i, y_i)` are
`pivot[s] + wheel_offset[i]` (x, y components) at q = 0.

## Types (`types.hpp`)

Fixed-size, no heap allocation:

- `BodyTwist2d { vx, vy, wz }`
- `Gyro { wx, wy, wz }` — body angular velocity from the IMU.
- `SuspensionState { q_left, q_right, dq_left, dq_right }`
- `ModuleCommand { steer_angle, wheel_rate, steer_defined }`
- `ModuleMeasurement { steer_angle, wheel_rate }`
- `ModuleCommand3d` = `ModuleCommand` + `contact_angle`
- `Fk2dResult { BodyTwist2d twist; double residual_rms; }`
- `Fk3dResult { Eigen::Vector3d v_body; std::array<double,4> contact_angle;
  double residual_rms; bool converged; int iterations; }` (`iterations` =
  accepted LM steps)
- `std::array<…, 4>` for per-module data.

## 2D kinematics (`kinematics_2d.{hpp,cpp}`)

**Inverse** `inverse_2d(BodyTwist2d) → array<ModuleCommand,4>`:
`v_i = (vx − wz·y_i, vy + wz·x_i)`, `ψ_i = atan2(v_iy, v_ix)`,
`θ̇_i = |v_i| / R`. If `|v_i| < g.steer_undefined_speed`: `steer_defined = false`,
`ψ_i = 0`, `θ̇_i = 0`.

**Forward** `forward_2d(array<ModuleMeasurement,4>) → Fk2dResult`:
each module gives `R·θ̇_i·[cos ψ_i, sin ψ_i] = [vx − wz·y_i, vy + wz·x_i]`;
stack to an 8×3 linear system and solve by least squares (Kelly & Seegmiller
eq. 75 with zero steering offset). `residual_rms` = RMS of the 8 equation
residuals [m/s] — a slip / inconsistency indicator.

## 3D kinematics (`kinematics_3d.{hpp,cpp}`)

Shared chain (transport theorem, Kelly & Seegmiller eq. 4), for wheel i on
side s:

```
a_i^b = R_s · g.wheel_offset[i]                 // pivot → wheel center, body coords
ω_s   = ω_b + dq_s · ŷ                          // rocker angular velocity
v_i   = v_b + ω_b × g.pivot[s] + ω_s × a_i^b    // wheel-center velocity
ζ_i   = y_i · ω_s                               // fork rotation about the axle
rolling (no slip):  v_i = R·(θ̇_i + ζ_i)·(cos η_i · x_i + sin η_i · z_i)
```

**Inverse** `inverse_3d(BodyTwist2d cmd, Gyro, SuspensionState)
→ array<ModuleCommand3d,4>`:
`v_b = (vx, vy, 0)` (body vertical velocity assumed zero, as in Toupet),
`ω_b = (gyro.wx, gyro.wy, cmd.wz)`. Compute `v_i`, express in the rocker
frame `u = R_sᵀ v_i`, `h = hypot(u_x, u_y)`:
`ψ_i = atan2(u_y, u_x)`, `η_i = atan2(u_z, h)`, `θ̇_i = |v_i|/R − ζ_i`.
If `h < g.steer_undefined_speed`: `steer_defined = false`, `ψ_i = 0`,
`θ̇_i = 0`, `η_i = 0`.

**Forward** `forward_3d(array<ModuleMeasurement,4>, Gyro, SuspensionState)
→ Fk3dResult`:
unknowns `x = [v_b (3), η_0..η_3]`, `ω_b` taken from the gyro. Residual per
module (3 rows, module coordinates):
`e_i = M_iᵀ (v_b + c_i) − s_i · [cos η_i, 0, sin η_i]ᵀ`, with
`c_i = ω_b × g.pivot[s] + ω_s × a_i^b`, `s_i = R·(θ̇_i + ζ_i)`; plus prior rows
`fk3d_eta_prior_weight · η_i` so η stays defined when a wheel is stationary,
and `fk3d_vz_prior_weight · v_z`. Contact angles are bounded:
`|η_i| ≤ fk3d_max_contact_angle < π/2`.
Solve with Levenberg-Marquardt, analytic Jacobian, initialised from
`forward_2d` (`v_b = (vx, vy, 0)`, η = 0). The damped matrix is the exact
Hessian: `JᵀJ` plus `s_i · e_i · [cos η_i, 0, sin η_i]ᵀ` on each η diagonal
entry (the residual is linear in `v_b`, so this is the only second-order
term). A step must keep `Hessian + λI` positive definite. λ grows up to
`1e12 ·` the Hessian's largest entry, by which `Hessian + λI` is positive
definite (Gershgorin). When no λ up to that cap reduces the cost, x counts as
converged only if every gradient entry `g_k = J_k · r` is at most
`1e-6 · |J_k| |r|` (where rounding stops the steps at a minimum it is below
1e-7).
An η at its bound whose gradient points outward stays fixed for that step
(projected gradient), and candidate η are clipped to the bounds.
`residual_rms` = RMS of the 12 kinematic rows (prior rows excluded).

The cost is non-convex in η, and LM returns a stationary point, not
necessarily the lowest one:

- On inconsistent data (a reversed or slipping wheel) it can stop in a local
  minimum whose v_b is far from the best fit's. Its `residual_rms` is large,
  so a large residual marks the result untrustworthy.
- The `forward_2d` start ignores the rocker angles, which suits rockers far
  from vertical. On random consistent motions (20 000 per angle) it found
  the true motion for |q| ≤ 1.25 rad. From about 1.3 rad, with the rockers
  turned opposite ways, some ended in a wrong minimum (about 6 in 10 000 near
  π/2), with v_b off by up to 1.8 m/s and `residual_rms` as small as
  0.001 m/s. Real rockers stay far below that.

The v_z prior, the bound and the exact Hessian were added in the Task 6
review; the relative damping cap, the stall check and the NaN outputs for a
non-finite cost (see Error handling) in the final review. The v_z prior weight still needs
design-owner sign-off. Reasons:

- **v_z prior.** Wheel speeds observe v_z only through the front/rear and
  left/right speed differences that rotation causes, and not at all without
  rotation. Without the prior, slip is explained as vertical motion. Example:
  10 % slip on one wheel at 1 m/s while pitching at 0.01 rad/s gives
  v = (0.17, 0, 0.96) (test geometry). Like `inverse_3d`, the prior assumes the body pitches
  and rolls about its origin. Its weight is the wheel-velocity noise divided
  by the expected spread of v_z. The cost: when the body pivots elsewhere,
  v_z is underestimated and the difference goes into the contact angles and
  `residual_rms`. Example: kRover at 0.3 m/s pitching nose-up at 0.3 rad/s
  about the rear axle (front wheels climbing a step), noise-free:

  | | v | η | residual |
  |---|---|---|---|
  | truth | (0.3, 0, 0.135) | (0.81, 0.81, 0, 0) | 0 |
  | weight 1 | (0.32, 0, 0.058) | (0.61, 0.61, −0.27, −0.27) | 0.019 |
  | weight 0.3 | v_z = 0.12 | rear −0.06 | 0.004 |

  With sensor noise the order reverses (5 % wheel rate, 0.02 rad steering,
  0.01 rad/s gyro). Over random motions pivoting about random ground points,
  weight 1 had the smallest errors among weights 1, 0.5, 0.3 and 0.1 in v_z,
  horizontal velocity and η. The 95th percentile of |Δv_z| was 0.068, 0.076,
  0.115 and 0.318 m/s. That is why the default is 1.
- **Contact-angle bound.** The residual is unchanged under
  `(η_i + π, s_i → −s_i)`. Without a bound, a wheel turning against its motion
  (reversed, slipping, miswired) is explained with η ≈ π and no residual. At
  |η| = π/2 the ambiguity remains, since `s·ẑ = (−s)(−ẑ)`, so the bound needs
  a margin. kRover uses 1.4 rad: the contact angle of a step 0.83 R high met
  with the rocker level (`acos(1 − h/R)`).
- **Relative damping cap and stall check.** The η curvature grows as
  `(R·θ̇)²`. With a fixed cap of 1e12, at rates around 1e7 rad/s the damped
  Hessian stayed indefinite up to the cap, and a stall was always reported as
  converged, so the start was reported as converged after 0 steps. With the
  relative cap such data converge up to about 1e10 rad/s; beyond that the η
  rows outweigh the velocity rows by more than double precision resolves, the
  steps stall short of a minimum, and the stall check reports it as not
  converged. On 160 000 random frames at normal rates the results are
  bit-identical to the fixed cap's.
- **Exact Hessian.** Gauss-Newton uses `JᵀJ` only. When a wheel's rolling
  speed disagrees with its motion (slip, or noise on a stopped rover), its η
  steps are poor and convergence is linear: about 10 % of noisy standstill
  frames hit the 50-iteration limit. With the exact term, none of 100 000
  noisy, stopped, exact or random frames hit it (at most 36 iterations).

## Visualization (`driver/tools/`)

- `rover_state` (C++ CLI): flags `--mode 2d|3d --vx --vy --wz --gx --gy
  --ql --qr --dql --dqr`; runs IK, then FK on the IK output (round trip), and
  prints one JSON object: config geometry (wheel radius, pivots, wheel
  offsets), wheel-center positions for the given rocker angles, module
  commands (ψ, θ̇, η, steer_defined), per-wheel velocity vectors, FK result
  (in 3D mode with the contact angles; its wz is the gyro's). A value that
  overflows prints as `null`; `viz.py` then stops with "command too large".
- `viz.py` (Python, matplotlib, from the pixi env): window with sliders for
  every CLI flag and a 2D/3D toggle; on change it calls `rover_state` and
  redraws:
  - **Top view**: body outline, pivots, wheels drawn as oriented rectangles
    at ψ, wheel velocity arrows (drawn on top), instantaneous center of
    rotation of the commanded twist (both modes, whenever wz ≠ 0).
  - **3D view**: body outline (a rectangle at pivot height), rockers as lines
    pivot→wheel centers at q, wheels as circles in their module plane,
    contact-direction markers at η (only where steering, and so η, is
    defined).
  - Text panel: commanded vs FK-estimated twist and residual; in 3D mode also
    IK vs FK contact angles, and FK's convergence and iterations.
- Body attitude is drawn level (rocker angles are relative to the body).

## Build & tasks

CMake project in `driver/` (C++17; Eigen3 and GTest from the pixi env),
targets: `rover_driver` (static library), `rover_driver_tests`,
`rover_state`. Sources: `driver/include/rover_driver/*.hpp`,
`driver/src/*.cpp`, `driver/tests/*.cpp`, `driver/tools/`. Add pixi tasks `driver-build`, `driver-test`, `driver-viz`.

## Error handling

Functions are pure and `noexcept`; no exceptions, no allocation in the
kinematics. Non-finite inputs propagate to non-finite outputs (caller's
responsibility). `forward_3d` reports `converged = false` if it hits the
iteration limit or its cost is not finite (non-finite input, or input so
large that it overflows); in the latter case `v_body`, the contact angles and
`residual_rms` are NaN, also when only the gyro or the rocker inputs are bad.

## Tests (GoogleTest, written before implementation)

1. 2D IK: straight → all ψ = 0, equal rates; strafe left → ψ = π/2;
   turn in place → ψ tangent to the wheel radius; worked example
   (wheels at ±0.5, ±0.4; vx = 1, wz = 0.5 → FL 17.4°, 0.838 m/s).
2. 2D IK: zero command → `steer_defined = false`.
3. 2D FK: round-trips 2D IK; residual ≈ 0; inconsistent measurements →
   residual > 0.
4. 3D IK with zero gyro, zero rocker angles/rates equals 2D IK.
5. 3D chain velocity equals the finite-difference derivative of wheel-center
   positions under random body/rocker motion (Kelly's verification method).
6. 3D IK, FL wheel climbing (left rocker rotating nose-up): FL gets
   η > 0 and a larger rate than FR.
7. 3D FK round-trips 3D IK: recovers `v_b = (vx, vy, 0)` and η; residual ≈ 0.
8. Static rocker angles, zero rates and zero gyro, pure forward command vx:
   - `q_L = q_R = q`: every module ψ = 0, θ̇ = vx/R, η = q.
   - `q_L = q, q_R = −q` (static twist): ψ = 0, θ̇ = vx/R on all modules,
     η = q on the left, −q on the right.
