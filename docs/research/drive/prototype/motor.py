"""DC motor + gearbox + per-wheel PI speed controller (research prototype).

All quantities are referred to the wheel (gearbox output) unless named *_m.
State per wheel: output-referred rotor speed w_o, gearbox windup d = theta_o - theta_wheel,
PI integrator, filtered speed measurement, ramped setpoint.

    J_r * dw_o/dt = N*eta*kt*i - tau_shaft - tau_fric(w_o)
    i   = clamp((u - ke*N*w_o)/R, -I_lim, I_lim)       (controller current limit)
    u   = clamp(Kp*e + Ki*int(e), -V, V)               (PI on measured motor speed)
    tau_shaft = k_s*deadzone(d, b/2) + c_s*(w_o - w_wheel)   (only when engaged)
"""
import math

DEFAULTS = dict(
    V=24.0,          # battery [V]
    R=0.46,          # winding resistance [ohm]           (Husky motor, arXiv 2110.00323 Table I)
    kt=0.0445,       # torque constant [N m/A]           (Husky)
    ke=0.0445,       # back-EMF constant [V s/rad]       (= kt in SI)
    N=50.0,          # gear ratio
    eta=0.80,        # gearbox efficiency (2-3 stage planetary)
    J_m=1.2e-5,      # rotor inertia [kg m^2]
    I_lim=20.0,      # controller current limit [A]
    i_free=1.0,      # no-load current [A] -> motor Coulomb friction kt*i_free
    b_out=0.05,      # output viscous friction [N m s/rad]
    k_s=1500.0,      # driveline torsional stiffness at the wheel [N m/rad]
    c_s=2.0,         # driveline damping [N m s/rad]
    backlash=math.radians(1.5),  # total backlash at the output [rad]
    Kp=4.0,          # [V/(rad/s)] on output speed
    Ki=40.0,         # [V/rad]
    tau_meas=0.005,  # speed measurement filter [s]
    accel=8.0,       # setpoint ramp [rad/s^2] at the wheel (0 = none)
    w_max=10.0,      # setpoint clamp [rad/s]
    substeps=4,      # internal integration substeps per physics step
)


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Wheel:
    def __init__(self, p):
        self.p = p
        self.w_o = 0.0
        self.d = 0.0
        self.integ = 0.0
        self.w_meas = 0.0
        self.sp = 0.0
        self.i = 0.0
        self.tau = 0.0

    def step(self, setpoint, w_wheel, dt):
        p = self.p
        J_r = p["J_m"] * p["N"] ** 2
        # setpoint ramp
        target = _clamp(setpoint, -p["w_max"], p["w_max"])
        if p["accel"] > 0:
            dmax = p["accel"] * dt
            self.sp += _clamp(target - self.sp, -dmax, dmax)
        else:
            self.sp = target
        # measurement filter (motor encoder, output-referred)
        a = dt / (p["tau_meas"] + dt)
        self.w_meas += a * (self.w_o - self.w_meas)
        e = self.sp - self.w_meas
        u_unsat = p["Kp"] * e + p["Ki"] * self.integ
        u = _clamp(u_unsat, -p["V"], p["V"])
        # anti-windup: integrate only if not saturated in the same direction
        h = dt / p["substeps"]
        tau_sum = 0.0
        for _ in range(p["substeps"]):
            i = (u - p["ke"] * p["N"] * self.w_o) / p["R"]
            i = _clamp(i, -p["I_lim"], p["I_lim"])
            tau_m = p["N"] * p["eta"] * p["kt"] * i
            # Coulomb friction of the motor (no-load current), smoothed near zero
            tf = p["N"] * p["kt"] * p["i_free"] * math.tanh(self.w_o / 0.05) + p["b_out"] * self.w_o
            half = p["backlash"] / 2
            if self.d > half:
                ts = p["k_s"] * (self.d - half) + p["c_s"] * (self.w_o - w_wheel)
                ts = max(ts, 0.0)
            elif self.d < -half:
                ts = p["k_s"] * (self.d + half) + p["c_s"] * (self.w_o - w_wheel)
                ts = min(ts, 0.0)
            else:
                ts = 0.0
            self.w_o += h * (tau_m - ts - tf) / J_r
            self.d += h * (self.w_o - w_wheel)
            tau_sum += ts
        self.i = i
        if not ((u_unsat > p["V"] and e > 0) or (u_unsat < -p["V"] and e < 0) or
                (abs(i) >= p["I_lim"] and i * e > 0)):
            self.integ += e * dt
        self.tau = tau_sum / p["substeps"]
        return self.tau


class Drivetrain:
    def __init__(self, params, dt=0.001):
        self.p = dict(DEFAULTS)
        self.p.update(params)
        self.dt = dt
        self.wheels = [Wheel(self.p) for _ in range(4)]
        self.last_current = [0.0] * 4
        self.last_tau = [0.0] * 4

    def step(self, setpoints, wheel_speeds):
        tau = [wh.step(s, w, self.dt) for wh, s, w in zip(self.wheels, setpoints, wheel_speeds)]
        self.last_current = [wh.i for wh in self.wheels]
        self.last_tau = tau
        return tau
