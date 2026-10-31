"""Extra Pegasus vehicle models: rotor lag and wind (mean + turbulence + discrete gusts).

Isaac Python only (imports Pegasus). Used by flight/pegasus_fly.py; settings in config/flight.yaml
`pegasus.motor` and `pegasus.wind` (a flight can override `wind:`).
"""

from __future__ import annotations

import numpy as np
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from scipy.spatial.transform import Rotation


class LaggedThrustCurve(QuadraticThrustCurve):
    """Quadratic thrust curve whose rotor speed follows the command through a first-order lag.

    Pegasus' curve applies the commanded speed instantly. Real motors + ESCs + props spin up in tens
    of ms and spin down more slowly (no active braking): omega_dot = (omega_cmd - omega) / tau, with
    tau_up when accelerating and tau_down when decelerating.
    """

    def __init__(self, tau_up: float, tau_down: float, config: dict | None = None):
        super().__init__(config or {})
        self.tau_up, self.tau_down = float(tau_up), float(tau_down)

    def update(self, state, dt: float):
        rolling_moment = 0.0
        for i in range(self._num_rotors):
            cmd = float(np.clip(self._input_reference[i], self.min_rotor_velocity[i], self.max_rotor_velocity[i]))
            tau = self.tau_up if cmd > self._velocity[i] else self.tau_down
            # Exact discretisation of the first-order lag over dt.
            self._velocity[i] = cmd + (self._velocity[i] - cmd) * np.exp(-dt / max(tau, 1e-6))
            self._force[i] = self._rotor_constant[i] * self._velocity[i] ** 2
            rolling_moment += self._rolling_moment_coefficient[i] * self._velocity[i] ** 2 * self._rot_dir[i]
        self._rolling_moment = rolling_moment
        return self._force, self._velocity, self._rolling_moment


class Wind:
    """Wind velocity (world ENU, m/s) as a function of sim time.

    mean_mps:              constant wind vector
    turbulence_sigma_mps:  RMS of the turbulence per horizontal axis (vertical gets half)
    length_scale_m:        turbulence length scale L; with airspeed V the correlation time is L / V
                           (first-order Dryden-like forming filter = Ornstein-Uhlenbeck per axis)
    gusts:                 list of {t_s, duration_s, peak_mps: [e, n, u]} 1-cos discrete gusts; t_s is
                           flight time (the reference clock: 0 = start of the tracked trajectory)
    """

    def __init__(self, cfg: dict, seed: int = 0):
        self.mean = np.asarray(cfg.get("mean_mps", [0.0, 0.0, 0.0]), float)
        sig = float(cfg.get("turbulence_sigma_mps", 0.0))
        self.sigma = np.array([sig, sig, 0.5 * sig])
        self.L = float(cfg.get("length_scale_m", 50.0))
        self.gusts = cfg.get("gusts", [])
        self.rng = np.random.default_rng(cfg.get("seed", seed))
        self.turb = np.zeros(3)

    def step(self, t: float | None, dt: float, airspeed: float) -> np.ndarray:
        if self.sigma.any():
            tc = self.L / max(airspeed, 1.0)                      # correlation time [s]
            a = np.exp(-dt / tc)
            self.turb = a * self.turb + self.sigma * np.sqrt(1 - a * a) * self.rng.standard_normal(3)
        w = self.mean + self.turb
        for g in self.gusts if t is not None else []:
            s = (t - g["t_s"]) / g["duration_s"]
            if 0.0 <= s <= 1.0:
                w = w + 0.5 * (1 - np.cos(2 * np.pi * s)) * np.asarray(g["peak_mps"], float)
        return w


class WindDrag(LinearDrag):
    """Pegasus' linear body drag, on the velocity relative to the air instead of the ground."""

    def __init__(self, drag_coefficients, wind: Wind):
        super().__init__(drag_coefficients)
        self.wind = wind
        self.t_flight = None                                       # set by the flight script (gust clock)
        self.current = np.zeros(3)                                 # last wind sample, for logging

    def update(self, state, dt: float):
        v = np.asarray(state.linear_velocity, float)
        self.current = self.wind.step(self.t_flight, dt, float(np.linalg.norm(v - self.wind.mean)))
        air_body = Rotation.from_quat(state.attitude).inv().apply(v - self.current)
        self._drag_force = -np.dot(self._drag_coefficients, air_body)
        return self._drag_force


def lockstep_px4_backend():
    """Pegasus' PX4 backend, fixed for physics running faster than the IMU.

    Pegasus only exchanges data with PX4 on physics steps that have a new IMU sample, but advances the
    PX4 clock by that one step's dt. With 1 kHz physics and a 250 Hz IMU, PX4's clock ran at 1/4
    speed, its actuator outputs (rate-limited in PX4 time) stopped arriving every sample, and the
    lockstep wait for them never returned. Here the dt of the skipped steps is carried forward.
    """
    from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend

    class LockstepPX4Backend(PX4MavlinkBackend):
        _pending_dt = 0.0

        def update(self, dt: float):
            self._pending_dt += dt
            if not self._received_first_hearbeat:
                self._pending_dt = 0.0
                return super().update(dt)
            if self._sensor_data.received_first_imu and not self._sensor_data.new_imu_data:
                return
            dt_all, self._pending_dt = self._pending_dt, 0.0
            return super().update(dt_all)

    return LockstepPX4Backend


class PhysXStateMixin:
    """Vehicle state read entirely from the PhysX body every physics step.

    Pegasus reads the attitude from the USD xform, which PhysX only writes back once per rendered
    frame (every ~17 physics steps at 1 kHz), so controllers and logs saw a stale attitude.
    """

    def update_state(self, dt: float, context=None):
        if self._body_rigid_prim is None:
            return
        positions, orientations = self._body_rigid_prim.get_world_poses()
        lin, ang = self._body_rigid_prim.get_velocities()
        qw, qx, qy, qz = orientations.numpy()[0].astype(np.float64)
        v = lin.numpy()[0].astype(np.float64)
        s = self._state
        rot = Rotation.from_quat([qx, qy, qz, qw])
        s.linear_acceleration = (v - s.linear_velocity) / dt if dt > 0.0 else np.zeros(3)
        s.position = positions.numpy()[0].astype(np.float64)
        s.attitude = np.array([qx, qy, qz, qw])
        s.linear_velocity = v
        s.linear_body_velocity = rot.inv().apply(v)
        s.angular_velocity = rot.inv().apply(ang.numpy()[0].astype(np.float64))   # world -> body
