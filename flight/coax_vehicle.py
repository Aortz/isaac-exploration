"""Coaxial helicopter for Pegasus (e.g. an Ascent AeroSystems Spirit-class drone), flown by PX4.

Two counter-rotating fixed-pitch rotors on one mast above the centre of mass. PX4's coaxial-helicopter
control allocation (CA_AIRFRAME 12) drives
  motor 0 / motor 1   rotor speeds: collective thrust from the sum, yaw from the reaction-torque difference
  servo 0 / servo 1   swashplate (cyclic): tilts both tip-path planes for pitch (servo 0) and roll (servo 1)
Model per physics step (body FLU frame):
  rotor speed      first-order lag to max_rotor_speed * motor command
  thrust           T = k (w_u^2 + eta w_l^2), along the tip-path-plane normal tilted by the cyclic angles
  cyclic           first-order lag (servo + flapping) to cyclic_max * servo command
  moments          hub-height lever (r_hub x T) + rigid-rotor hub spring K_beta * tilt * T / (m g)
                   + yaw c (w_cw^2 - w_ccw^2)
  drag             linear body drag on the air-relative velocity (flight/pegasus_models.WindDrag)
The body is one rigid cylinder (with a collider, so it can stand on the take-off slab); the rotor discs
are visual only.

Isaac Python only.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.vehicle import Vehicle
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from flight.pegasus_models import PhysXStateMixin, lockstep_px4_backend


def write_coax_usd(path: Path, vcfg: dict) -> Path:
    """A self-contained vehicle asset: rigid cylinder body (mass, inertia, collider) + two rotor discs."""
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/coax")
    stage.SetDefaultPrim(root.GetPrim())
    body = UsdGeom.Xform.Define(stage, "/coax/body")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
    mass.CreateMassAttr(float(vcfg["mass_kg"]))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in vcfg["inertia_kgm2"]]))
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    # Colliders use PhysX-native shapes (capsule, box): its cylinder collider let the parked body jitter
    # through the ground slab and spin up to hundreds of deg/s with the motors off.
    hull = UsdGeom.Capsule.Define(stage, "/coax/body/hull")
    hull.CreateRadiusAttr(0.053)
    hull.CreateHeightAttr(0.194)                      # + 2 x radius = 0.30 m overall
    hull.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(hull.GetPrim())
    hull.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.17)])
    # Landing gear: a wide, thin collision ring under the hull. Without it the 0.3 m x 0.106 m cylinder
    # standing on end tipped over in the wind before take-off (the autopilots then saw it upside down).
    gear = UsdGeom.Cube.Define(stage, "/coax/body/gear")
    gear.CreateSizeAttr(1.0)
    gx = UsdGeom.Xformable(gear)
    gx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.16))
    gx.AddScaleOp().Set(Gf.Vec3f(0.40, 0.40, 0.02))
    UsdPhysics.CollisionAPI.Apply(gear.GetPrim())
    gear.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.3, 0.3)])
    for i, dz in enumerate((0.06, 0.0)):
        disc = UsdGeom.Cylinder.Define(stage, f"/coax/body/rotor{i}")
        disc.CreateRadiusAttr(float(vcfg["rotor_radius_m"]))
        disc.CreateHeightAttr(0.005)
        disc.CreateAxisAttr("Z")
        UsdGeom.Xformable(disc).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, float(vcfg["hub_height_m"]) + dz))
        disc.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.6)])
        disc.CreateDisplayOpacityAttr([0.25])
    stage.GetRootLayer().Save()
    return path


# Every coax backend exposes `cmd` = [clockwise rotor 0..1, counter-clockwise rotor 0..1,
#                                  pitch cyclic -1..1 (nose up +), roll cyclic -1..1 (right down +)].

def coax_px4_backend(imu_hz: float):
    """PX4 (CA_AIRFRAME 12): motor 1 clockwise, motor 2 counter-clockwise, servo 1 = +pitch, servo 2 = -roll
    (CA_SP0_ANG 0 / 90: servo_i = pitch cos(a_i) - roll sin(a_i))."""
    from pymavlink import mavutil

    Base = lockstep_px4_backend()

    class CoaxPX4Backend(Base):
        def __init__(self, config):
            super().__init__(config)
            self.cmd = np.zeros(4)
            self.raw = np.zeros(4)

        def handle_control(self, time_usec, controls, mode, flags):
            if mode == mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED + 1:
                self.raw = np.asarray(controls[:4], float)
                self.cmd = np.array([self.raw[0], self.raw[1], self.raw[2], -self.raw[3]])
            else:
                self.raw = np.zeros(4)
                self.cmd = np.zeros(4)

    return CoaxPX4Backend(PX4MavlinkBackendConfig({"vehicle_id": 0, "px4_autolaunch": False, "enable_lockstep": True,
                                                    "update_rate": float(imu_hz)}))


class _Actuators:
    """Mirrors Multirotor._thrusters._velocity for the flight log: [w_cw, w_ccw, cyclic_lon, cyclic_lat]."""

    def __init__(self):
        self._velocity = [0.0, 0.0, 0.0, 0.0]


class CoaxialHelicopter(PhysXStateMixin, Vehicle):
    def __init__(self, stage_prefix: str, usd_file: str, init_pos, init_orientation, vcfg: dict, sensors, backend, drag):
        super().__init__(stage_prefix, usd_file, init_pos, init_orientation, sensors, [], [], [backend])
        self.v = vcfg
        self.backend = backend
        self.drag = drag
        self.w = np.zeros(2)                     # rotor speeds: [0] clockwise (upper), [1] counter-clockwise (lower)
        self.cyc = np.zeros(2)                   # tip-path-plane tilt [lon (nose-up positive), lat (roll-right positive)]
        self._thrusters = _Actuators()

    def update(self, dt: float, context=None):
        v = self.v
        self.backend.update(dt)
        cmd = self.backend.cmd
        w_cmd = np.clip(cmd[:2], 0.0, 1.0) * v["max_rotor_speed_rad_s"]
        cyc_cmd = np.clip(cmd[2:4], -1.0, 1.0) * np.radians(v["cyclic_max_deg"])
        self.w = w_cmd + (self.w - w_cmd) * np.exp(-dt / v["motor_tau_s"])
        self.cyc = cyc_cmd + (self.cyc - cyc_cmd) * np.exp(-dt / v["cyclic_tau_s"])
        k, c, eta = v["thrust_coeff"], v["torque_coeff"], v["lower_rotor_efficiency"]
        thrust = k * (self.w[0] ** 2 + eta * self.w[1] ** 2)
        lon, lat = self.cyc
        # Tilting the tip-path plane back (nose-up command) and right (roll-right command) puts the thrust
        # line behind / right of the hub, which above the CoM gives nose-up / roll-right moments.
        n = np.array([-np.sin(lon), -np.sin(lat), 1.0])
        n /= np.linalg.norm(n)
        f_hub = thrust * n
        r_hub = np.array([0.0, 0.0, v["hub_height_m"]])
        kb = v["hub_stiffness_nm_per_rad"]
        # Reaction torque on the body: the clockwise (from above) rotor pushes the body counter-clockwise (+z).
        yaw = c * (self.w[0] ** 2 - self.w[1] ** 2)
        # The hub spring only exists while the rotors spin (flapping needs lift): scale it with thrust relative
        # to hover. Unscaled, the servos' disarmed position (PWM at its minimum = full tilt) put 1.4 N m on the
        # parked aircraft and tumbled it before take-off.
        kb_eff = kb * thrust / (v["mass_kg"] * 9.81)
        torque = np.cross(r_hub, f_hub) + np.array([kb_eff * lat, -kb_eff * lon, yaw])
        drag = self.drag.update(self._state, dt)
        self._apply(f_hub + drag, torque)
        self._thrusters._velocity = [float(self.w[0]), float(self.w[1]), float(lon), float(lat)]

    def _apply(self, force, torque) -> None:
        rp = self._body_rigid_prim
        if rp is None or not rp.is_physics_tensor_entity_valid():
            return
        rp.apply_forces_and_torques_at_pos(forces=np.asarray([force], np.float32), torques=np.asarray([torque], np.float32),
                                           positions=np.zeros((1, 3), np.float32), local_frame=True)


def coax_ardupilot_backend(vehicle_id: int = 0):
    """ArduCopter coax (FRAME_CLASS 9) over Pegasus' JSON lockstep plugin (UDP 9002): SERVO1 roll cyclic,
    SERVO2 pitch cyclic (1000..2000 us, trim 1500), SERVO5 counter-clockwise rotor, SERVO6 clockwise rotor.
    Arming is read from the PWM (motors at 1000 us = stopped), so this backend never reads MAVLink and
    the flight script's GUIDED pilot can own the MAVLink link."""
    from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (ArduPilotMavlinkBackend,
                                                                              ArduPilotMavlinkBackendConfig)

    class CoaxArduPilotBackend(ArduPilotMavlinkBackend):
        def __init__(self, config):
            super().__init__(config)
            self.cmd = np.zeros(4)
            self.pwm = np.zeros(16)

        def update_is_armed(self):
            self._armed = True

        def update_motor_commands(self, servos):
            if servos is None or len(servos) < 6:
                return
            pwm = np.asarray(servos, float)
            self.pwm = pwm
            norm = lambda us: float(np.clip((us - 1000.0) / 1000.0, 0.0, 1.0))   # noqa: E731
            cyc = lambda us: float(np.clip((us - 1500.0) / 500.0, -1.0, 1.0)) if us > 0 else 0.0   # noqa: E731
            self.cmd = np.array([norm(pwm[5]), norm(pwm[4]), cyc(pwm[1]), cyc(pwm[0])])

    return CoaxArduPilotBackend(ArduPilotMavlinkBackendConfig({
        "vehicle_id": vehicle_id, "ardupilot_autolaunch": False, "connection_type": "udpin",
        "connection_ip": "127.0.0.1", "connection_baseport": 14650, "num_rotors": 4}))
