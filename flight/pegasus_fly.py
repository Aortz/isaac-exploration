"""Fly an F1 reference with a simulated quadrotor (Pegasus Simulator) and log what it actually did.

The F1 spline (reference_1000hz.csv, written by f1_trajectory.py for flights with `dynamics: pegasus`)
is flown by the Iris model: PhysX rigid-body dynamics, quadratic rotor thrust curve with first-order
motor lag (`pegasus.motor`), rotor drag torques, and linear body drag on the air-relative velocity
under wind, turbulence and gusts (`pegasus.wind`, per-flight `wind:`). The controller is either
  python  Pegasus' geometric NonlinearController with perfect state (default), or
  px4     PX4 v1.16 SITL in Docker (px4/), with EKF2 on Pegasus' simulated IMU/GPS/baro/mag, in
          offboard mode (flight/px4_offboard.py): takeoff from the ground, hold, then the reference.
Every physics step logs the vehicle state; flight/pegasus_post.py turns that log into ground truth +
IMU in the F1 file layout, so rig_fisheye.py / OpenVINS run on it unchanged.

No Cesium here: the tiles have no colliders, so the dynamics never see them. Flights in an Isaac
environment (`env:`) fly inside that stage instead, so its colliders are live and a bad trajectory
crashes (the tracking-error gate catches it). The cameras are
rendered afterwards by replaying the flown poses in lockstep (rig_fisheye.py), which gives the same
images as rendering during the flight, because nothing in the controller depends on the images.

Run (Isaac Python): $ISAAC_ROOT/python.sh flight/pegasus_fly.py --flight marina_bay_loop_pegasus
Writes build/flight/<flight>/pegasus_state_1000hz.csv and pegasus.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PEGASUS_EXT = PROJECT_ROOT / "third_party" / "pegasus" / "extensions"
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "third_party" / "isaac_pydeps"))   # pymavlink for Isaac's Python

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--flight", required=True)
ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
ap.add_argument("--gui", action="store_true", help="open the Isaac Sim window")
ap.add_argument("--max-s", type=float, default=0.0, help="stop after this much flight time (test runs)")
ap.add_argument("--controller", choices=["python", "px4"], help="override the flight's controller")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui,
                     "extra_args": ["--ext-folder", str(PEGASUS_EXT), "--enable", "pegasus.simulator",
                                    "--/crashreporter/url=", "--/crashreporter/skipOldDumpUpload=true"]})

import faulthandler  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402

faulthandler.register(signal.SIGUSR1, all_threads=True)   # `kill -USR1 <pid>` prints every thread's stack

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # noqa: E402
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackendConfig  # noqa: E402
from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.sensors import GPS, IMU, Barometer, Magnetometer  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pipeline.common.config import BUILD, load_config  # noqa: E402

from flight.pegasus_models import LaggedThrustCurve, PhysXStateMixin, Wind, WindDrag, lockstep_px4_backend  # noqa: E402
from pipeline.common import environments as envs  # noqa: E402

sys.path.insert(0, str(PEGASUS_EXT.parent / "examples" / "utils"))
from nonlinear_controller import NonlinearController  # noqa: E402

VEHICLE = "/World/quadrotor"
PX4_CONTAINER = "px4_sitl"
AP_CONTAINER = "ardupilot_sitl"
AUTOPILOTS = ("px4", "ardupilot")


def ground_plane(stage, z: float, size: float = 2000.0) -> None:
    """Static collision slab with its top at z (PX4 takes off from the ground; Pegasus' stage has none)."""
    cube = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z - 0.5))
    xf.AddScaleOp().Set(Gf.Vec3f(size, size, 1.0))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


class Tracker(NonlinearController):
    """Pegasus' controller fed from an in-memory reference instead of its (row-reversed) CSV format."""

    def __init__(self, reference: np.ndarray, mass: float, gains: dict):
        self._reference = reference
        super().__init__(trajectory_file="<memory>", **gains)
        self.m = mass

    def read_trajectory_from_csv(self, file_name: str):
        return self._reference


class PhysicsMultirotor(PhysXStateMixin, Multirotor):
    """Pegasus multirotor with its state read from the PhysX body (see PhysXStateMixin)."""


def vehicle_mass(stage, root: str) -> float:
    total = 0.0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.MassAPI):
            total += UsdPhysics.MassAPI(prim).GetMassAttr().Get() or 0.0
    return total


def main() -> None:
    cfg = load_config(args.config)
    pcfg = cfg["pegasus"]
    fdir = BUILD / "flight" / args.flight
    ref = np.loadtxt(fdir / "reference_1000hz.csv", delimiter=",", comments="#")
    if args.max_s:
        ref = ref[ref[:, 0] <= args.max_s]
    hz, preroll = int(pcfg["physics_hz"]), float(pcfg["preroll_s"])
    if abs((ref[1, 0] - ref[0, 0]) * hz - 1.0) > 1e-6:
        raise SystemExit(f"reference rate != physics_hz {hz}")
    # Controller time starts at 0 at the first physics step: hold the first reference row for the pre-roll.
    n_pre = int(round(preroll * hz))
    hold = np.repeat(ref[:1], n_pre, axis=0)
    hold[:, 0] = np.arange(n_pre) / hz
    reference = np.vstack([hold, ref + np.r_[preroll, np.zeros(ref.shape[1] - 1)]])

    fl = cfg["flights"][args.flight]
    if "env" in fl and cfg["environments"][fl["env"]].get("colliders"):
        stage = envs.open_env(app, cfg["environments"][fl["env"]])
    else:
        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    envs.ensure_physics_scene(stage, hz)

    pg = PegasusInterface()
    pg.set_world_settings(physics_dt=1.0 / hz)
    pg.initialize_world()

    controller = args.controller or fl.get("controller", "python")
    vname = fl.get("vehicle")                     # None = Pegasus' own vehicle (pegasus.vehicle)
    vcfg = pcfg["vehicles"][vname] if vname else None
    if vcfg and controller not in AUTOPILOTS:
        raise SystemExit(f"vehicle {vname} needs controller: px4 or ardupilot")
    yaw0 = float(ref[0, 13])
    mcfg = MultirotorConfig()
    motor = pcfg["motor"] if fl.get("motor_lag") else None
    if motor:
        mcfg.thrust_curve = LaggedThrustCurve(motor["tau_up_s"], motor["tau_down_s"])
    wind_cfg = {**pcfg.get("wind_defaults", {}), **fl["wind"]} if fl.get("wind") else {}
    drag_coeffs = vcfg["drag"] if vcfg else pcfg["drag"]
    drag = WindDrag(drag_coeffs, Wind(wind_cfg)) if wind_cfg else LinearDrag(drag_coeffs)
    mcfg.drag = drag
    if controller in AUTOPILOTS:
        # The autopilot needs the sensors Pegasus simulates for it; our own IMU still comes from the physics
        # log. ArduPilot's JSON interface takes the IMU every physics step and simulates GPS/baro/mag itself.
        imu_hz = float(pcfg["px4"]["imu_hz"]) if controller == "px4" else float(hz)
        mcfg.sensors = [Barometer(), IMU({"update_rate": imu_hz}), Magnetometer(), GPS()]
        if controller == "px4":
            mcfg.backends = [lockstep_px4_backend()(PX4MavlinkBackendConfig({
                "vehicle_id": 0, "px4_autolaunch": False, "enable_lockstep": True, "update_rate": imu_hz}))]
        elif not vcfg:
            raise SystemExit("controller: ardupilot is wired for the coaxial vehicle only (vehicle: coax_*)")
        ground_z = float(fl.get("ground_z", cfg["environments"].get(fl.get("env"), {}).get("ground_z", 0.0)))
        ground_plane(stage, ground_z)
        spawn = [float(ref[0, 1]), float(ref[0, 2]), ground_z + (0.18 if vcfg else 0.07)]
        tracker = None
    else:
        mcfg.sensors = []               # IMU is synthesised from the logged physics state (pegasus_post.py)
        tracker = Tracker(reference, mass=1.5, gains=pcfg["gains"])
        mcfg.backends = [tracker]
        spawn = ref[0, 1:4].tolist()
    quat0 = [0.0, 0.0, np.sin(yaw0 / 2), np.cos(yaw0 / 2)]
    if vcfg:
        from flight.coax_vehicle import CoaxialHelicopter, coax_ardupilot_backend, coax_px4_backend, write_coax_usd
        usd = write_coax_usd(BUILD / "vehicles" / f"{vname}.usda", vcfg)
        backend = coax_px4_backend(pcfg["px4"]["imu_hz"]) if controller == "px4" else coax_ardupilot_backend()
        vehicle = CoaxialHelicopter(VEHICLE, str(usd), spawn, quat0, vcfg, mcfg.sensors, backend, drag)
    else:
        vehicle = PhysicsMultirotor(VEHICLE, ROBOTS[pcfg["vehicle"]], 0, spawn, quat0, config=mcfg)
    mass = vehicle_mass(stage, VEHICLE)
    if tracker and mass > 0:
        tracker.m = mass

    rows, dts, k = [], [], [0]
    state = {"t_track0": None, "phase": "python"}
    pilot = None
    if controller == "px4":
        from flight.px4_offboard import OffboardPilot
        pilot = OffboardPilot(ref, ref[0, 1:4], pcfg["px4"]["params"], settle_s=float(pcfg["px4"]["settle_s"]))
    n_total = n_pre + len(ref)

    def on_step(dt: float, context=None) -> None:
        i = k[0]
        t_sim = (i + 1) / hz                      # state after physics step i
        s = vehicle.state
        if pilot:
            yaw = Rotation.from_quat(s.attitude).as_euler("ZYX")[0]
            state["phase"] = pilot.step(t_sim, s.position, s.linear_velocity, yaw)
            state["t_track0"] = pilot.t_track0
        else:
            state["t_track0"] = preroll
        if isinstance(drag, WindDrag):
            drag.t_flight = t_sim - state["t_track0"] if state["t_track0"] is not None else None
        wind = drag.current if isinstance(drag, WindDrag) else np.zeros(3)
        rows.append(np.r_[t_sim, s.position, s.attitude, s.linear_velocity, s.angular_velocity,
                          vehicle._thrusters._velocity, wind])
        dts.append(dt)
        k[0] = i + 1

    cb = SimulationManager.register_callback(on_step, SimulationEvent.PHYSICS_POST_STEP)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    px4_proc = None
    if controller == "px4":
        subprocess.run(["docker", "rm", "-f", PX4_CONTAINER], capture_output=True)
        px4_args = []
        if vcfg:      # custom airframe file, mounted next to PX4's own SITL airframes
            af = next((PROJECT_ROOT / "px4" / "airframes").glob(f"*_{vcfg['px4_model']}"))
            px4_args = ["-e", f"PX4_SIM_MODEL={vcfg['px4_model']}", "-v",
                        f"{af}:/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/{af.name}:ro"]
        px4_proc = subprocess.Popen(["docker", "run", "--rm", "--init", "--name", PX4_CONTAINER, "--net=host",
                                     *px4_args, pcfg["px4"]["image"], "0"],
                                    stdout=open(fdir / "px4.log", "w"), stderr=subprocess.STDOUT)
    ap_proc = None
    if controller == "ardupilot":
        acfg = pcfg["ardupilot"]
        subprocess.run(["docker", "rm", "-f", AP_CONTAINER], capture_output=True)
        params = [f"/opt/ardupilot/Tools/autotest/default_params/{f}" for f in acfg["base_params"]]
        params += [f"/params/{Path(f).name}" for f in [vcfg["ardupilot_params"]]]
        ap_proc = subprocess.Popen(["docker", "run", "--rm", "--init", "--name", AP_CONTAINER, "--net=host",
                                    "-v", f"{PROJECT_ROOT / Path(vcfg['ardupilot_params']).parent}:/params:ro",
                                    acfg["image"], "0", acfg["home"], *params],
                                   stdout=open(fdir / "ardupilot.log", "w"), stderr=subprocess.STDOUT)
        from flight.px4_offboard import ArduPilotGuidedPilot
        for _ in range(60):                       # SITL opens its MAVLink TCP port a few seconds after start
            try:
                pilot = ArduPilotGuidedPilot(ref, ref[0, 1:4], acfg.get("params", {}), settle_s=float(acfg["settle_s"]))
                break
            except OSError:
                time.sleep(1.0)
        else:
            raise SystemExit("ArduPilot SITL MAVLink port (tcp 5760) never opened")
    t0, next_report = time.time(), 0
    max_sim_s = preroll + ref[-1, 0] + 1.0 + (float(pcfg["px4"]["max_setup_s"]) if pilot else 0.0)

    def finished() -> bool:
        if pilot:
            return state["phase"] == "done"
        return k[0] >= n_total

    try:
        while not finished() and app.is_running() and k[0] / hz < max_sim_s:
            app.update()
            if k[0] >= next_report:
                print(f"  t_sim={k[0] / hz:6.1f} s  phase={state['phase']}  ({time.time() - t0:.0f} s wall)", flush=True)
                next_report += 10 * hz
    finally:
        wall = time.time() - t0
        timeline.stop()
        SimulationManager.deregister_callback(cb)
        if px4_proc:
            subprocess.run(["docker", "kill", PX4_CONTAINER], capture_output=True)
        if ap_proc:
            subprocess.run(["docker", "kill", AP_CONTAINER], capture_output=True)

    if not finished():
        if rows:              # keep the partial log for diagnosis (sim time, not flight time)
            np.savetxt(fdir / "pegasus_state_failed.csv", np.asarray(rows), fmt="%.6f", delimiter=",",
                       header="t_sim,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,act0,act1,act2,act3,wind_e,wind_n,wind_u")
        raise SystemExit(f"flight did not finish: phase {state['phase']} after {k[0] / hz:.1f} s sim"
                         + (f"; PX4 events: {pilot.events[-10:]}" if pilot else ""))
    dts = np.asarray(dts)
    if np.abs(dts * hz - 1.0).max() > 1e-4:
        raise SystemExit(f"physics dt varied: {dts.min():.6f}..{dts.max():.6f} s")
    log = np.asarray(rows)
    # Flight time 0 = start of the reference (end of the pre-roll, or PX4's settle hold).
    t = log[:, 0] - state["t_track0"]
    keep = (t >= -1e-9) & (t <= ref[-1, 0] + 1e-9)
    out = np.column_stack([t[keep], log[keep, 1:]])
    np.savetxt(fdir / "pegasus_state_1000hz.csv", out, fmt="%.9f", delimiter=",",
               header="t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,rotor0,rotor1,rotor2,rotor3,wind_e,wind_n,wind_u  "
                      "(body FLU in world ENU; v world; w body; actual rotor speeds rad/s; wind m/s ENU)")
    meta = {"flight": args.flight, "env": fl.get("env"), "controller": controller, "vehicle": vname or pcfg["vehicle"],
            "mass_kg": round(mass or 1.5, 4), "physics_hz": hz, "setup_s": round(float(state["t_track0"]), 2),
            "steps": int(k[0]), "wall_s": round(wall, 1), "realtime_factor": round(k[0] / hz / wall, 2),
            "drag": drag_coeffs, "motor": motor, "wind": wind_cfg or None,
            "gains": pcfg["gains"] if controller == "python" else None,
            "autopilot": ({"name": controller, "events": pilot.events} if pilot else None),
            "pegasus_commit": "fcb99c0 (PegasusSimulator PR #144, Isaac Sim 6.0 port)"}
    (fdir / "pegasus.json").write_text(json.dumps(meta, indent=2) + "\n")
    print("PEGASUS", json.dumps(meta))


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:      # app.close() ends the process before Python would print it
        import traceback

        traceback.print_exc()
        print(f"PEGASUS_FAILED {e}", flush=True)
        raise
    finally:
        app.close()
