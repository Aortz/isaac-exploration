"""Fly an F1 reference with a simulated quadrotor (Pegasus Simulator) and log what it actually did.

The F1 spline (reference_1000hz.csv, written by f1_trajectory.py for flights with `dynamics: pegasus`)
is tracked by Pegasus' geometric NonlinearController on the Iris model: PhysX rigid-body dynamics,
quadratic rotor thrust curve, rotor drag torques and linear body drag. Every physics step logs the
vehicle state; flight/pegasus_post.py turns that log into ground truth + IMU in the F1 file layout,
so rig_fisheye.py / OpenVINS run on it unchanged.

No Cesium here: the tiles have no colliders, so the dynamics never see them. The cameras are
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

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--flight", required=True)
ap.add_argument("--config", default=str(PROJECT_ROOT / "config" / "flight.yaml"))
ap.add_argument("--gui", action="store_true", help="open the Isaac Sim window")
ap.add_argument("--max-s", type=float, default=0.0, help="stop after this much flight time (test runs)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui,
                     "extra_args": ["--ext-folder", str(PEGASUS_EXT), "--enable", "pegasus.simulator",
                                    "--/crashreporter/url=", "--/crashreporter/skipOldDumpUpload=true"]})

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # noqa: E402
from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pipeline.common.config import BUILD, load_config  # noqa: E402

sys.path.insert(0, str(PEGASUS_EXT.parent / "examples" / "utils"))
from nonlinear_controller import NonlinearController  # noqa: E402

VEHICLE = "/World/quadrotor"


class Tracker(NonlinearController):
    """Pegasus' controller fed from an in-memory reference instead of its (row-reversed) CSV format."""

    def __init__(self, reference: np.ndarray, mass: float, gains: dict):
        self._reference = reference
        super().__init__(trajectory_file="<memory>", **gains)
        self.m = mass

    def read_trajectory_from_csv(self, file_name: str):
        return self._reference


class PhysicsMultirotor(Multirotor):
    """Multirotor whose state comes entirely from the PhysX body.

    Pegasus reads the attitude from the USD xform, which PhysX only writes back once per rendered
    frame (every ~17 physics steps here), so the controller and the log saw a stale attitude.
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

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(hz)

    pg = PegasusInterface()
    pg.set_world_settings(physics_dt=1.0 / hz)
    pg.initialize_world()

    yaw0 = float(ref[0, 13])
    mcfg = MultirotorConfig()
    mcfg.drag = LinearDrag(pcfg["drag"])
    mcfg.sensors = []               # IMU is synthesised from the logged physics state (pegasus_post.py)
    tracker = Tracker(reference, mass=1.5, gains=pcfg["gains"])
    mcfg.backends = [tracker]
    vehicle = PhysicsMultirotor(VEHICLE, ROBOTS[pcfg["vehicle"]], 0, ref[0, 1:4].tolist(),
                         [0.0, 0.0, np.sin(yaw0 / 2), np.cos(yaw0 / 2)], config=mcfg)
    mass = vehicle_mass(stage, VEHICLE)
    if mass > 0:
        tracker.m = mass

    n_total = n_pre + len(ref)
    log = np.zeros((n_total, 18))   # step, p(3), q_xyzw(4), v_world(3), w_body(3), rotor_ref(4)
    dts = np.zeros(n_total)
    k = [0]

    def on_step(dt: float, context=None) -> None:
        i = k[0]
        if i >= n_total:
            return
        s = vehicle.state
        log[i] = np.r_[i, s.position, s.attitude, s.linear_velocity, s.angular_velocity, tracker.input_ref]
        dts[i] = dt
        k[0] = i + 1

    cb = SimulationManager.register_callback(on_step, SimulationEvent.PHYSICS_POST_STEP)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    t0, next_report = time.time(), 0
    while k[0] < n_total and app.is_running():
        app.update()
        if k[0] >= next_report:
            print(f"  t={k[0] / hz - preroll:6.1f} s  ({k[0]}/{n_total} steps, {time.time() - t0:.0f} s wall)", flush=True)
            next_report += 10 * hz
    wall = time.time() - t0
    timeline.stop()
    SimulationManager.deregister_callback(cb)

    if k[0] < n_total:
        raise SystemExit(f"stopped after {k[0]} of {n_total} physics steps")
    if np.abs(dts * hz - 1.0).max() > 1e-4:
        raise SystemExit(f"physics dt varied: {dts.min():.6f}..{dts.max():.6f} s")
    # Row i holds the state after physics step i, i.e. at time (i + 1) / hz since the start.
    t = (log[:, 0] + 1) / hz - preroll
    keep = t >= -1e-9
    out = np.column_stack([t[keep], log[keep, 1:]])
    np.savetxt(fdir / "pegasus_state_1000hz.csv", out, fmt="%.9f", delimiter=",",
               header="t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,rotor0,rotor1,rotor2,rotor3  "
                      "(body FLU in world ENU; v world; w body; rotor speed references rad/s)")
    meta = {"flight": args.flight, "vehicle": pcfg["vehicle"], "mass_kg": round(tracker.m, 4),
            "physics_hz": hz, "preroll_s": preroll, "steps": int(n_total), "wall_s": round(wall, 1),
            "realtime_factor": round(n_total / hz / wall, 2), "drag": pcfg["drag"], "gains": pcfg["gains"],
            "pegasus_commit": "fcb99c0 (PegasusSimulator PR #144, Isaac Sim 6.0 port)"}
    (fdir / "pegasus.json").write_text(json.dumps(meta, indent=2) + "\n")
    print("PEGASUS", json.dumps(meta))


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
