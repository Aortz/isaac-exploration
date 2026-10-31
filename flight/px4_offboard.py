"""Fly a reference trajectory through PX4 offboard (or ArduPilot GUIDED) control, stepped from the physics callback.

PX4 (SITL, flight/pegasus_fly.py starts it in Docker) closes the loop with its own EKF2 and position /
attitude / rate controllers on Pegasus' simulated IMU, GPS, baro and magnetometer, exactly as on a real
vehicle. This pilot only does what a companion computer would: set parameters, wait for a GPS fix, arm,
take off to the start point, hold, then stream position + velocity + acceleration + yaw setpoints from
the reference at 50 Hz of sim time.

Frames: the reference is world ENU; PX4 setpoints are local NED around PX4's EKF origin. The mapping
is fixed when the vehicle is armed (EKF position <-> true position), so EKF drift after that shows up
as real tracking error, as it would in flight.
"""

from __future__ import annotations

import struct
import time

import numpy as np
from pymavlink import mavutil

PX4_MODE_OFFBOARD = 6                       # PX4 custom main mode
MASK_POS_VEL_ACC_YAW_RATE = 0               # type_mask: use everything (pos, vel, acc, yaw, yaw rate)


def _int_param(v: int) -> float:
    """PX4 decodes INT32 params bytewise from the float field."""
    return struct.unpack("<f", struct.pack("<i", int(v)))[0]


class OffboardPilot:
    PHASES = ("connect", "wait_gps", "arm", "climb", "settle", "track", "done")
    AUTOPILOT = mavutil.mavlink.MAV_AUTOPILOT_PX4
    FLY_MODE = PX4_MODE_OFFBOARD
    NAME = "px4"

    def __init__(self, reference: np.ndarray, start_enu: np.ndarray, params: dict, url: str = "udpin:0.0.0.0:14540",
                 settle_s: float = 5.0, setpoint_hz: float = 50.0, gps_wait_s: float = 12.0):
        self.ref = reference                      # t, p(3), v(3), a(3), j(3), yaw, yaw_rate  (ENU)
        self.start = np.asarray(start_enu, float)
        self.params = params
        self.conn = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.settle_s, self.dt_sp, self.gps_wait_s = settle_s, 1.0 / setpoint_hz, gps_wait_s
        self.phase = "connect"
        self.t_phase = 0.0
        self.t_last_sp = -1e9
        self.t_last_cmd = -1e9
        self.t_track0 = None                      # sim time at which the reference clock starts
        self.ned0 = None                          # EKF local position at arming
        self.enu0 = None                          # true position at arming
        self.yaw_off = 0.0                        # EKF yaw - true yaw (NED) at arming
        self.local = None                         # latest LOCAL_POSITION_NED (n, e, d)
        self.yaw_est = None
        self.armed = False
        self.mode = None
        self.home = False
        self.still_since = None
        self.events = []

    # --- MAVLink plumbing -------------------------------------------------------------------
    def _poll(self) -> None:
        while True:
            m = self.conn.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "HEARTBEAT" and m.get_srcComponent() == 1 and m.autopilot == self.AUTOPILOT:
                self.conn.target_system, self.conn.target_component = m.get_srcSystem(), m.get_srcComponent()
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self._decode_mode(m.custom_mode)
            elif t == "LOCAL_POSITION_NED":
                self.local = np.array([m.x, m.y, m.z])
            elif t == "ATTITUDE":
                self.yaw_est = m.yaw
            elif t == "HOME_POSITION":
                self.home = True
            elif t == "STATUSTEXT":
                self.events.append(m.text)

    @staticmethod
    def _decode_mode(custom_mode: int) -> int:
        return (custom_mode >> 16) & 0xFF            # PX4: main mode in bits 16-23

    def _engage(self, t: float, p_true: np.ndarray, yaw_true_enu: float) -> bool:
        """Arm and switch to the setpoint-following mode; True once flying in it."""
        self._sp_start(t, yaw_true_enu)
        if t - self.t_phase > 1.0 and t - self.t_last_cmd > 1.0:     # setpoints must flow before OFFBOARD
            self._cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                      self.FLY_MODE)
            self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self.t_last_cmd = t
        return self.armed and self.mode == self.FLY_MODE

    def _cmd(self, command: int, *p) -> None:
        p = list(p) + [0.0] * (7 - len(p))
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component, command, 0, *p)

    def _interval(self, msg_id: int, hz: float) -> None:
        self._cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def _enu_to_ned(self, p_enu: np.ndarray) -> np.ndarray:
        d = p_enu - self.enu0
        return self.ned0 + np.array([d[1], d[0], -d[2]])

    def _send_sp(self, t_ms: int, p_enu, v_enu, a_enu, yaw_enu: float, yaw_rate_enu: float) -> None:
        n = self._enu_to_ned(np.asarray(p_enu, float))
        v = np.array([v_enu[1], v_enu[0], -v_enu[2]])
        a = np.array([a_enu[1], a_enu[0], -a_enu[2]])
        yaw = (np.pi / 2 - yaw_enu) + self.yaw_off
        self.conn.mav.set_position_target_local_ned_send(
            t_ms, self.conn.target_system, self.conn.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MASK_POS_VEL_ACC_YAW_RATE, *n, *v, *a, float(np.arctan2(np.sin(yaw), np.cos(yaw))), -float(yaw_rate_enu))

    # --- state machine, called every physics step ----------------------------------------------
    def step(self, t: float, p_true: np.ndarray, v_true: np.ndarray, yaw_true_enu: float) -> str:
        self._poll()
        if self.phase == "connect":
            if self.conn.target_system:          # set by the first heartbeat
                for k, v in self.params.items():
                    val, typ = (_int_param(v), mavutil.mavlink.MAV_PARAM_TYPE_INT32) if isinstance(v, int) else \
                        (float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    self.conn.mav.param_set_send(self.conn.target_system, self.conn.target_component,
                                                 k.encode(), val, typ)
                self._interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)
                self._interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
                self._interval(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1)
                self._enter("wait_gps", t)
        elif self.phase == "wait_gps":
            if self.home and self.local is not None and t - self.t_phase > self.gps_wait_s:
                self.ned0, self.enu0 = self.local.copy(), np.asarray(p_true, float).copy()
                if self.yaw_est is not None:
                    self.yaw_off = float(np.angle(np.exp(1j * (self.yaw_est - (np.pi / 2 - yaw_true_enu)))))
                self._enter("arm", t)
        elif self.phase == "arm":
            if self._engage(t, p_true, yaw_true_enu):
                self._enter("climb", t)
        elif self.phase == "climb":
            self._sp_start(t, yaw_true_enu)
            # 0.5 m / 0.3 m/s: a helicopter in turbulence never sits inside the 0.2 m / 0.1 m/s an Iris
            # manages in still air.
            near = np.linalg.norm(p_true - self.start) < 0.5 and np.linalg.norm(v_true) < 0.3
            self.still_since = (self.still_since or t) if near else None
            if self.still_since is not None and t - self.still_since > 2.0:
                self._enter("settle", t)
        elif self.phase == "settle":
            self._sp_start(t, yaw_true_enu)
            if t - self.t_phase > self.settle_s:
                self.t_track0 = t
                self._enter("track", t)
        elif self.phase == "track":
            tr = t - self.t_track0
            if tr > self.ref[-1, 0]:
                self._enter("done", t)
            elif t - self.t_last_sp >= self.dt_sp - 1e-9:
                i = min(int(round(tr * 1000)), len(self.ref) - 1)
                r = self.ref[i]
                self._send_sp(int(t * 1000), r[1:4], r[4:7], r[7:10], r[13], r[14])
                self.t_last_sp = t
        if self.phase in ("climb", "settle", "track") and not self.armed:
            raise RuntimeError(f"{self.NAME} disarmed during {self.phase} at t={t:.2f}; events: {self.events[-8:]}")
        return self.phase

    def _sp_start(self, t: float, yaw_true_enu: float) -> None:
        if t - self.t_last_sp >= self.dt_sp - 1e-9:
            self._send_sp(int(t * 1000), self.start, np.zeros(3), np.zeros(3), float(self.ref[0, 13]), 0.0)
            self.t_last_sp = t

    def _enter(self, phase: str, t: float) -> None:
        self.events.append(f"{phase} at t={t:.2f} (wall {time.strftime('%H:%M:%S')})")
        print(f"  [{self.NAME}] {self.events[-1]}", flush=True)
        self.phase, self.t_phase = phase, t


class ArduPilotGuidedPilot(OffboardPilot):
    """Same flight through ArduCopter: GUIDED mode, arm, NAV_TAKEOFF to the start height, then the same
    position + velocity + acceleration + yaw targets (ArduCopter accepts all of them in GUIDED)."""

    AUTOPILOT = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
    FLY_MODE = 4                                      # ArduCopter GUIDED
    NAME = "ardupilot"

    def __init__(self, *a, url: str = "tcp:127.0.0.1:5760", **kw):
        super().__init__(*a, url=url, **kw)
        self.took_off = False

    @staticmethod
    def _decode_mode(custom_mode: int) -> int:
        return int(custom_mode)

    def _engage(self, t: float, p_true: np.ndarray, yaw_true_enu: float) -> bool:
        if t - self.t_last_cmd > 1.0:
            if self.mode != self.FLY_MODE:
                self._cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          self.FLY_MODE)
            elif not self.armed:
                self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            elif not self.took_off:
                # ArduCopter ignores position targets on the ground: take off first (altitude above home).
                self._cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, float(self.start[2] - self.enu0[2]))
                self.took_off = True
            self.t_last_cmd = t
        if self.took_off and self.local is not None and -self.local[2] + self.ned0[2] > 0.8 * (self.start[2] - self.enu0[2]):
            return True                               # most of the way up: hand over to position targets
        return False
