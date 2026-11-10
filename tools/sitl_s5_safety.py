"""S5 safety v2: PX4 v1.14.3 SITL failsafe scenarios on navrl_quad, the stand-in until the real frame is modelled.

Plan stage S5 (docs/plans/sitl_validation_plan_v1.md, row S5: "geofence, kill switch and RC override").
Preregistration: PREREG below (approval pending). --smoke and --launch refuse until it is committed, because it is one of
the bound sources. New dated v2 retains the v1 safety parameter set and scenarios, fixes observation/cleanup separation,
first-INVALID abort, and adds mandatory raw telemetry/ULog coverage. Historical 2026-11-08 WIP and prereg remain intact.
CPU tests exercise fake PX4 and the installed pyulog parser; live execution remains separately gated.

One fresh-rootfs PX4 + Gazebo boot per scenario (tools/sitl_owned_session.py, as S3b/S3c):
  1. one fixed safety parameter set (SAFETY_PARAMS + AUX_PARAMS) and S3b's navrl_quad vehicle parameters are set and
     every one is read back (INT32 byte-cast into the float field, as PX4 requires); a mismatch stops before arming;
  2. S3b's take-off to 1.0 m under offboard position setpoints, a 5 s pre-injection window with nothing injected (the
     in-scenario null), then one injection;
  3. every autopilot HEARTBEAT (armed, PX4 custom mode) is logged against PX4 simulation time, with STATUSTEXT,
     COMMAND_ACK, RC_CHANNELS (PX4's echo of input_rc), MANUAL_CONTROL (PX4's echo of the *selected* manual setpoint and
     rc_update's switches), EXTENDED_SYS_STATE, the EKF position and the simulator's true position;
  4. assess() applies the scenario's fixed criterion (expected mode or disarm within a fixed time, not earlier than the
     configured timeout, through allowed modes only, persisting PERSIST_S) after the harness validity gates, and
     requires the scenario's own cause (v2 review r1 M2): its ULog failsafe_flags flag first (OBL, GFH, GFV, RCL,
     DLL), or its cause STATUSTEXT with no live cause flag (OVJ, OVR, KIL), or the answered commands with no live
     cause flag (FDS); every pre-window and null window must keep the live cause flags false.
Live discrimination is NOT_TESTED: every must-FAIL control is in the fake-PX4 CPU tests only (v2 review r1 M1).

Attempt scenarios (SCHEDULE): N0 null, OBL offboard setpoints stopped, GFH out past the horizontal fence, GFV up past the
vertical fence, OVJ MANUAL_CONTROL stick pulse, OVR RC_CHANNELS_OVERRIDE stick pulse, RCL RC stopped, DLL GCS heartbeat
stopped, KIL RC kill channel high, FDS GCS disarm (non-forced refused in the air, forced 21196 disarms). smoke01
(SMOKE_SCHEDULE) runs the nulls N0, N0R (RC streamed) and N0J (joystick streamed), read only for harness validity.

PX4 v1.14.3 facts used (read in /home/fair/workspaces/px4_ws/PX4-Autopilot, tag v1.14.3; review r1 re-traced them):
  - parameter definitions: navigator/geofence_params.c:62,90,115,128,142; commander/commander_params.c:104,143,186,284,
    313,342,610,625,787,806,820,926; rc_update/params.c:1168,1200,1267,1299,1442,1966;
  - PARAM_SET needs a matching type and copies the raw bytes (mavlink/mavlink_parameters.cpp:128-134);
  - HEARTBEAT is a constant 1 Hz stream (mavlink_main.cpp:2143-2144; an interval request is refused,
    mavlink_receiver.cpp:2172-2174), so mode changes are resolved to about 1 s and PERSIST_S is 2.5 s;
  - RC_CHANNELS_OVERRIDE -> input_rc (mavlink_receiver.cpp:1953-2037), RC_CHANNELS echoes input_rc (streams/
    RC_CHANNELS.hpp:58); rc_update publishes SOURCE_RC (rc_update.cpp:677) on a change or after 300 ms (:529-530), so the
    harness dithers the unmapped channel 8 (1000/1001) to make every frame a change; MANUAL_CONTROL (msg 69,
    streams/MANUAL_CONTROL.hpp:60-91) echoes the selected manual_control_setpoint (x pitch, y roll, z throttle, r yaw,
    x1000) and rc_update's kill switch in buttons bits 12-13 (ON 1, OFF 3, ManualControlSwitches.msg);
  - kill: OFF->ON edge from an RC source (ManualControl.cpp:170-172,229-235) -> manual_lockdown (Commander.cpp:1532-1548)
    -> disarm after COM_KILL_DISARM (:2155-2168); in-air disarm without 21196 is refused with TEMPORARILY_REJECTED
    (Commander.cpp:595-615,863,887-889);
  - failsafe mapping failsafe.cpp:42-121,279-322; RC loss gates :337-360,466-477; offboard loss is a mode fallback with no
    COM_FAIL_ACT_T hold (:460-490; framework.cpp:330-341), RC/GCS-loss Land waits COM_FAIL_ACT_T in Hold (framework.cpp:
    331-336,463-468); GCS link Commander.cpp:657,2550-2562,2627-2634 with the 2.5 s heartbeat timeout checked every
    1.25 s (mavlink_receiver.cpp:2908-2921); NAV_DLL_ACT > 0 needs a GCS link to arm (rcAndDataLinkCheck.cpp), so GCS
    heartbeats run from the start in every scenario;
  - geofence distances are from home on the global position (navigator/geofence.cpp:245-300).
Checked by reading in review r1: runtime COM_RC_IN_MODE/RC_MAP_* changes apply without a reboot; the default RC
calibration passes the arming check; a centred 1500 us throttle reads as zero. INFERRED until a run: that all of these
behave as read when driven over MAVLink in this SITL build.

  tools/sitl_s5_safety.py --check | --smoke | --launch | --analyze [ATTEMPT_DIR]
Run only under the dated preregistration's exact CPU environment, idle-host and deadline limits.
"""
import argparse
import collections
import json
import math
import os
import pathlib
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))
import sitl_s3b_formal as SF                                    # noqa: E402

PROTOCOL = "s5-safety-v2"
PREREG = "docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md"
SCOPE = ("S5 safety v2: PX4 v1.14.3 SITL on navrl_quad (stand-in vehicle), fixed safety parameter set v1, scenarios "
         "N0 OBL GFH GFV OVJ OVR RCL DLL KIL FDS once each in this order, one fresh-rootfs boot per scenario; "
         "development smoke01 (nulls N0 N0R N0J, read for harness validity only) then attempt01; no retries; "
         "SITL only, no claim about real hardware")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "sitl_next_20261110" / "s5_safety_v2"
ATTEMPT_DIR = UNIT_DIR / "attempt01"
SMOKE_DIR = UNIT_DIR / "smoke01"
MODEL = "navrl_quad"
AIRFRAME = SF.AIRFRAMES[MODEL]
OWN_SOURCES = ("tools/sitl_s5_safety.py", "tests/test_sitl_s5_safety.py", "tools/sitl_s3b_formal.py",
               "tools/sitl_s3b_closed_loop.py", "tools/sitl_owned_session.py", "tools/sitl_dev08_attempt.py", PREREG)

# ------------------------------------------------------------------------------------------- the parameter set
# (name, PX4 type, value, v1.14.3 definition, meaning). Conservative SITL stand-ins; every value is a user decision for
# the real vehicle (USER_DECISIONS).
SAFETY_PARAMS = (
    ("GF_ACTION", "INT32", 2, "navigator/geofence_params.c:62", "Hold mode on a fence breach"),
    ("GF_MAX_HOR_DIST", "REAL32", 17.0, "navigator/geofence_params.c:115",
     "17 m from home: 3 m inside the 20 m half width of MOTAR's 40 x 40 m arena when home is its centre"),
    ("GF_MAX_VER_DIST", "REAL32", 2.5, "navigator/geofence_params.c:128",
     "2.5 m above home: 0.5 m under MOTAR's 3 m arena height; MOTAR flies at 1.0 m"),
    ("GF_SOURCE", "INT32", 0, "navigator/geofence_params.c:90", "estimator global position (GPOS)"),
    ("GF_PREDICT", "INT32", 0, "navigator/geofence_params.c:142", "off (PX4 marks it experimental)"),
    ("COM_OF_LOSS_T", "REAL32", 0.5, "commander/commander_params.c:313", "0.5 s = 5 missed 10 Hz policy steps"),
    ("COM_OBL_RC_ACT", "INT32", 5, "commander/commander_params.c:342", "Hold mode on offboard loss"),
    ("COM_RC_OVERRIDE", "INT32", 3, "commander/commander_params.c:610", "stick override in auto and offboard modes"),
    ("COM_RC_STICK_OV", "REAL32", 30.0, "commander/commander_params.c:625", "30 % (PX4 default)"),
    ("COM_RC_IN_MODE", "INT32", 2, "commander/commander_params.c:186",
     "RC and joystick with fallback: SITL needs both paths; the real vehicle should use 0 (RC only)"),
    ("NAV_RCL_ACT", "INT32", 3, "commander/commander_params.c:806", "Land mode on RC loss"),
    ("COM_RCL_EXCEPT", "INT32", 0, "commander/commander_params.c:820", "no exception: RC loss also acts in offboard"),
    ("NAV_DLL_ACT", "INT32", 3, "commander/commander_params.c:787",
     "Land mode on GCS link loss (also: arming needs a GCS link)"),
    ("COM_KILL_DISARM", "REAL32", 5.0, "commander/commander_params.c:926",
     "disarm 5 s after the kill switch (PX4 default); the motors stop at once (lockdown)"),
    ("RC_MAP_KILL_SW", "INT32", 7, "rc_update/params.c:1442", "kill switch on RC channel 7 (SITL emulation)"),
)
AUX_PARAMS = (
    ("COM_RC_LOSS_T", "REAL32", 0.5, "commander/commander_params.c:143", "PX4 default"),
    ("COM_DL_LOSS_T", "INT32", 5, "commander/commander_params.c:104", "5 s, PX4's minimum (default 10)"),
    ("COM_FAIL_ACT_T", "REAL32", 5.0, "commander/commander_params.c:284", "PX4 default hold before Hold/RTL/Land"),
    ("RC_KILLSWITCH_TH", "REAL32", 0.75, "rc_update/params.c:1966", "PX4 default"),
    ("RC_MAP_ROLL", "INT32", 1, "rc_update/params.c:1168", "SITL RC emulation"),
    ("RC_MAP_PITCH", "INT32", 2, "rc_update/params.c:1200", "SITL RC emulation"),
    ("RC_MAP_THROTTLE", "INT32", 3, "rc_update/params.c:1267", "SITL RC emulation"),
    ("RC_MAP_YAW", "INT32", 4, "rc_update/params.c:1299", "SITL RC emulation"),
)
USER_DECISIONS = ("GF_ACTION", "GF_MAX_HOR_DIST", "GF_MAX_VER_DIST", "GF_SOURCE", "GF_PREDICT", "COM_OF_LOSS_T",
                  "COM_OBL_RC_ACT", "COM_RC_OVERRIDE", "COM_RC_STICK_OV", "COM_RC_IN_MODE", "NAV_RCL_ACT",
                  "COM_RCL_EXCEPT", "NAV_DLL_ACT", "COM_KILL_DISARM", "RC_MAP_KILL_SW", "COM_RC_LOSS_T",
                  "COM_DL_LOSS_T", "COM_FAIL_ACT_T", "RC_KILLSWITCH_TH")
# Left at the PX4 default here, not tested, and still a real-vehicle decision (review r1 m5)
UNTESTED_REAL_VEHICLE_PARAMS = (
    ("CBRK_FLIGHTTERM", "lib/circuit_breaker/circuit_breaker_params.c:101", "flight termination circuit breaker"),
    ("FD_FAIL_R", "commander/failure_detector/failure_detector_params.c:61", "attitude failure detector, roll"),
    ("FD_FAIL_P", "commander/failure_detector/failure_detector_params.c:79", "attitude failure detector, pitch"),
    ("COM_LKDOWN_TKO", "commander/commander_params.c:967", "lockdown after a failed take-off"),
    ("COM_DISARM_LAND", "commander/commander_params.c:214", "auto-disarm after landing"),
    ("MAN_ARM_GESTURE", "manual_control/manual_control_params.c:43", "stick arm/disarm gesture"),
    ("COM_RC_ARM_HYST", "commander/commander_params.c:198", "stick gesture hold time"),
    ("RC_MAP_FLTMODE", "rc_update/params.c:1330", "the safety pilot's mode switch (not tested here)"),
)
MAV_PARAM_TYPE = {"INT32": 6, "REAL32": 9}

ARENA_HALF_XY_M = 20.0
ARENA_Z_M = 3.0
ALT_M = 1.0
HB_TIMEOUT_S = 2.5
HB_CHECK_S = 1.25
GF_SPEED_MS = 2.5
GFV_SPEED_MS = 0.5
LOCKDOWN_WITHIN_S = 1.5
LOCKDOWN_DROP_M = 0.3
FENCE_TOL_M = 0.5
FENCE_TOL_Z_M = 0.3
EARLY_TOL_S = 0.3
PERSIST_S = 2.5                 # 1 Hz heartbeats: at least 2 inside the window (review r1 B1)
MIN_PRE_OBS = 3
MIN_AFTER_OBS = 2
MIN_ECHO = 10                   # PX4 MANUAL_CONTROL echoes needed in the pre-window (a path gate)
ECHO_TOL = 20                   # of 1000 (full stick)
KILL_SHIFT = 12                 # MANUAL_CONTROL.buttons: kill_switch << 12
SWITCH_ON, SWITCH_OFF = 1, 3
RTF_MIN = 0.5                   # a sim-time phase taking longer than duration / RTF_MIN wall: SIM_TOO_SLOW (INVALID)
GAP_LIMIT_S = {"sp": 0.25, "hb": 1.6, "rc": 0.2, "joy": 0.25}
# Cause attribution (v2 review r1 M2). ULog failsafe_flags fields, msg/FailsafeFlags.msg:33,37,38,46. The topic is
# published on a check-result change or every 500 ms from the 10 Hz checks (HealthAndArmingChecks.cpp:101-106,
# Commander.cpp:1720-1724; logged per publication, logged_topics.cpp:64), so a published flag may trail PX4's internal
# flag (and the mode change it causes) by up to about 0.6 s: CAUSE_LAG_S.
CAUSE_FLAGS = ("offboard_control_signal_lost", "manual_control_signal_lost", "gcs_connection_lost",
               "primary_geofence_breached")
CAUSE_LAG_S = 0.7
FLAG_GAP_S = 1.0                # ULog failsafe_flags max gap in the covered window (published at least every ~0.6 s)
ULOG_POSITION_GAP_S = 0.25
ULOG_BOUNDARY_TOL_S = 0.20
SDLOG_STOP_AFTER_DISARM_S = 1.0  # SITL SDLOG_MODE 1 (rcS:177-178): the file closes ~1 s after the first disarm
#                                  (logger.cpp:1145-1150, 1588-1597, 721-727)
# Launch refusals for an idle host and disk (v2 review r1 m2, m3)
MIN_FREE_BYTES = 3 * 1024 ** 3
GPU_ALLOWED_APPS = ("/usr/share/rustdesk/rustdesk",)   # the remote-desktop client's GPU context is tolerated
TIMING = {"tick_s": 0.02, "sp_period_s": 0.1, "hb_period_s": 1.0, "rc_period_s": 0.05, "joy_period_s": 0.05,
          "boot_wait_s": 90.0, "settle_s": 10.0, "param_gap_s": 0.05, "pre_offboard_s": 2.0, "pre_arm_s": 1.0,
          "takeoff_s": 10.0, "pre_s": 5.0, "post_hold_s": 3.5, "stick_pulse_s": 0.3, "fds_reject_s": 3.0,
          "null_window_s": 15.0, "land_s": 20.0, "post_extra_wall_s": 5.0, "stall_wall_s": 2.0,
          "check_every_s": 0.1}

MAV_FRAME_LOCAL_NED = 1
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_RESULT_ACCEPTED, MAV_RESULT_TEMPORARILY_REJECTED = 0, 1
FORCE_MAGIC = 21196
ARMED_FLAG = 128
# LOCAL_POSITION_NED, ATTITUDE, HIL_STATE_QUATERNION 50 Hz (as S3b), EXTENDED_SYS_STATE 5 Hz, RC_CHANNELS 20 Hz,
# MANUAL_CONTROL 20 Hz (mavlink_messages.cpp:363, settable per mavlink_receiver.cpp:2195-2199). HEARTBEAT is fixed 1 Hz.
MESSAGE_INTERVALS = ((32, 20000), (30, 20000), (115, 20000), (245, 200000), (65, 50000), (69, 50000))
TM_POS_XYZ = 8 | 16 | 32 | 64 | 128 | 256 | 1024 | 2048
TM_POS_Z_VEL_XY_YAWRATE = 1 | 2 | 32 | 64 | 128 | 256 | 1024
TM_VEL_XYZ_YAWRATE = 1 | 2 | 4 | 64 | 128 | 256 | 1024
RC_CENTER = (1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000)    # AETR centred, ch5-8 low; ch7 low = kill OFF
RC_DITHER_INDEX, RC_DITHER_US = 7, (1000, 1001)                # unmapped ch8: every frame a change (review r1 M2)
RC_KILL_US = 2000
RC_PITCH_PULSE_US = 1750
JOY_CENTER = (0, 0, 500, 0)                                    # MANUAL_CONTROL x, y, z (500 = centre), r
JOY_PULSE = (500, 0, 500, 0)
EVENT_TEXT = {"GFH": ("Maximum distance from home reached",), "GFV": ("Maximum altitude above home reached",),
              "OVJ": ("Pilot took over using sticks",), "OVR": ("Pilot took over using sticks",),
              "RCL": ("Failsafe activated",), "DLL": ("Connection to ground station lost", "Failsafe activated"),
              "KIL": ("Kill-switch engaged",)}           # geofence.cpp:264,292; Commander.cpp:2766,2634,1534; framework:211

MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED",
              8: "RATTITUDE"}
AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND", 8: "FOLLOW_TARGET",
            9: "PRECLAND", 10: "VTOL_TAKEOFF"}
OBL_ACT_MODE = {0: "POSCTL", 1: "ALTCTL", 2: "STABILIZED", 3: "AUTO_RTL", 4: "AUTO_LAND", 5: "AUTO_LOITER",
                7: "DISARMED"}
GF_ACT_MODE = {2: "AUTO_LOITER", 3: "AUTO_RTL", 5: "AUTO_LAND"}
NAV_ACT_MODE = {1: "AUTO_LOITER", 2: "AUTO_RTL", 3: "AUTO_LAND", 6: "DISARMED"}


def mode_name(custom_mode):
    main, sub = (int(custom_mode) >> 16) & 0xFF, (int(custom_mode) >> 24) & 0xFF
    if main == 4:
        return "AUTO_" + AUTO_SUB.get(sub, str(sub))
    return MAIN_MODES.get(main, "MAIN_%d" % main)


def all_params():
    return SAFETY_PARAMS + AUX_PARAMS


def param_values(table=None):
    return {n: v for n, _, v, _, _ in (table or all_params())}


def encode_param(ptype, value):
    """(param_value, param_type) for PARAM_SET: an INT32 is byte-cast into the float field. Only 0 <= v < 2**23, whose
    bit pattern is zero or a subnormal float, never a NaN."""
    if ptype == "INT32":
        v = int(value)
        if v != value or not 0 <= v < (1 << 23):
            raise ValueError("INT32 parameter value %r outside 0..2**23-1" % (value,))
        return struct.unpack("<f", struct.pack("<i", v))[0], MAV_PARAM_TYPE["INT32"]
    if ptype == "REAL32":
        return float(value), MAV_PARAM_TYPE["REAL32"]
    raise ValueError("unknown parameter type %r" % (ptype,))


def table_problems(table=None):
    table = table or all_params()
    p = []
    names = [n for n, _, _, _, _ in table]
    if len(set(names)) != len(names):
        p.append("duplicate parameter names")
    v = param_values(table)
    if v.get("COM_OBL_RC_ACT") not in OBL_ACT_MODE:
        p.append("COM_OBL_RC_ACT %r has no tested mode" % v.get("COM_OBL_RC_ACT"))
    if v.get("GF_ACTION") not in GF_ACT_MODE:
        p.append("GF_ACTION %r has no tested mode" % v.get("GF_ACTION"))
    for k in ("NAV_RCL_ACT", "NAV_DLL_ACT"):
        if v.get(k) not in NAV_ACT_MODE:
            p.append("%s %r has no tested mode" % (k, v.get(k)))
    if not 0 < v.get("GF_MAX_HOR_DIST", 0) < ARENA_HALF_XY_M:
        p.append("GF_MAX_HOR_DIST must lie inside the arena half width")
    if not ALT_M < v.get("GF_MAX_VER_DIST", 0) < ARENA_Z_M:
        p.append("GF_MAX_VER_DIST must lie between the flight altitude and the arena height")
    if v.get("COM_RC_IN_MODE") != 2:
        p.append("COM_RC_IN_MODE must be 2 in SITL: OVJ needs the joystick path and OVR/RCL/KIL the RC path")
    if not v.get("COM_RC_OVERRIDE", 0) & 2:
        p.append("COM_RC_OVERRIDE bit 1 (offboard) must be set for OVJ/OVR")
    if v.get("COM_RCL_EXCEPT", 0) & 4:
        p.append("COM_RCL_EXCEPT bit 2 exempts offboard: RCL would expect no failsafe")
    if v.get("RC_MAP_KILL_SW") not in range(5, 8):
        p.append("RC_MAP_KILL_SW must be one of the emulated channels 5-7 (8 is the dither channel)")
    for n, t, val, _, _ in table:
        try:
            encode_param(t, val)
        except ValueError as e:
            p.append("%s: %s" % (n, e))
    return p


def scenarios(table=None):
    """Every scenario with its criterion, derived from the parameter values. Times in seconds: earliest_s from t_ref
    (the last message of a stopped stream, else the injection), deadline_s from the injection."""
    v = param_values(table)
    fail_t = v["COM_FAIL_ACT_T"]
    hold = ("AUTO_LOITER",)
    fence, ceil = v["GF_MAX_HOR_DIST"], v["GF_MAX_VER_DIST"]
    null = {"inject": "none", "expect": "UNCHANGED", "window_s": TIMING["null_window_s"]}
    return (
        dict(null, id="N0", what="null: offboard hover, nothing injected", streams=("sp", "hb")),
        dict(null, id="N0R", what="null with RC streamed (smoke only)", streams=("sp", "hb", "rc"), path="rc",
             needs_rc_echo=True, smoke_only=True),
        dict(null, id="N0J", what="null with the joystick streamed (smoke only)", streams=("sp", "hb", "joy"),
             path="joy", smoke_only=True),
        {"id": "OBL", "what": "offboard setpoints stopped", "streams": ("sp", "hb"), "inject": "stop_setpoints",
         "expect": OBL_ACT_MODE[v["COM_OBL_RC_ACT"]], "allowed": hold,
         "earliest_s": v["COM_OF_LOSS_T"] - EARLY_TOL_S, "deadline_s": v["COM_OF_LOSS_T"] + 3.0,
         "cause_flag": "offboard_control_signal_lost"},
        {"id": "GFH", "what": "flying outward at %.1f m/s past the %.0f m horizontal fence" % (GF_SPEED_MS, fence),
         "streams": ("sp", "hb"), "inject": "fly_out", "expect": GF_ACT_MODE[v["GF_ACTION"]], "allowed": (),
         "earliest_s": 0.0, "deadline_s": fence / GF_SPEED_MS + 4.0, "min_trigger_xy_m": fence - FENCE_TOL_M,
         "max_truth_xy_m": ARENA_HALF_XY_M, "event_text": EVENT_TEXT["GFH"], "cause_flag": "primary_geofence_breached"},
        {"id": "GFV", "what": "climbing at %.1f m/s past the %.1f m vertical fence" % (GFV_SPEED_MS, ceil),
         "streams": ("sp", "hb"), "inject": "climb", "expect": GF_ACT_MODE[v["GF_ACTION"]], "allowed": (),
         "earliest_s": 0.0, "deadline_s": (ceil - ALT_M) / GFV_SPEED_MS + 4.0,
         "min_trigger_alt_m": ceil - FENCE_TOL_Z_M, "max_truth_alt_m": ARENA_Z_M, "event_text": EVENT_TEXT["GFV"],
         "cause_flag": "primary_geofence_breached"},
        {"id": "OVJ", "what": "MANUAL_CONTROL pitch pulse (joystick override)", "streams": ("sp", "hb", "joy"),
         "inject": "joystick_pulse", "expect": "POSCTL", "allowed": (), "earliest_s": 0.0, "deadline_s": 3.0,
         "path": "joy", "event_text": EVENT_TEXT["OVJ"], "cause_text": EVENT_TEXT["OVJ"]},
        {"id": "OVR", "what": "RC_CHANNELS_OVERRIDE pitch pulse (RC stick override)", "streams": ("sp", "hb", "rc"),
         "inject": "rc_stick_pulse", "expect": "POSCTL", "allowed": (), "earliest_s": 0.0, "deadline_s": 3.0,
         "needs_rc_echo": True, "path": "rc", "event_text": EVENT_TEXT["OVR"], "cause_text": EVENT_TEXT["OVR"]},
        {"id": "RCL", "what": "RC_CHANNELS_OVERRIDE stopped (RC loss)", "streams": ("sp", "hb", "rc"),
         "inject": "stop_rc", "expect": NAV_ACT_MODE[v["NAV_RCL_ACT"]], "allowed": hold,
         "earliest_s": v["COM_RC_LOSS_T"] - EARLY_TOL_S, "deadline_s": v["COM_RC_LOSS_T"] + fail_t + 3.0,
         "needs_rc_echo": True, "path": "rc", "event_text": EVENT_TEXT["RCL"],
         "cause_flag": "manual_control_signal_lost"},
        {"id": "DLL", "what": "GCS heartbeat stopped (data-link loss)", "streams": ("sp", "hb"),
         "inject": "stop_gcs_heartbeat", "expect": NAV_ACT_MODE[v["NAV_DLL_ACT"]], "allowed": hold,
         "earliest_s": v["COM_DL_LOSS_T"] - 0.5,
         "deadline_s": v["COM_DL_LOSS_T"] + HB_TIMEOUT_S + HB_CHECK_S + fail_t + 3.0, "event_text": EVENT_TEXT["DLL"],
         "cause_flag": "gcs_connection_lost"},
        {"id": "KIL", "what": "RC_CHANNELS_OVERRIDE kill channel %d high" % v["RC_MAP_KILL_SW"],
         "streams": ("sp", "hb", "rc"), "inject": "kill_switch", "kill_channel": v["RC_MAP_KILL_SW"],
         "expect": "DISARMED", "allowed": None, "earliest_s": 0.0, "deadline_s": v["COM_KILL_DISARM"] + 2.0,
         "lockdown_within_s": LOCKDOWN_WITHIN_S, "lockdown_drop_m": LOCKDOWN_DROP_M, "needs_rc_echo": True,
         "path": "rc", "needs_kill_on": True, "event_text": EVENT_TEXT["KIL"], "cause_text": EVENT_TEXT["KIL"]},
        {"id": "FDS", "what": "GCS disarm in the air: non-forced (must be refused), then forced 21196",
         "streams": ("sp", "hb"), "inject": "gcs_disarm", "expect": "FDS", "reject_window_s": TIMING["fds_reject_s"],
         "deadline_s": 2.0},
    )


SCHEDULE = tuple(s["id"] for s in scenarios() if not s.get("smoke_only"))
SMOKE_SCHEDULE = ("N0", "N0R", "N0J")


def schedule():
    return [{"scenario": sid} for sid in SCHEDULE]


def smoke_schedule():
    return [{"scenario": sid} for sid in SMOKE_SCHEDULE]


def live_flags(scen):
    """The cause flags that must read false while their input is being fed: offboard with setpoints, GCS with the
    heartbeat, manual control with RC or joystick, the geofence always. manual_control_signal_lost is true throughout a
    scenario without RC or joystick (rcAndDataLinkCheck.cpp:42-58), so it is not live there."""
    s = set(scen["streams"])
    on = {"offboard_control_signal_lost": "sp" in s, "manual_control_signal_lost": bool(s & {"rc", "joy"}),
          "gcs_connection_lost": "hb" in s, "primary_geofence_breached": True}
    return [f for f in CAUSE_FLAGS if on[f]]


def _gpu_app_name(line):
    parts = [x.strip() for x in str(line).split(",")]
    return parts[1] if len(parts) > 1 else parts[0]


def gpu_problems(host):
    """Idle-host rule (AGENTS.md, real-time SITL acceptance): no GPU compute process except GPU_ALLOWED_APPS; an
    unreadable GPU process list refuses too (v2 review r1 m3)."""
    apps = (host or {}).get("gpu_compute_apps")
    if not isinstance(apps, list):
        return ["GPU compute processes unknown: %s" % (apps,)]
    others = [a for a in apps if str(a).strip() and _gpu_app_name(a) not in GPU_ALLOWED_APPS]
    return ["GPU compute process running: %s" % others] if others else []


def free_bytes(path):
    """Free bytes on the filesystem that holds path (its nearest existing ancestor)."""
    p = pathlib.Path(path)
    while not p.exists() and p != p.parent:
        p = p.parent
    return shutil.disk_usage(str(p)).free


def host_snapshot():
    """Host-wide load and the GPU's compute processes (review r1 l2); failures are recorded, not raised."""
    out = {}
    try:
        out["loadavg"] = list(os.getloadavg())
    except OSError as e:
        out["loadavg"] = "unreadable: %s" % e
    try:
        r = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name,used_memory", "--format=csv,noheader"],
                           capture_output=True, text=True, timeout=10)
        out["gpu_compute_apps"] = r.stdout.strip().splitlines() if r.returncode == 0 else "rc %d" % r.returncode
    except (OSError, subprocess.TimeoutExpired) as e:
        out["gpu_compute_apps"] = "unavailable: %s" % e
    return out


# ------------------------------------------------------------------------------------------- MAVLink link

def init_logs(link):
    link.hb_log, link.statustext, link.acks, link.rc_echo, link.landed, link.man_echo = [], [], [], [], [], []
    link.other_component_hb = 0
    link.rx_packets, link.tx_calls = [], []
    link.last_rx = {}
    link.log_clock = time.monotonic


def sim_now(link):
    return link.pos[0] if getattr(link, "pos", None) is not None else None


def handle_message(link, msg):
    """LOCAL_POSITION_NED, HIL_STATE_QUATERNION, HEARTBEAT, ATTITUDE and PARAM_VALUE parse exactly as S3b's ParamLink (a
    test compares the two); the rest and the logs are S5's, stamped with PX4 simulation time."""
    k = msg.get_type()
    now = link.log_clock()
    src = [msg.get_srcSystem(), msg.get_srcComponent()]
    payload = msg.to_dict() if hasattr(msg, "to_dict") else {k: v for k, v in vars(msg).items() if not k.startswith("_")}
    pos_age = now - link.last_rx["LOCAL_POSITION_NED"] if "LOCAL_POSITION_NED" in link.last_rx else None
    native = {n: getattr(msg, n) for n in ("time_boot_ms", "time_usec") if hasattr(msg, n)}
    link.rx_packets.append({"type": k, "source": src, "seq": msg.get_seq() if hasattr(msg, "get_seq") else None,
                            "mono": now, "epoch": time.time(), "sim_t": sim_now(link), "pos_age_s": pos_age,
                            "native": native or "NOT_RECORDED", "payload": json_safe(payload),
                            "packet_hex": bytes(msg.get_msgbuf()).hex() if hasattr(msg, "get_msgbuf") else "NOT_RECORDED"})
    if src != [link.m.target_system, link.m.target_component]:
        if k == "HEARTBEAT" and src[0] == link.m.target_system:
            link.other_component_hb += 1
        return
    link.last_rx[k] = now
    if k == "LOCAL_POSITION_NED":
        link.pos = (msg.time_boot_ms * 1e-3, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
    elif k == "HIL_STATE_QUATERNION":
        q = msg.attitude_quaternion
        link.truth = (time.time(), msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3, q[0], q[1], q[2], q[3],
                      msg.vx * 1e-2, msg.vy * 1e-2, msg.vz * 1e-2)
    elif k == "HEARTBEAT" and msg.get_srcSystem() == link.m.target_system:
        link.hb = (msg.base_mode, msg.custom_mode)
        if msg.get_srcComponent() == link.m.target_component:
            link.hb_log.append((sim_now(link), time.time(), int(msg.base_mode), int(msg.custom_mode)))
    elif k == "ATTITUDE":
        link.att = (msg.time_boot_ms * 1e-3, msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed,
                    msg.yawspeed)
    elif k == "PARAM_VALUE":
        pid = msg.param_id
        pid = pid.decode() if isinstance(pid, bytes) else pid
        link.params[pid.rstrip("\x00")] = SF.decode_param(msg)
    elif k == "STATUSTEXT":
        text = msg.text.decode(errors="replace") if isinstance(msg.text, bytes) else str(msg.text)
        link.statustext.append((sim_now(link), int(msg.severity), text.rstrip("\x00")))
    elif k == "COMMAND_ACK":
        link.acks.append((sim_now(link), int(msg.command), int(msg.result)))
    elif k == "RC_CHANNELS":
        link.rc_echo.append((sim_now(link),) + tuple(int(getattr(msg, "chan%d_raw" % i)) for i in range(1, 9)))
    elif k == "MANUAL_CONTROL" and msg.get_srcSystem() == link.m.target_system:
        link.man_echo.append((sim_now(link), int(msg.x), int(msg.y), int(msg.z), int(msg.r), int(msg.buttons)))
    elif k == "EXTENDED_SYS_STATE":
        link.landed.append((sim_now(link), int(msg.landed_state)))


def _safety_link_class():
    base = SF._param_link_class()

    class _SafetyLink(base):
        def __init__(self):
            super().__init__()
            init_logs(self)

        def rx(self):
            while not self.stop:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
                if msg is not None:
                    handle_message(self, msg)

    return _SafetyLink


def SafetyLink():
    return _safety_link_class()()


class Streams:
    """The harness's periodic outputs from the main loop: sp (offboard setpoints, 10 Hz), hb (GCS heartbeat, 1 Hz), rc
    (RC_CHANNELS_OVERRIDE, 20 Hz, channel 8 dithered 1000/1001) and joy (MANUAL_CONTROL, 20 Hz). A stream is on while
    its value is set; stopping it is an injection. Send gaps of running streams are tracked per phase."""

    def __init__(self, L, timing):
        self.L = L
        self.period = {"sp": timing["sp_period_s"], "hb": timing["hb_period_s"], "rc": timing["rc_period_s"],
                       "joy": timing["joy_period_s"]}
        self.sp, self.hb, self.rc, self.joy = None, False, None, None
        self.last = dict.fromkeys(self.period)
        self.last_sim = dict.fromkeys(self.period)
        self.sent = collections.Counter()
        self.track = False
        self.max_gap = dict.fromkeys(self.period, 0.0)
        self.sim_seen = None
        self.tx_streams = []

    def on(self, k):
        return {"sp": self.sp is not None, "hb": bool(self.hb), "rc": self.rc is not None,
                "joy": self.joy is not None}[k]

    def gaps(self, reset=True):
        g = dict(self.max_gap)
        if reset:
            self.max_gap = dict.fromkeys(self.period, 0.0)
        return g

    def tick(self, now):
        for k in self.period:
            if not self.on(k):
                self.last[k] = None
                continue
            if self.last[k] is not None and now - self.last[k] < self.period[k] - 1e-9:
                continue
            if self.track and self.last[k] is not None:
                self.max_gap[k] = max(self.max_gap[k], now - self.last[k])
            self._send(k)
            self.tx_streams.append({"stream": k, "mono": now, "sim_t": sim_now(self.L)})
            self.last[k] = now
            self.last_sim[k] = sim_now(self.L)
            self.sent[k] += 1

    def _send(self, k):
        L = self.L
        ts, tc = L.m.target_system, L.m.target_component
        if k == "sp":
            mask, x, y, z, vx, vy, vz, wz = self.sp
            L.send(L.m.mav.set_position_target_local_ned_send, 0, ts, tc, MAV_FRAME_LOCAL_NED, mask, x, y, z, vx, vy,
                   vz, 0, 0, 0, 0, wz)
        elif k == "hb":
            L.send(L.m.mav.heartbeat_send, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
        elif k == "rc":
            ch = [int(c) for c in self.rc]
            ch[RC_DITHER_INDEX] = RC_DITHER_US[self.sent["rc"] % 2]
            L.send(L.m.mav.rc_channels_override_send, ts, tc, *ch)
        else:
            x, y, z, r = self.joy
            L.send(L.m.mav.manual_control_send, ts, int(x), int(y), int(z), int(r), 0)


# ------------------------------------------------------------------------------------------- one scenario

def run_loop(L, session, S, clock, sleep, timing, wall_s=None, until=None, on_tick=None, sample=None):
    """Tick the streams until until() or for wall_s; check the owned session every check_every_s and that PX4 simulation
    time advances (PX4_TIME_STALLED after stall_wall_s, tracked on S across loops). True if until() held."""
    t0 = clock()
    next_check = t0
    if S.sim_seen is None:
        S.sim_seen = [sim_now(L), t0]
    while True:
        now = clock()
        if on_tick is not None:
            on_tick(now)
        S.tick(now)
        if sample is not None:
            sample()
        if until is not None and until():
            return True
        if wall_s is not None and now - t0 >= wall_s:
            return False
        if now >= next_check:
            session.check(required=("gzserver", "px4"))
            next_check = now + timing["check_every_s"]
        sim = sim_now(L)
        if sim != S.sim_seen[0]:
            S.sim_seen = [sim, now]
        elif sim is not None and now - S.sim_seen[1] > timing["stall_wall_s"]:
            raise RuntimeError("PX4_TIME_STALLED: sim time %.3f s for %.1f s wall" % (sim, now - S.sim_seen[1]))
        sleep(timing["tick_s"])


def new_trace():
    return {"rx_packets": [], "tx_calls": [], "samples": [], "hb": [], "pos": [], "truth": [], "statustext": [], "acks": [], "rc_echo": [], "man_echo": [], "landed": [],
            "events": [], "home_ned": None, "truth_origin": None}


def _truth_local(L, origin):
    tr = L.truth
    lat0, lon0, alt0 = origin
    R_E = 6371000.0
    dn = math.radians(tr[1] - lat0) * R_E
    de = math.radians(tr[2] - lon0) * R_E * math.cos(math.radians(lat0))
    return de, dn, tr[3] - alt0


def _autopilot_state(L):
    """Armed and mode from the autopilot's last HEARTBEAT (component-filtered hb_log; review r1 l6)."""
    if not L.hb_log:
        return False, None
    _, _, base, custom = L.hb_log[-1]
    return bool(base & ARMED_FLAG), mode_name(custom)


def _first_hb(L, t0, pred):
    for t, _, base, custom in list(L.hb_log):
        if t is not None and t >= t0 and pred(bool(base & ARMED_FLAG), mode_name(custom)):
            return t
    return None


def _reached(scen, L, t_inj):
    exp = scen["expect"]
    if exp in ("DISARMED", "FDS"):
        return _first_hb(L, t_inj, lambda armed, mode: not armed)
    if exp == "UNCHANGED":
        return None
    return _first_hb(L, t_inj, lambda armed, mode: armed and mode == exp)


PX4_PYTHON = pathlib.Path("/home/fair/miniconda3/envs/px4/bin/python")
# Use the installed PX4 parser; do not infer ULog timestamp layout or install a new dependency.
ULOG_SCRIPT = r"""
import contextlib, hashlib, json, sys
import numpy as np
import pyulog, pyulog.core
from pyulog import ULog
with contextlib.redirect_stdout(sys.stderr):
    u = ULog(sys.argv[1], disable_str_exceptions=False)
topics = []
for d in u.data_list:
    ts = d.data.get('timestamp')
    topics.append({'name': d.name, 'multi_id': int(d.multi_id),
                   'fields': sorted(d.data), 'count': len(ts) if ts is not None else 0,
                   'first_us': int(ts[0]) if ts is not None and len(ts) else None,
                   'last_us': int(ts[-1]) if ts is not None and len(ts) else None,
                   'max_gap_s': float(np.max(np.diff(ts.astype(np.int64)))) / 1e6 if ts is not None and len(ts)>1 else None,
                   'monotonic': bool(np.all(np.diff(ts.astype(np.int64)) >= 0)) if ts is not None else False})
names = json.loads(sys.argv[2])
flags = None
fs = sorted((d for d in u.data_list if d.name == 'failsafe_flags'), key=lambda d: d.multi_id)
if fs:
    d = fs[0]
    missing = [n for n in names if n not in d.data]
    ts = d.data.get('timestamp')
    rows = [] if missing or ts is None else [[int(ts[i])] + [int(bool(d.data[n][i])) for n in names] for i in range(len(ts))]
    flags = {'multi_id': int(d.multi_id), 'fields': names, 'missing_fields': missing, 'rows': rows}
print(json.dumps({'parser': pyulog.__file__, 'parser_core_sha256': hashlib.sha256(open(pyulog.core.__file__, 'rb').read()).hexdigest(),
                  'file_corruption': u.file_corruption, 'topics': topics, 'failsafe_flags': flags,
                  'dropouts': [{'timestamp_us': int(d.timestamp), 'duration_ms': int(d.duration)} for d in u.dropouts]}))
"""


def ulog_inventory(path):
    path = pathlib.Path(path)
    out = {"path": str(path), "bytes": path.stat().st_size, "sha256": SF.sha256_file(path)}
    try:
        with path.open("rb") as f:
            if f.read(7) != b"ULog\x01\x12\x35":
                raise ValueError("invalid ULog magic")
            if len(f.read(9)) != 9:
                raise ValueError("truncated ULog header")
            while True:
                header = f.read(3)
                if not header:
                    break
                if len(header) != 3:
                    raise ValueError("truncated ULog record header")
                size = struct.unpack("<H", header[:2])[0]
                if len(f.read(size)) != size:
                    raise ValueError("truncated ULog record payload")
        r = subprocess.run([str(PX4_PYTHON), "-B", "-c", ULOG_SCRIPT, str(path), json.dumps(list(CAUSE_FLAGS))],
                           capture_output=True, text=True,
                           timeout=30, env=dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1",
                                                OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1"))
        if r.returncode:
            raise ValueError("parser rc %d: %s" % (r.returncode, r.stderr[-1000:]))
        out.update(json.loads(r.stdout))
        out["parser_stderr"] = r.stderr[-2000:]
        out["parse_ok"] = not out["file_corruption"]
    except (OSError, ValueError, subprocess.TimeoutExpired) as e:
        out.update(parse_ok=False, error=str(e))
    return out


def _ulogs(ep_dir):
    root = pathlib.Path(ep_dir) / "rootfs"
    return [ulog_inventory(p) for p in sorted(root.rglob("*.ulg"))] if root.is_dir() else []


def ulog_coverage_end(res, trace=None):
    """End of the window the ULog must cover (v2 review r1 B1). PX4 SITL logs from boot until the first disarm
    (SDLOG_MODE 1, rcS:177-178) and closes the file about SDLOG_STOP_AFTER_DISARM_S later, so a scenario that disarms
    inside the observation window (KIL, FDS; RCL/DLL if the land-disarm falls inside it) is covered up to its first
    disarmed autopilot heartbeat; a scenario that stays armed must be covered to the observation end."""
    lo, hi = res.get("t_pre_start"), res.get("t_observation_end")
    if trace is None or lo is None or hi is None:
        return hi, "observation_end"
    for row in bounded_rows(trace, res).get("hb", []):
        if row[0] is not None and lo <= row[0] <= hi and not int(row[2]) & ARMED_FLAG:
            return row[0], "first_disarmed_heartbeat"
    return hi, "observation_end"


def ulog_flag_rows(logs):
    """[(t_s, {flag: bool})] from the parsed ULog failsafe_flags (only logs whose fields are exactly CAUSE_FLAGS)."""
    rows = []
    for log in logs or []:
        ff = log.get("failsafe_flags") or {}
        if ff.get("missing_fields") or list(ff.get("fields") or []) != list(CAUSE_FLAGS):
            continue
        rows += [(r[0] * 1e-6, dict(zip(CAUSE_FLAGS, (bool(v) for v in r[1:])))) for r in ff.get("rows", [])]
    return sorted(rows, key=lambda r: r[0])


def ulog_problems(logs, scen, res=None, trace=None):
    required = {"vehicle_status", "failsafe_flags", "actuator_armed", "vehicle_local_position", "vehicle_command"}
    if scen.get("path"):
        required.add("manual_control_setpoint")
    if "rc" in scen["streams"]:
        required |= {"input_rc", "manual_control_switches"}
    if scen["id"] == "FDS":
        required.add("vehicle_command_ack")
    if not logs:
        return ["NO_ULOG"]
    problems, seen = [], set()
    for log in logs:
        if not log.get("parse_ok"):
            problems.append("ULOG_UNREADABLE")
        if log.get("dropouts"):
            problems.append("ULOG_DROPOUT")
        for topic in log.get("topics", []):
            if topic.get("count", 0) > 0 and topic.get("monotonic"):
                seen.add(topic["name"])
    problems += ["ULOG_TOPIC_MISSING " + n for n in sorted(required - seen)]
    flag_logs = [u.get("failsafe_flags") for u in logs if u.get("failsafe_flags")]
    if "failsafe_flags" in seen and (not flag_logs or any(f.get("missing_fields") for f in flag_logs)):
        problems.append("ULOG_FLAGS_FIELDS_MISSING")
    if res and res.get("t_observation_end") is not None and res.get("t_pre_start") is not None:
        # SITL SDLOG_PROFILE 131 logs vehicle_local_position at full rate (36 historical SITL ULogs: max gap 0.008 s);
        # the 0.25 s bound is a coverage bound, not a rate claim. The window ends at ulog_coverage_end (B1).
        lo = res["t_pre_start"]
        hi, _ = ulog_coverage_end(res, trace)
        pos = [t for u in logs for t in u.get("topics", []) if t["name"] == "vehicle_local_position"]
        if not any(t.get("first_us") is not None and t["first_us"] * 1e-6 <= lo + ULOG_BOUNDARY_TOL_S
                   and t["last_us"] * 1e-6 >= hi - ULOG_BOUNDARY_TOL_S and t.get("max_gap_s") is not None
                   and t["max_gap_s"] <= ULOG_POSITION_GAP_S for t in pos):
            problems.append("ULOG_POSITION_COVERAGE")
        if "failsafe_flags" in seen and "ULOG_FLAGS_FIELDS_MISSING" not in problems:
            st = cadence([t for t, _ in ulog_flag_rows(logs)], lo, hi)
            if st["count"] < 2 or st["max_gap_s"] > FLAG_GAP_S + 1e-9:
                problems.append("ULOG_FLAGS_COVERAGE")
    return problems


def json_safe(value):
    if isinstance(value, bytes):
        return {"bytes_hex": value.hex()}
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": repr(value)}
    return value


def instrument_link(L, clock):
    L.log_clock = clock
    original_send, original_cmd = L.send, L.cmd

    def record(kind, fn, args):
        rec = {"kind": kind, "name": getattr(fn, "__name__", str(fn)), "args": json_safe(args),
               "mono_entry": clock(), "epoch_entry": time.time(), "sim_t": sim_now(L),
               "destination": [L.m.target_system, L.m.target_component]}
        L.tx_calls.append(rec)
        try:
            return fn(*args)
        except BaseException as e:
            rec["error"] = type(e).__name__ + ": " + str(e)
            raise
        finally:
            rec["mono_return"], rec["epoch_return"] = clock(), time.time()
    # Preserve the wire function name; command records additionally expose command+parameters.
    def send(fn, *a):
        def invoke(*b):
            return original_send(fn, *b)
        invoke.__name__ = getattr(fn, "__name__", str(fn))
        return record("send", invoke, a)
    L.send = send
    L.cmd = lambda *a: record("command", original_cmd, a)


def bounded_rows(trace, res):
    end = res.get("t_observation_end")
    if end is None:
        return trace
    out = dict(trace)
    for key in ("hb", "pos", "truth", "statustext", "acks", "rc_echo", "man_echo", "landed"):
        rows = trace.get(key, [])[:res.get("observation_counts", {}).get(key)]
        out[key] = [r for r in rows if r[0] is not None and r[0] <= end]
    return out


def cadence(times, start, end):
    times = sorted(t for t in times if start <= t <= end)
    points = [start] + times + [end]
    return {"count": len(times), "max_gap_s": max((b-a for a, b in zip(points, points[1:])), default=0.0),
            "first": times[0] if times else None, "last": times[-1] if times else None}


def command_inventory(trace):
    """Requests, ACK status and actual reported state remain separate. Raw packets preserve ambiguous/late ACKs."""
    calls = [r for r in trace.get("tx_calls", []) if r["kind"] == "command"]
    packets = trace.get("rx_packets", [])
    out = []
    labels = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
              5: "IN_PROGRESS", 6: "CANCELLED"}
    for i, r in enumerate(calls):
        command = r["args"][0]
        end = next((n["mono_entry"] for n in calls[i+1:] if n["args"][0] == command), float("inf"))
        acks = [p for p in packets if p["type"] == "COMMAND_ACK" and p["mono"] >= r["mono_entry"]
                and p["mono"] < end and p["payload"].get("command") == command
                and p["source"] == r["destination"]]
        reports = [p for p in packets if p["type"] == "HEARTBEAT" and p["source"] == r["destination"]
                   and p["mono"] >= r["mono_entry"]]
        first = reports[0] if reports else None
        out.append({"command": command, "args": r["args"][1:], "mono_entry": r["mono_entry"],
                    "mono_return": r["mono_return"], "sim_t": r["sim_t"],
                    "ack_results": [labels.get(p["payload"].get("result"), str(p["payload"].get("result"))) for p in acks],
                    "ack_count": len(acks), "duplicate_ack_count": max(0, len(acks)-1),
                    "ack_state": "RECEIVED" if acks else "NOT_RECORDED", "timeout_s": None,
                    "first_reported_actual": {"mode": mode_name(first["payload"]["custom_mode"]),
                        "armed": bool(first["payload"]["base_mode"] & ARMED_FLAG), "mono": first["mono"],
                        "sim_t": first["sim_t"]} if first else "NOT_RECORDED"})
    return out


def recording_contract(scen, res, trace):
    problems, inventory = [], {}
    lo, hi = res.get("t_pre_start"), res.get("t_observation_end")
    if lo is None or hi is None or hi <= lo:
        return {"problems": ["NO_OBSERVATION_BOUNDARY"], "inventory": {}}
    packets = [p for p in trace.get("rx_packets", [])[:res.get("observation_counts", {}).get("rx_packets")] if isinstance(p.get("sim_t"), (int, float))
               and lo <= p["sim_t"] <= hi and p["source"] == [1, 1]]
    # Requests are 50 Hz LPN/attitude/HIL, fixed 1 Hz heartbeat, 20 Hz RC/manual. Allow >2 periods of jitter.
    limits = {"LOCAL_POSITION_NED": .10, "ATTITUDE": .10, "HIL_STATE_QUATERNION": .10, "HEARTBEAT": 1.6}
    for name, limit in limits.items():
        rows = [p for p in packets if p["type"] == name]
        stamps = [p["sim_t"] for p in rows]
        if any(b < a for a, b in zip(stamps, stamps[1:])):
            problems.append("RX_TIME_REVERSED " + name)
        if any(isinstance(v, dict) and "nonfinite" in v for p in rows for v in p["payload"].values()):
            problems.append("RX_NONFINITE " + name)
        stats = inventory[name] = cadence(stamps, lo, hi)
        stats["max_position_stamp_age_s"] = max((p["pos_age_s"] for p in rows if p["pos_age_s"] is not None), default=None)
        if stats["count"] < 2 or stats["max_gap_s"] > limit + 1e-8:
            problems.append("RX_COVERAGE " + name)
        if stats["max_position_stamp_age_s"] is None or stats["max_position_stamp_age_s"] > .10:
            problems.append("RX_POSITION_STAMP_STALE " + name)
    for kind, limit in (("RC_CHANNELS", .20), ("MANUAL_CONTROL", .25)):
        if kind == "RC_CHANNELS" and "rc" not in scen["streams"] or kind == "MANUAL_CONTROL" and not scen.get("path"):
            continue
        end = res["t_inject"] if scen["inject"] == "stop_rc" else hi
        stats = inventory[kind] = cadence([p["sim_t"] for p in packets if p["type"] == kind], lo, end)
        if stats["count"] < MIN_ECHO or stats["max_gap_s"] > limit + 1e-8:
            problems.append("RX_COVERAGE " + kind)
    for phase, start, end in (("pre", lo, res["t_inject"]), ("post", res["t_inject"], hi)):
        for stream in scen["streams"]:
            if phase == "post" and scen["inject"] == {"sp": "stop_setpoints", "rc": "stop_rc", "hb": "stop_gcs_heartbeat"}.get(stream):
                continue
            times = [r["sim_t"] for r in trace.get("tx_streams", []) if r["stream"] == stream and r["sim_t"] is not None]
            stats = inventory["TX_" + phase + "_" + stream] = cadence(times, start, end)
            if not stats["count"] or stats["max_gap_s"] > GAP_LIMIT_S[stream] + 1e-8:
                problems.append("TX_COVERAGE " + phase + " " + stream)
    samples = [r for r in trace.get("samples", []) if lo <= r["sim_t"] <= hi]
    if not samples or any(r["truth_age_s"] is None or r["truth_age_s"] > .10 for r in samples):
        problems.append("TRUTH_SAMPLE_STALE")
    inventory["truth_max_sample_age_s"] = max((r["truth_age_s"] for r in samples if r["truth_age_s"] is not None), default=None)
    inventory["commands"] = command_inventory(trace)
    inventory["contact"] = "NOT_RECORDED: no Gazebo contact recorder; altitude/position are not contact"
    inventory["event_driven_counts"] = {k: sum(p["type"] == k for p in packets) for k in ("STATUSTEXT", "COMMAND_ACK")}
    return {"problems": sorted(set(problems)), "inventory": inventory}



PX4_BINARY = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/build/px4_sitl_default/bin/px4")


def stop_owned_logger(session, plan, expected_binary=PX4_BINARY):
    """Stop+join this exclusive owned instance's logger before PX4 teardown. No global pkill or shared rootfs writes.

    PX4 ModuleBase::stop_command waits up to5s and returns failure on forced stop; logger joins its writer thread.
    The owned-session lock/port checks establish that the instance is ours; use the exact launched binary's alias.
    expected_binary is a parameter only so that the tests can build a host-independent positive/negative pair.
    """
    session.check(required=("px4", "gzserver"))
    rootfs = pathlib.Path(session.episode_dir) / "rootfs"
    if pathlib.Path(plan["px4"]["cwd"]) != rootfs:
        raise RuntimeError("LOGGER_STOP_ROOTFS_MISMATCH")
    expected_binary = pathlib.Path(expected_binary)
    argv = plan["px4"]["argv"]
    expected_argv = [str(expected_binary), "-d", str(expected_binary.parent.parent / "etc")]
    if argv != expected_argv:
        raise RuntimeError("LOGGER_STOP_INSTANCE_PLAN_MISMATCH")
    owned = session._procs.get("px4", {}).get("record", {})
    if owned.get("argv") != argv or pathlib.Path(owned.get("cwd", "")) != rootfs:
        raise RuntimeError("LOGGER_STOP_OWNERSHIP_MISMATCH")
    binary = pathlib.Path(argv[0])
    alias = binary.with_name("px4-logger")
    if not alias.is_file() or alias.resolve() != binary.resolve():
        raise RuntimeError("LOGGER_STOP_BINARY_MISMATCH")
    started = time.monotonic()
    session.start("logger_stop", [str(alias), "stop"], cwd=rootfs, env=plan["px4"]["env"])
    rc = session.wait("logger_stop", timeout=15.)
    out = {"returncode": rc, "wall_s": time.monotonic() - started, "argv": [str(alias), "stop"],
           "binary_sha256": SF.sha256_file(alias), "completed_before_px4_teardown": True}
    if rc != 0:
        raise RuntimeError("LOGGER_STOP_FAILED %s" % out)
    return out


def run_scenario(OS, V0, scen, ep_dir, link_factory=None, clock=time.monotonic, sleep=time.sleep, timing=None,
                 table=None):
    timing = dict(TIMING, **(timing or {}))
    link_factory = link_factory or SafetyLink
    table = table or all_params()
    res = {"scenario": scen["id"], "outcome": None}
    trace = new_trace()
    L = None
    base_env = dict(os.environ)
    try:
        with OS.OwnedSession(ep_dir, rss_cap_bytes=8 * 1024**3) as session:
            name = "motar_s5_safety"
            base = (V0.WORLDS / "empty.world").read_text()
            session.write_world(name, SF.world_text(base, [], [], (0.0, 0.0, 0.0), 0.0, V0.ODE_ITERS, V0.SPAWN,
                                                    V0.rot))
            plan = OS.direct_launch_plan(session.episode_dir, name, base_env, model=MODEL)
            res["runtime_binding"] = OS.runtime_binding(plan, airframe=AIRFRAME)
            parser_core = PX4_PYTHON.parent.parent / "lib/python3.8/site-packages/pyulog/core.py"
            px4_src = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot")
            for key, path in (("ulog_parser_core", parser_core), ("ulog_parser_python", PX4_PYTHON),
                              ("logger_source", px4_src / "src/modules/logger/logger.cpp"),
                              ("logger_topics_source", px4_src / "src/modules/logger/logged_topics.cpp"),
                              ("module_stop_source", px4_src / "platforms/common/include/px4_platform_common/module.h")):
                res["runtime_binding"]["files"][key] = {"path": str(path), "bytes": path.stat().st_size,
                                                         "sha256": SF.sha256_file(path)}
            OS.launch_px4_gazebo(session, plan)
            session.sleep(3)
            L = link_factory()
            for key, module in (("pymavlink_mavutil", getattr(L, "mu", None)),
                                ("pymavlink_dialect", sys.modules.get(type(L.m.mav).__module__))):
                path = pathlib.Path(getattr(module, "__file__", ""))
                if path.is_file():
                    res["runtime_binding"]["files"][key] = {"path": str(path), "bytes": path.stat().st_size,
                                                             "sha256": SF.sha256_file(path)}
            instrument_link(L, clock)
            try:
                if L.m.wait_heartbeat(timeout=120) is None:
                    raise RuntimeError("no heartbeat")
                th = threading.Thread(target=L.rx, daemon=True)
                th.start()
                try:
                    _fly(V0, L, session, scen, res, trace, clock, sleep, timing, table)
                    res["logger_stop"] = stop_owned_logger(session, plan)
                finally:
                    L.stop = True
                    th.join(timeout=5.0)
                    res["rx_thread_alive_after_join"] = th.is_alive()
                if res["rx_thread_alive_after_join"]:
                    raise RuntimeError("RX_THREAD_STUCK: the MAVLink receiver did not stop")
            finally:
                L.stop = True
                getattr(L.m, "close", lambda: None)()
                res["link_closed"] = True
    except Exception as e:                                               # noqa: BLE001
        res["outcome"] = "HARNESS_INVALID" if "HARNESS_INVALID" in str(e) else "ERROR"
        res["error"] = "%s: %s" % (e.__class__.__name__, e)
    finally:
        if L is not None:
            L.stop = True
            for key, attr in (("hb", "hb_log"), ("statustext", "statustext"), ("acks", "acks"),
                              ("rc_echo", "rc_echo"), ("man_echo", "man_echo"), ("landed", "landed")):
                trace[key] = [list(x) for x in getattr(L, attr, [])]
            trace["rx_packets"], trace["tx_calls"] = L.rx_packets, L.tx_calls
            res["other_component_heartbeats"] = getattr(L, "other_component_hb", None)
        ep_dir = pathlib.Path(ep_dir)
        if ep_dir.is_dir():
            write_new(ep_dir / "trace.json", json.dumps(trace) + "\n")
            res["trace_sha256"] = SF.sha256_file(ep_dir / "trace.json")
            receipt = ep_dir / "session.json"
            res["session_receipt_sha256"] = SF.sha256_file(receipt) if receipt.is_file() else None
            res["ulogs"] = _ulogs(ep_dir)
            res["recording"] = recording_contract(scen, res, trace)
            res["recording"]["problems"] += ulog_problems(res["ulogs"], scen, res, trace)
            res["dataman_removed_bytes"] = SF.remove_dataman(ep_dir)
    res.update(assess(scen, res, trace))
    return res


def _fly(V0, L, session, scen, res, trace, clock, sleep, timing, table):
    S = Streams(L, timing)
    S.hb = True
    if "rc" in scen["streams"]:
        S.rc = list(RC_CENTER)
    if "joy" in scen["streams"]:
        S.joy = JOY_CENTER
    for mid, us in MESSAGE_INTERVALS:
        L.cmd(MAV_CMD_SET_MESSAGE_INTERVAL, mid, us)
    origin = {}

    def sample():
        if L.pos is None:
            return
        if not trace["pos"] or trace["pos"][-1][0] != L.pos[0]:
            trace["pos"].append([L.pos[0], L.pos[1], L.pos[2], L.pos[3]])
            if origin and L.truth is not None:
                trace["samples"].append({"sim_t": L.pos[0], "mono": clock(),
                                         "truth_age_s": clock() - L.last_rx["HIL_STATE_QUATERNION"] if "HIL_STATE_QUATERNION" in L.last_rx else None})
                trace["truth"].append([L.pos[0], *_truth_local(L, origin["o"])])

    def event(name, **kw):
        trace["events"].append(dict(name=name, sim_t=sim_now(L), wall=clock(), **kw))

    loop = lambda **kw: run_loop(L, session, S, clock, sleep, timing, sample=sample, **kw)       # noqa: E731

    def sim_phase(duration_s, done=None, on_tick=None):
        """Run until done() or duration_s of PX4 simulation time; a wall cap of duration_s / RTF_MIN marks the scenario
        SIM_TOO_SLOW (review r1 m3)."""
        end = sim_now(L) + duration_s
        ok = loop(wall_s=duration_s / RTF_MIN + timing["post_extra_wall_s"], on_tick=on_tick,
                  until=lambda: (done is not None and done()) or sim_now(L) >= end)
        if not ok:
            res["sim_too_slow"] = True
        return ok

    if not loop(wall_s=timing["boot_wait_s"], until=lambda: L.pos is not None and L.att is not None
                and L.truth is not None):
        raise RuntimeError("HARNESS_INVALID no telemetry within %.0f s" % timing["boot_wait_s"])
    loop(wall_s=timing["settle_s"])
    origin["o"] = (L.truth[1], L.truth[2], L.truth[3])
    trace["truth_origin"] = list(origin["o"])
    wanted = dict(param_values(table), **V0.PARAMS)
    types = dict({n: t for n, t, _, _, _ in table}, **{n: "REAL32" for n in V0.PARAMS})
    for k in wanted:
        val, typ = encode_param(types[k], wanted[k])
        L.send(L.m.mav.param_set_send, L.m.target_system, L.m.target_component, k.encode(), val, typ)
        sleep(timing["param_gap_s"])
        S.tick(clock())
    res["params_set"] = {k: [types[k], wanted[k]] for k in wanted}
    res["params_readback"] = L.read_params(list(wanted))
    res["param_problems"] = SF.params_problems(res["params_readback"], wanted)
    if res["param_problems"]:
        res["outcome"] = "PARAM_MISMATCH"                       # never fly with an unconfirmed failsafe set
        return
    x0n, y0e, z0 = L.pos[1], L.pos[2], L.pos[3]
    trace["home_ned"] = [x0n, y0e, z0]
    S.sp = (TM_POS_XYZ, x0n, y0e, -ALT_M, 0.0, 0.0, 0.0, 0.0)
    loop(wall_s=timing["pre_offboard_s"])
    L.cmd(MAV_CMD_DO_SET_MODE, 1, 6)
    event("offboard_requested")
    loop(wall_s=timing["pre_arm_s"])
    L.cmd(MAV_CMD_COMPONENT_ARM_DISARM, 1)
    event("arm_requested")
    loop(wall_s=timing["takeoff_s"])
    armed, mode = _autopilot_state(L)
    tx, ty, tz = _truth_local(L, origin["o"])
    res["start_check"] = {"armed": armed, "mode": mode, "ekf_alt_m": -L.pos[3], "truth_alt_m": tz,
                          "truth_from_home_xy_m": math.hypot(tx, ty)}
    if not (armed and mode == "OFFBOARD" and abs(-L.pos[3] - ALT_M) < 0.3 and 0.3 < tz < 2.5):
        raise RuntimeError("HARNESS_INVALID start state %s" % res["start_check"])
    S.track = True
    S.gaps()
    res["t_pre_start"] = sim_now(L)
    wall_pre = clock()
    event("pre_start")
    sim_phase(timing["pre_s"])
    res["stream_gaps"] = {"pre": S.gaps()}
    _inject(L, S, scen, res, event, sim_phase, timing)
    res["stream_gaps"]["post"] = S.gaps()
    res["streams_sent"] = dict(S.sent)
    res["real_time_factor"] = (sim_now(L) - res["t_pre_start"]) / max(1e-9, clock() - wall_pre)
    res["t_observation_end"] = sim_now(L)
    res["observation_counts"] = {key: len(getattr(L, attr)) for key, attr in
                                 (("hb", "hb_log"), ("statustext", "statustext"), ("acks", "acks"),
                                  ("rc_echo", "rc_echo"), ("man_echo", "man_echo"), ("landed", "landed"),
                                  ("rx_packets", "rx_packets"), ("tx_calls", "tx_calls"))}
    res["observation_counts"].update({k: len(trace[k]) for k in ("pos", "truth", "samples")})
    trace["tx_streams"] = list(S.tx_streams)
    event("observation_end")
    event("cleanup_start")
    armed, _ = _autopilot_state(L)
    if armed:                                                   # end disarmed, so the ULog closes (review r1 l5)
        L.cmd(MAV_CMD_DO_SET_MODE, 1, 4, 6)
        t_land = sim_now(L)
        loop(wall_s=timing["land_s"] / RTF_MIN, until=lambda: _first_hb(L, t_land, lambda a, m: not a) is not None)
    event("cleanup_end")
    res["ended_disarmed"] = not _autopilot_state(L)[0]
    res["outcome"] = "COMPLETED"


def _inject(L, S, scen, res, event, sim_phase, timing):
    inj = scen["inject"]
    t_inj = sim_now(L)
    res["t_inject"] = res["t_ref"] = t_inj
    hold_sp = S.sp
    if inj == "stop_setpoints":
        res["t_ref"] = S.last_sim["sp"]
        S.sp = None
    elif inj == "stop_rc":
        res["t_ref"] = S.last_sim["rc"]
        S.rc = None
    elif inj == "stop_gcs_heartbeat":
        res["t_ref"] = S.last_sim["hb"]
        S.hb = False
    elif inj == "fly_out":
        S.sp = (TM_POS_Z_VEL_XY_YAWRATE, 0.0, 0.0, hold_sp[3], GF_SPEED_MS, 0.0, 0.0, 0.0)
    elif inj == "climb":
        S.sp = (TM_VEL_XYZ_YAWRATE, 0.0, 0.0, 0.0, 0.0, 0.0, -GFV_SPEED_MS, 0.0)
    elif inj == "kill_switch":
        rc = list(S.rc)
        rc[scen["kill_channel"] - 1] = RC_KILL_US
        S.rc = rc
    event("inject", kind=inj)
    if inj == "none":
        sim_phase(scen["window_s"])
        return
    if inj == "gcs_disarm":
        L.cmd(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0)
        event("disarm_nonforced")
        sim_phase(scen["reject_window_s"])
        t_b = res["t_inject_b"] = sim_now(L)
        L.cmd(MAV_CMD_COMPONENT_ARM_DISARM, 0, FORCE_MAGIC)
        event("disarm_forced")
        reached = {"t": None}

        def done_b():
            if reached["t"] is None:
                reached["t"] = _first_hb(L, t_b, lambda armed, mode: not armed)
            return reached["t"] is not None and sim_now(L) >= reached["t"] + timing["post_hold_s"]
        sim_phase(scen["deadline_s"] + timing["post_hold_s"], done=done_b)
        return
    pulse = {"end": None}

    def pulses(now):
        if inj not in ("joystick_pulse", "rc_stick_pulse"):
            return
        if pulse["end"] is None:
            pulse["end"] = now + timing["stick_pulse_s"]
            if inj == "joystick_pulse":
                S.joy = JOY_PULSE
            else:
                rc = list(S.rc)
                rc[1] = RC_PITCH_PULSE_US
                S.rc = rc
        elif pulse["end"] is not False and now >= pulse["end"]:
            if inj == "joystick_pulse":
                S.joy = JOY_CENTER
            else:
                S.rc = list(RC_CENTER)
            pulse["end"] = False
            event("pulse_end")
    reached = {"t": None}

    def done():
        if reached["t"] is None:
            reached["t"] = _reached(scen, L, t_inj)
        return reached["t"] is not None and sim_now(L) >= reached["t"] + timing["post_hold_s"]
    sim_phase(scen["deadline_s"] + timing["post_hold_s"], done=done, on_tick=pulses)


# ------------------------------------------------------------------------------------------- assessment (pure)

def _obs(trace):
    return [(t, bool(base & ARMED_FLAG), mode_name(custom)) for t, _, base, custom in trace.get("hb", [])
            if t is not None]


def _nominal(o):
    return o[1] and o[2] == "OFFBOARD"


def _rc_echo_seen(trace, t0, t1, want=RC_CENTER, tol=2):
    return any(t is not None and t0 <= t < t1 and all(abs(a - b) <= tol for a, b in zip(ch, want))
               for t, *ch in trace.get("rc_echo", []))


def _path_echoes(trace, t0, t1, path):
    """PX4 MANUAL_CONTROL echoes of a centred selected setpoint in [t0, t1); on the RC path also the kill switch OFF,
    which only rc_update publishes (review r1 M1)."""
    n = 0
    for t, x, y, z, r, buttons in trace.get("man_echo", []):
        if t is None or not t0 <= t < t1 or max(abs(x), abs(y), abs(z), abs(r)) > ECHO_TOL:
            continue
        if path == "rc" and (buttons >> KILL_SHIFT) & 3 != SWITCH_OFF:
            continue
        n += 1
    return n


def _at_or_before(rows, t):
    best = None
    for r in rows:
        if r[0] <= t:
            best = r
        else:
            break
    return best


def assess(scen, res, trace, gap_limits=None):
    """valid (the harness did what the scenario needs) and, if valid, passed with the first failed criterion. Pure."""
    gap_limits = GAP_LIMIT_S if gap_limits is None else gap_limits
    out = {"valid": False, "invalid_reason": None, "passed": None, "fail_reason": None, "criteria": {}}
    if res.get("outcome") != "COMPLETED":
        out["invalid_reason"] = res.get("outcome") or "NO_OUTCOME"
        return out
    if res.get("param_problems"):
        out["invalid_reason"] = "PARAM_MISMATCH"
        return out
    if res.get("sim_too_slow"):
        out["invalid_reason"] = "SIM_TOO_SLOW"
        return out
    for phase, g in (res.get("stream_gaps") or {}).items():
        for k, v in g.items():
            if v > gap_limits[k]:
                out["invalid_reason"] = "STREAM_GAP %s %s %.3f s" % (phase, k, v)
                return out
    trace = bounded_rows(trace, res)
    obs = _obs(trace)
    t_pre, t_inj = res.get("t_pre_start"), res.get("t_inject")
    if t_pre is None or t_inj is None:
        out["invalid_reason"] = "NO_INJECTION_TIME"
        return out
    pre = [o for o in obs if t_pre <= o[0] < t_inj]
    out["criteria"]["pre_observations"] = len(pre)
    if len(pre) < MIN_PRE_OBS:
        out["invalid_reason"] = "TOO_FEW_OBSERVATIONS_BEFORE"
        return out
    if scen.get("needs_rc_echo") and not _rc_echo_seen(trace, t_pre, t_inj):
        out["invalid_reason"] = "RC_INPUT_NOT_SEEN"
        return out
    if scen.get("path"):
        n = out["criteria"]["manual_echo_pre"] = _path_echoes(trace, t_pre, t_inj, scen["path"])
        if n < MIN_ECHO:
            out["invalid_reason"] = "RC_PATH_NOT_SELECTED" if scen["path"] == "rc" else "JOYSTICK_PATH_NOT_SELECTED"
            return out
    if scen.get("needs_kill_on") and not any(t is not None and t >= t_inj and (b >> KILL_SHIFT) & 3 == SWITCH_ON
                                             for t, _, _, _, _, b in trace.get("man_echo", [])):
        out["invalid_reason"] = "KILL_SWITCH_NOT_SEEN"
        return out
    if scen["inject"] in ("joystick_pulse", "rc_stick_pulse"):
        if not any(t is not None and t_inj <= t and abs(x) >= 300 for t, x, *_ in trace.get("man_echo", [])):
            return _invalid(out, "STICK_PULSE_NOT_SEEN")
    if res.get("recording", {}).get("problems"):
        out["invalid_reason"] = "RECORDING_CONTRACT " + "; ".join(res["recording"]["problems"])
        return out
    flags = ulog_flag_rows(res.get("ulogs"))
    if not any(t_pre <= t <= t_inj for t, _ in flags):              # cause evidence needs the ULog flags (M2)
        out["invalid_reason"] = "ULOG_FLAGS_NOT_RECORDED"
        return out
    out["valid"] = True
    bad = [o for o in pre if not _nominal(o)]
    if bad:
        return _fail(out, "BEFORE_INJECTION %s armed=%s at %.2f s" % (bad[0][2], bad[0][1], bad[0][0]))
    live = live_flags(scen)
    raised = _raised(flags, live, t_pre, t_inj, closed_start=True)
    if raised:                                                   # the pre-window is a null for the flags too
        return _fail(out, "FLAG_BEFORE_INJECTION %s at %.2f s" % ("+".join(raised[1]), raised[0]))
    post = [o for o in obs if o[0] >= t_inj]
    exp = scen["expect"]
    if exp == "UNCHANGED":
        win = [o for o in post if o[0] <= t_inj + scen["window_s"]]
        out["criteria"]["window_observations"] = len(win)
        if len(win) < max(MIN_PRE_OBS, int(scen["window_s"] / 1.5)):
            return _invalid(out, "TOO_FEW_OBSERVATIONS_AFTER")
        bad = [o for o in win if not _nominal(o)]
        if bad:
            return _fail(out, "MODE_CHANGED %s armed=%s" % (bad[0][2], bad[0][1]))
        raised = _raised(flags, live, t_inj, t_inj + scen["window_s"])
        if raised:
            return _fail(out, "NULL_FLAG_RAISED %s at +%.2f s" % ("+".join(raised[1]), raised[0] - t_inj))
        return _pass(out)
    if exp == "FDS":
        return _assess_fds(out, scen, res, trace, obs, post, t_inj)
    t_ref = res.get("t_ref") if res.get("t_ref") is not None else t_inj
    return _expect_state(out, scen, post, t_inj, t_ref, exp, scen.get("allowed", ()), trace, res)


def _raised(flags, live, t0, t1, closed_start=False):
    """(t, [live flags true]) of the first failsafe_flags row in (t0, t1] ([t0, t1] if closed_start) with any live
    flag true, else None."""
    for t, f in flags:
        if (t0 <= t if closed_start else t0 < t) and t <= t1:
            on = [k for k in live if f[k]]
            if on:
                return t, on
    return None


def _cause(out, scen, res, trace, t_inj, dep_t):
    """Cause attribution (v2 review r1 M2). Returns a FAIL reason or None; records out["criteria"]["cause"].
    cause_flag scenarios: the first ULog failsafe_flags row after the injection with any live flag true must lie by the
    departure + CAUSE_LAG_S and show the scenario's own flag alone. cause_text scenarios: no live flag may be true
    from the injection to the cause STATUSTEXT + CAUSE_LAG_S (to the departure + CAUSE_LAG_S if no text; the missing
    text itself is INVALID EVENT_TEXT_NOT_SEEN after the other criteria)."""
    flags, live = ulog_flag_rows(res.get("ulogs")), live_flags(scen)
    own = scen.get("cause_flag")
    if own:
        first = _raised(flags, live, t_inj, dep_t + CAUSE_LAG_S)
        out["criteria"]["cause"] = {"evidence": "failsafe_flags", "flag": own,
                                    "first_raised": first[1] if first else None,
                                    "rise_after_inject_s": first[0] - t_inj if first else None}
        if first is None:
            return "CAUSE_NOT_SEEN %s by departure + %.1f s" % (own, CAUSE_LAG_S)
        if first[1] != [own]:
            return "WRONG_CAUSE %s (expected %s)" % ("+".join(first[1]), own)
        return None
    texts = scen.get("cause_text") or ()
    hits = [r[0] for r in trace.get("statustext", []) if r[0] is not None and t_inj <= r[0] <= dep_t + CAUSE_LAG_S
            and any(x in r[2] for x in texts)]
    t_c = min(hits) if hits else None
    other = _raised(flags, live, t_inj, (t_c if t_c is not None else dep_t) + CAUSE_LAG_S)
    out["criteria"]["cause"] = {"evidence": "statustext", "text_after_inject_s": t_c - t_inj if hits else None,
                                "other_raised": other[1] if other else None}
    if other:
        return "WRONG_CAUSE %s raised at +%.2f s" % ("+".join(other[1]), other[0] - t_inj)
    return None


def _assess_fds(out, scen, res, trace, obs, post, t_inj):
    t_b = res.get("t_inject_b")
    if t_b is None:
        return _invalid(out, "NO_SECOND_INJECTION")
    win = [o for o in post if o[0] < min(t_inj + scen["reject_window_s"], t_b)]
    out["criteria"]["reject_window_observations"] = len(win)
    if len(win) < MIN_AFTER_OBS:
        return _invalid(out, "TOO_FEW_OBSERVATIONS_AFTER")
    bad = [o for o in win if not _nominal(o)]
    if bad:
        return _fail(out, "NONFORCED_DISARM_ACTED %s armed=%s" % (bad[0][2], bad[0][1]))
    acks = [a for a in trace.get("acks", []) if a[0] is not None and a[1] == MAV_CMD_COMPONENT_ARM_DISARM]
    a1 = next((a for a in acks if t_inj <= a[0] < t_b), None)
    a2 = next((a for a in acks if a[0] >= t_b), None)
    out["criteria"]["acks"] = [a1, a2]
    if a1 is None or a2 is None:                                # the command did not reach the commander (m1)
        return _invalid(out, "NO_COMMAND_ACK")
    if a1[2] != MAV_RESULT_TEMPORARILY_REJECTED:
        return _fail(out, "NONFORCED_ACK result %d" % a1[2])
    if a2[2] != MAV_RESULT_ACCEPTED:
        return _fail(out, "FORCED_ACK result %d" % a2[2])
    # Cause (M2): the commands, answered as above; no live failsafe flag from the first command to the forced one
    # + CAUSE_LAG_S (the fall after the disarm is not attributed).
    other = _raised(ulog_flag_rows(res.get("ulogs")), live_flags(scen), t_inj, t_b + CAUSE_LAG_S)
    out["criteria"]["cause"] = {"evidence": "command_ack", "other_raised": other[1] if other else None}
    if other:
        return _fail(out, "WRONG_CAUSE %s raised at +%.2f s" % ("+".join(other[1]), other[0] - t_inj))
    return _expect_state(out, scen, [o for o in obs if o[0] >= t_b], t_b, t_b, "DISARMED", None, trace)


def _event_time(trace, scen, t_inj, dep_t):
    """The earliest of the first preregistered STATUSTEXT after the injection and the first non-nominal heartbeat:
    both lie at or after the true event, so the "not too early" checks cannot fail on resolution (review r1 m2)."""
    texts = scen.get("event_text") or ()
    hits = [r[0] for r in trace.get("statustext", []) if r[0] is not None and r[0] >= t_inj
            and any(x in r[2] for x in texts)]
    if hits and min(hits) <= dep_t:
        return min(hits), "statustext"
    return dep_t, "heartbeat"


def _expect_state(out, scen, post, t_inj, t_ref, exp, allowed, trace, res=None):
    disarm = exp == "DISARMED"
    hit = (lambda o: not o[1]) if disarm else (lambda o: o[1] and o[2] == exp)          # noqa: E731
    dep = next((o for o in post if not _nominal(o)), None)
    if dep is None:
        return _fail(out, "NO_DEPARTURE from OFFBOARD")
    ev, src = _event_time(trace, scen, t_inj, dep[0])
    out["criteria"].update(departure_mode=dep[2] if dep[1] else "DISARMED", event_source=src,
                           event_after_ref_s=ev - t_ref, departure_after_inject_s=dep[0] - t_inj)
    if scen.get("cause_flag") or scen.get("cause_text"):
        why = _cause(out, scen, res or {}, trace, t_inj, dep[0])
        if why:
            return _fail(out, why)
    if ev - t_ref < scen.get("earliest_s", 0.0) - 1e-9:
        return _fail(out, "TOO_EARLY %.2f s < %.2f s (%s)" % (ev - t_ref, scen["earliest_s"], src))
    reach = next((o for o in post if o[0] >= dep[0] and hit(o)), None)
    between = [o for o in post if dep[0] <= o[0] < (reach[0] if reach else float("inf"))]
    if allowed is not None:
        for o in between:
            if not o[1] or o[2] not in tuple(allowed) + (exp,):
                return _fail(out, "UNEXPECTED_STATE %s armed=%s at +%.2f s" % (o[2], o[1], o[0] - t_inj))
    if reach is None:
        return _fail(out, "NOT_REACHED %s" % exp)
    out["criteria"]["reached_after_inject_s"] = reach[0] - t_inj
    if reach[0] - t_inj > scen["deadline_s"] + 1e-9:
        return _fail(out, "LATE %.2f s > %.2f s" % (reach[0] - t_inj, scen["deadline_s"]))
    keep = [o for o in post if reach[0] <= o[0] <= reach[0] + PERSIST_S]
    if len(keep) < MIN_AFTER_OBS:
        return _invalid(out, "TOO_FEW_OBSERVATIONS_AFTER")
    for o in keep:
        if not (hit(o) or (exp == "AUTO_LAND" and not o[1])):
            return _fail(out, "NOT_PERSISTENT %s armed=%s" % (o[2], o[1]))
    home = trace.get("home_ned")
    pos = trace.get("pos", [])
    truth = [r for r in trace.get("truth", []) if r[0] >= t_inj]
    if "min_trigger_xy_m" in scen:
        p = _at_or_before(pos, ev)
        d = math.hypot(p[1] - home[0], p[2] - home[1]) if (p and home) else None
        out["criteria"]["trigger_xy_m"] = d
        if d is None or d < scen["min_trigger_xy_m"]:
            return _fail(out, "TOO_EARLY_DISTANCE %r < %.2f m (%s)" % (d, scen["min_trigger_xy_m"], src))
        m = max((math.hypot(r[1], r[2]) for r in truth), default=None)
        out["criteria"]["max_truth_xy_m"] = m
        if m is None or m > scen["max_truth_xy_m"]:
            return _fail(out, "LEFT_ARENA %r > %.1f m" % (m, scen["max_truth_xy_m"]))
    if "min_trigger_alt_m" in scen:
        p = _at_or_before(pos, ev)
        a = -(p[3] - home[2]) if (p and home) else None
        out["criteria"]["trigger_alt_m"] = a
        if a is None or a < scen["min_trigger_alt_m"]:
            return _fail(out, "TOO_EARLY_ALTITUDE %r < %.2f m (%s)" % (a, scen["min_trigger_alt_m"], src))
        m = max((r[3] for r in truth), default=None)
        out["criteria"]["max_truth_alt_m"] = m
        if m is None or m > scen["max_truth_alt_m"]:
            return _fail(out, "ABOVE_ARENA %r > %.1f m" % (m, scen["max_truth_alt_m"]))
    if "lockdown_within_s" in scen:
        r0 = _at_or_before(trace.get("truth", []), t_inj)
        win = [r for r in truth if r[0] <= t_inj + scen["lockdown_within_s"]]
        drop = (r0[3] - min(r[3] for r in win)) if (r0 and win) else None
        out["criteria"]["lockdown_drop_m"] = drop
        if reach[0] - t_inj > scen["lockdown_within_s"] and (drop is None or drop < scen["lockdown_drop_m"]):
            return _fail(out, "NO_LOCKDOWN drop %r m in %.1f s" % (drop, scen["lockdown_within_s"]))
    if scen.get("cause_text") and (out["criteria"].get("cause") or {}).get("text_after_inject_s") is None:
        return _invalid(out, "EVENT_TEXT_NOT_SEEN")          # cause not attributable: harness result, never PASS
    return _pass(out)


def _pass(out):
    out["passed"] = True
    return out


def _fail(out, why):
    out["passed"], out["fail_reason"] = False, why
    return out


def _invalid(out, why):
    out["valid"], out["passed"], out["invalid_reason"] = False, None, why
    return out


# ------------------------------------------------------------------------------------------- unit

def write_new(path, text):
    with open(path, "x") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())


def run_unit(sched, run_one, AT, ledger, busy_wait_s=None, host=None):
    """Idle check, same binding, kernel faults/tree, and abort at the first INVALID anywhere; no retry.
    A termination is kept in run_unit.terminated. Returns(status, reason, recorded)."""
    busy_wait_s = SF.HOST_BUSY_WAIT_S if busy_wait_s is None else busy_wait_s
    host = host or host_snapshot
    done, ref = 0, None
    try:
        faults0 = AT.kernel_faults()
        git0 = AT.git_state(ROOT)
        for i, item in enumerate(sched):
            idle_ok, idle = AT.idle_check()
            if not idle_ok and busy_wait_s > 0:
                time.sleep(busy_wait_s)
                idle_ok, idle = AT.idle_check()
            host_before = host()
            gpu = gpu_problems(host_before)                  # v2 review r1 m3: an idle host, not only idle cores
            if gpu:
                idle_ok, idle = False, dict(idle, gpu=gpu)
            t0 = time.time()
            if idle_ok:
                res = run_one(i, item)
                ok, why = bool(res.get("valid")), res.get("invalid_reason")
                core = SF.binding_core(res.get("runtime_binding"))
                if ok and core:
                    ref = ref or core
                    if core != ref:
                        ok, why = False, "BINDING_CHANGED"
            else:
                res, ok, why = {"outcome": "NOT_RUN"}, False, "HOST_BUSY"
            faults = AT.kernel_faults()
            rec = dict(item)
            rec.update(res)
            rec.update(index=i, wall_s=time.time() - t0, kernel_faults=faults, valid=ok, invalid_reason=why,
                       passed=res.get("passed") if ok else None, host_idle_before=dict(idle, ok=idle_ok),
                       host_before=host_before)
            SF.write_line(ledger, rec)
            done += 1
            print(json.dumps({k: rec.get(k) for k in ("index", "scenario", "outcome", "valid", "invalid_reason",
                                                       "passed", "fail_reason", "wall_s")}), flush=True)
            if faults != faults0:
                return "ABORTED", "kernel fault count rose %d -> %d" % (faults0, faults), done
            if AT.git_state(ROOT) != git0:
                return "ABORTED", "the tracked tree changed during scenario %d" % i, done
            if not ok:
                return "ABORTED", "first INVALID scenario %d (%s): the harness does not work" % (i, why), done
        return "COMPLETE", None, done
    except BaseException as e:                                       # noqa: BLE001 - recorded, then re-raised
        if not isinstance(e, Exception):
            run_unit.terminated = e
        return "ABORTED", "%s: %s" % (e.__class__.__name__, e), done


run_unit.terminated = None


def launch_problems(AT, gs, approval_path, attempt_dir, sources, env_report=None, environ=None, scope=SCOPE,
                    host=None, disk_free=None):
    """Every reason to refuse the launch. host and disk_free are injectable for the tests (host_snapshot and the free
    bytes on attempt_dir's filesystem by default)."""
    env_report = env_report or SF.environment_report
    host = host or host_snapshot
    disk_free = disk_free or free_bytes
    p = []
    if not pathlib.Path(approval_path).is_file():
        p.append("no approval file %s" % approval_path)
    else:
        try:
            p += SF.approval_problems(json.loads(pathlib.Path(approval_path).read_text()), gs["head"], scope=scope)
        except ValueError as e:
            p.append("approval file unreadable: %s" % e)
    if gs["dirty"]:
        p.append("tracked tree is not clean")
    p += AT.source_problems(ROOT, sources)
    p += table_problems()
    p += env_report()[1]
    p += SF.env_problems(environ)
    idle_ok, idle = AT.idle_check()
    if not idle_ok:
        p.append("runner cores not idle: %s" % idle)
    p += ["host not idle: %s" % x for x in gpu_problems(host())]
    try:
        free = disk_free(attempt_dir)
        if free < MIN_FREE_BYTES:
            p.append("free disk %.2f GiB < %.0f GiB on the unit directory's filesystem"
                     % (free / 1024 ** 3, MIN_FREE_BYTES / 1024 ** 3))
    except OSError as e:
        p.append("free disk unreadable: %s" % e)
    if pathlib.Path(attempt_dir).exists():
        p.append("attempt directory exists (single use): %s" % attempt_dir)
    return p


def smoke_problems(smoke_dir, head, sources):
    """attempt01 needs smoke01 COMPLETE at this HEAD and these source bytes, with N0, N0R and N0J all VALID and
    nominal (the RC and joystick paths seen before any injection depends on them; review r1 M1)."""
    f, e, a = (pathlib.Path(smoke_dir) / n for n in ("final.json", "episodes.jsonl", "attempt.json"))
    if not (f.is_file() and e.is_file() and a.is_file()):
        return ["no development smoke record in %s" % smoke_dir]
    att = json.loads(a.read_text())
    if (att.get("git") or {}).get("head") != head:
        return ["development smoke ran at %s, not HEAD %s" % ((att.get("git") or {}).get("head"), head)]
    if att.get("sources") != sources:
        return ["development smoke ran other source bytes"]
    rows, final = SF.read_jsonl(e), json.loads(f.read_text())
    if final.get("status") != "COMPLETE" or [r.get("scenario") for r in rows] != list(SMOKE_SCHEDULE) \
            or not all(r.get("valid") and r.get("passed") for r in rows):
        return ["development smoke not COMPLETE with %s VALID and nominal: %s" % (
            list(SMOKE_SCHEDULE), [(r.get("scenario"), r.get("invalid_reason"), r.get("fail_reason")) for r in rows])]
    return integrity_problems(att, final, rows, smoke_dir, allow_smoke=True)


def write_final(attempt_dir, AT, status, reason, done, sched, faults0):
    try:
        faults1 = AT.kernel_faults()
    except Exception as e:                                   # noqa: BLE001
        faults1 = "unreadable: %s" % e
    try:
        git_after = AT.git_state(ROOT)
    except Exception as e:                                   # noqa: BLE001
        git_after = {"error": str(e)}
    final = {"status": status, "reason": reason, "scenarios_recorded": done,
             "scenarios_scheduled": len(sched) if sched is not None else None, "kernel_faults_before": faults0,
             "kernel_faults_after": faults1, "finished": time.time(), "git_after": git_after}
    write_new(pathlib.Path(attempt_dir) / "final.json", json.dumps(final, indent=1, default=str) + "\n")
    print(json.dumps(final, default=str), flush=True)
    return final


def launch(smoke=False):
    signal.signal(signal.SIGTERM, SF._on_sigterm)
    import sitl_dev08_attempt as AT
    gs = AT.git_state(ROOT)
    out_dir = SMOKE_DIR if smoke else ATTEMPT_DIR
    sources = sorted(OWN_SOURCES)
    src_sha = {s: SF.sha256_file(ROOT / s) for s in sources if (ROOT / s).is_file()}
    problems = launch_problems(AT, gs, UNIT_DIR / "execution_approval.json", out_dir, sources)
    if not smoke:
        problems += smoke_problems(SMOKE_DIR, gs["head"], src_sha)
    faults0 = None
    if not problems:
        try:
            faults0 = AT.kernel_faults()
        except Exception as e:                               # noqa: BLE001
            problems.append("kernel fault baseline unreadable: %s" % e)
    if problems:
        raise SystemExit("refused:\n  " + "\n  ".join(problems))
    out_dir.mkdir(parents=True)
    sched, done, terminated = None, 0, None
    try:
        sched = smoke_schedule() if smoke else schedule()
        import sitl_owned_session as OS
        import sitl_s3b_closed_loop as V0
        rec = {"protocol": PROTOCOL, "prereg": PREREG, "scope": SCOPE, "development_smoke": bool(smoke), "git": gs,
               "approval_sha256": SF.sha256_file(UNIT_DIR / "execution_approval.json"), "sources": src_sha,
               "parameter_set": [list(r) for r in all_params()], "vehicle_params": V0.PARAMS,
               "scenarios": [dict(s) for s in scenarios()], "timing": TIMING, "persist_s": PERSIST_S,
               "data_contract": {"rx_gap_s": {"LOCAL_POSITION_NED": .10, "HIL_STATE_QUATERNION": .10,
                    "ATTITUDE": .10, "HEARTBEAT": 1.6, "RC_CHANNELS": .20, "MANUAL_CONTROL": .25},
                    "sample_age_s": .10, "sample_age_clock": "wall (monotonic receive time)",
                    "tx_gap_s": GAP_LIMIT_S, "ulog_position_gap_s": ULOG_POSITION_GAP_S,
                    "ulog_position_boundary_tolerance_s": ULOG_BOUNDARY_TOL_S,
                    "ulog_coverage_end": "min(observation end, first disarmed autopilot heartbeat)",
                    "ulog_flags_gap_s": FLAG_GAP_S, "cause_flags": list(CAUSE_FLAGS), "cause_lag_s": CAUSE_LAG_S,
                    "min_free_bytes": MIN_FREE_BYTES, "gpu_allowed_apps": list(GPU_ALLOWED_APPS),
                    "contacts": "NOT_RECORDED"},
               "environment": SF.environment_report()[0], "environment_keys": sorted(os.environ),
               "idle": AT.idle_check()[1], "host": host_snapshot(), "schedule": sched,
               "kernel_faults_before": faults0, "started": time.time()}
        write_new(out_dir / "attempt.json", json.dumps(rec, indent=1, default=str) + "\n")
        by_id = {s["id"]: s for s in scenarios()}
        run_unit.terminated = None
        status, reason, done = run_unit(
            sched, lambda i, item: run_scenario(OS, V0, by_id[item["scenario"]], out_dir / ("ep%03d" % i)), AT,
            out_dir / "episodes.jsonl")
        terminated = run_unit.terminated
    except BaseException as e:                               # noqa: BLE001 - recorded, then re-raised
        status, reason = "ABORTED", "%s: %s" % (e.__class__.__name__, e)
        if not isinstance(e, Exception):
            terminated = e
    write_final(out_dir, AT, status, reason, done, sched, faults0)
    if terminated is not None:
        raise terminated


# ------------------------------------------------------------------------------------------- analysis

def integrity_problems(attempt, final, rows, attempt_dir=None, allow_smoke=False):
    if attempt is None:
        return ["no attempt record"]
    p = []
    sched = attempt.get("schedule", [])
    if [r.get("scenario") for r in rows] != [s["scenario"] for s in sched[:len(rows)]]:
        p.append("ledger rows are not the schedule's prefix in order")
    if final.get("status") == "COMPLETE" and len(rows) != len(sched):
        p.append("COMPLETE with %d of %d scheduled scenarios" % (len(rows), len(sched)))
    if final.get("kernel_faults_after") != attempt.get("kernel_faults_before"):
        p.append("kernel faults %r -> %r" % (attempt.get("kernel_faults_before"), final.get("kernel_faults_after")))
    ga, g0 = final.get("git_after") or {}, attempt.get("git") or {}
    if ga.get("head") != g0.get("head") or ga.get("dirty"):
        p.append("tree changed during the run")
    if attempt.get("development_smoke") and not allow_smoke:
        p.append("a development smoke has no reading")
    if attempt_dir is not None:
        expected = smoke_schedule() if allow_smoke else schedule()
        if sched != expected:
            p.append("attempt schedule differs from registered schedule")
        if attempt.get("protocol") != PROTOCOL or attempt.get("scope") != SCOPE or attempt.get("prereg") != PREREG:
            p.append("attempt protocol/scope/prereg binding differs")
        by_id = {s["id"]: s for s in attempt.get("scenarios", [])}
        for r in rows:
            if r.get("outcome") == "NOT_RUN":
                continue
            f = pathlib.Path(attempt_dir) / ("ep%03d" % r["index"]) / "trace.json"
            if not f.is_file() or SF.sha256_file(f) != r.get("trace_sha256"):
                p.append("trace of scenario %s missing or changed" % r.get("scenario"))
                continue
            trace = json.loads(f.read_text())
            ep_dir = f.parent
            receipt = ep_dir / "session.json"
            if not receipt.is_file() or SF.sha256_file(receipt) != r.get("session_receipt_sha256"):
                p.append("session receipt of scenario %s missing or changed" % r.get("scenario"))
            for binding in (r.get("runtime_binding") or {}).get("files", {}).values():
                bf = pathlib.Path(binding.get("path", ""))
                if not bf.is_file() or SF.sha256_file(bf) != binding.get("sha256"):
                    p.append("runtime file of scenario %s missing or changed" % r.get("scenario"))
            for u in r.get("ulogs", []):
                uf = pathlib.Path(u["path"])
                if not uf.is_file() or uf.stat().st_size != u["bytes"] or SF.sha256_file(uf) != u["sha256"]:
                    p.append("ULog of scenario %s missing or changed" % r.get("scenario"))
            recording = recording_contract(by_id[r["scenario"]], r, trace)
            recording["problems"] += ulog_problems(r.get("ulogs", []), by_id[r["scenario"]], r, trace)
            if recording != r.get("recording"):
                p.append("recording contract of scenario %s differs" % r.get("scenario"))
            again = assess(by_id[r["scenario"]], r, trace)
            if any(again[k] != r.get(k) for k in ("valid", "passed", "fail_reason", "invalid_reason", "criteria")) \
                    and r.get("invalid_reason") != "BINDING_CHANGED":
                p.append("scenario %s: recomputed assessment differs from the ledger" % r.get("scenario"))
    return p


def reading(final, rows, attempt, attempt_dir=None):
    """VOID (integrity) > FAIL_SITL (any VALID scenario failed) > INCOMPLETE (not COMPLETE, or any INVALID / NOT_RUN)
    > PASS_SITL. SITL on the stand-in vehicle only."""
    integ = integrity_problems(attempt, final, rows, attempt_dir)
    per = {r["scenario"]: {k: r.get(k) for k in ("valid", "invalid_reason", "passed", "fail_reason", "criteria")}
           for r in rows}
    failed = sorted(s for s, r in per.items() if r["valid"] and r["passed"] is False)
    invalid = sorted(s for s, r in per.items() if not r["valid"])
    n_sched = len((attempt or {}).get("schedule", []))
    if integ:
        label, why = "VOID", "; ".join(integ)
    elif failed:
        label, why = "FAIL_SITL", "failed: %s" % ", ".join(failed)
    elif final.get("status") != "COMPLETE" or invalid or len(rows) != n_sched:
        label, why = "INCOMPLETE", "status %s, invalid %s" % (final.get("status"), invalid)
    else:
        label, why = "PASS_SITL", None
    return {"protocol": PROTOCOL, "reading": label, "reason": why, "scenarios": per, "integrity_problems": integ,
            "attempt_status": final.get("status"), "attempt_reason": final.get("reason"),
            "scope": "PX4 v1.14.3 SITL, navrl_quad stand-in; no claim about the real flight controller, RC or vehicle"}


def analyze(attempt_dir=ATTEMPT_DIR):
    attempt_dir = pathlib.Path(attempt_dir)
    load = lambda name, default: (json.loads((attempt_dir / name).read_text())          # noqa: E731
                                  if (attempt_dir / name).is_file() else default)
    attempt = load("attempt.json", None)
    if (attempt or {}).get("development_smoke"):
        raise SystemExit("%s is a development smoke: it has no reading" % attempt_dir)
    final = load("final.json", {"status": "NO_FINAL_RECORD", "reason": "final.json missing"})
    rows = SF.read_jsonl(attempt_dir / "episodes.jsonl") if (attempt_dir / "episodes.jsonl").is_file() else []
    out = reading(final, rows, attempt, attempt_dir)
    write_new(attempt_dir / "reading.json", json.dumps(out, indent=1, sort_keys=True, default=str) + "\n")
    print(json.dumps({k: out[k] for k in ("reading", "reason")}), flush=True)
    return out


def check():
    out = {"protocol": PROTOCOL, "scope": SCOPE, "prereg": PREREG, "table_problems": table_problems(),
           "parameter_set": [list(r) for r in all_params()], "user_decisions": list(USER_DECISIONS),
           "untested_real_vehicle_params": [list(r) for r in UNTESTED_REAL_VEHICLE_PARAMS],
           "scenarios": [dict(s) for s in scenarios()], "schedule": SCHEDULE, "smoke_schedule": SMOKE_SCHEDULE,
           "environment_problems": SF.environment_report()[1], "env_problems": SF.env_problems()}
    print(json.dumps(out, indent=1, default=str))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--smoke", action="store_true")
    g.add_argument("--launch", action="store_true")
    g.add_argument("--analyze", nargs="?", const=str(ATTEMPT_DIR))
    a = ap.parse_args(argv)
    if a.check:
        check()
    elif a.smoke:
        launch(smoke=True)
    elif a.launch:
        launch()
    else:
        analyze(a.analyze)


if __name__ == "__main__":
    main()
