#!/usr/bin/env python3
"""Compare the flight controller's live parameters with the lab's (read-only).

    python3 scripts/params_check.py [--device /dev/serial/by-id/<id>[:baud]] [--json]
    python3 scripts/host_setup.py --params-check [--device ...] [--json]     # same thing

Reads each parameter named in config/lab.params from the flight controller over its USB port
(MAVLink PARAM_REQUEST_READ, via pymavlink) and reports where the live value differs. It never
writes a parameter, changes a mode or arms: to fix a difference, set the value in QGroundControl.

Rows ({name,status,detail,fix,why}; PASS / WARN / INFO):

  params-vs-lab    summary: PASS when every readable parameter matches (allowed differences
                   aside), WARN with the count otherwise. INFO when the port cannot be read.
  param:<NAME>     one WARN per differing parameter (live vs lab), the QGC fix as text.
  allowed:<NAME>   INFO for a difference that is expected (ALLOWED_DIFFS below).

ALLOWED_DIFFS (fnmatch patterns) are parameters that legitimately differ between set-ups: GPS,
barometer and the height reference (EKF2_GPS_CTRL, EKF2_GPS_CHECK, EKF2_BARO_CTRL,
EKF2_HGT_REF and their relatives). They are reported as INFO, never as WARN.

Opening the port needs it free (not held by MAVROS); the check says so when it is busy.
Python stdlib only (pymavlink is imported when installed: scripts/deps/px4.sh).
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

PASS, WARN, INFO = "PASS", "WARN", "INFO"
USB_DEVICE = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"
PYMAVLINK_DIR = "/opt/pymavlink"
LAB_PARAMS = Path(__file__).resolve().parent.parent / "config" / "lab.params"

# Differences that are expected between set-ups (indoor flow vs outdoor GPS).
ALLOWED_DIFFS = (
    "EKF2_GPS_*",        # EKF2_GPS_CTRL, EKF2_GPS_CHECK, ...
    "EKF2_BARO_*",       # EKF2_BARO_CTRL, ...
    "EKF2_HGT_REF",
)

# MAVLink MAV_PARAM_TYPE values that carry an integer (PX4 sends the integer's bytes
# in the float field of PARAM_VALUE).
_INT_TYPES = {1: "<B", 2: "<b", 3: "<H", 4: "<h", 5: "<I", 6: "<i", 7: "<Q", 8: "<q"}
_FLOAT_TYPE = 9
TOL = 1e-4


def chk(name, status, detail, fix="", why=""):
    return {"name": name, "status": status, "detail": detail, "fix": fix, "why": why}


def load_lab(path=LAB_PARAMS):
    """{name: float} from a QGroundControl .params file (tab separated; '#' comments)."""
    out = {}
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        f = line.split()
        if len(f) >= 4:
            out[f[2]] = float(f[3])
    return out


def is_allowed(name, allowed=ALLOWED_DIFFS):
    return any(fnmatch.fnmatchcase(name, pat) for pat in allowed)


def same(live, lab):
    return math.isclose(live, lab, rel_tol=TOL, abs_tol=1e-6)


def decode_value(param_value, param_type):
    """PARAM_VALUE.param_value (a float field) -> the parameter's number.

    PX4 packs integer parameters bytewise into the float field: re-read the 4 bytes as the
    integer type. Floats are used as they are.
    """
    t = int(param_type)
    if t in _INT_TYPES and struct.calcsize(_INT_TYPES[t]) <= 4:
        raw = struct.pack("<f", float(param_value))
        return float(struct.unpack(_INT_TYPES[t], raw[:struct.calcsize(_INT_TYPES[t])])[0])
    return float(param_value)


def compare(lab, live, allowed=ALLOWED_DIFFS):
    """-> (diffs, allowed_diffs, unreadable): lists of (name, live, lab) / names."""
    diffs, allowed_diffs, unreadable = [], [], []
    for name, want in lab.items():
        got = live.get(name)
        if got is None:
            unreadable.append(name)
        elif not same(got, want):
            (allowed_diffs if is_allowed(name, allowed) else diffs).append((name, got, want))
    return diffs, allowed_diffs, unreadable


def _fmt(v):
    return str(int(v)) if float(v).is_integer() else f"{v:.6g}"


def rows(lab, live, allowed=ALLOWED_DIFFS):
    """The JSON rows for a comparison (pure)."""
    diffs, allowed_diffs, unreadable = compare(lab, live, allowed)
    read = len(lab) - len(unreadable)
    out = []
    if not read:
        return [chk("params-vs-lab", INFO, "no parameter could be read from the flight controller",
                    "check the USB cable and that nothing else (MAVROS) holds the port")]
    if diffs:
        out.append(chk("params-vs-lab", WARN,
                       f"{len(diffs)} of {read} parameters differ from config/lab.params"
                       + (f" ({len(allowed_diffs)} allowed)" if allowed_diffs else "")
                       + (f"; {len(unreadable)} unreadable" if unreadable else ""),
                       "set the listed parameters in QGroundControl (this check never writes them)",
                       "the lab parameters (EKF2 optical flow / range fusion, UAVCAN, battery) are what "
                       "the indoor OFFBOARD flight was set up with"))
    else:
        out.append(chk("params-vs-lab", PASS,
                       f"all {read} readable parameters match config/lab.params"
                       + (f" ({len(allowed_diffs)} allowed differences)" if allowed_diffs else "")
                       + (f"; {len(unreadable)} unreadable" if unreadable else "")))
    for name, got, want in diffs:
        out.append(chk(f"param:{name}", WARN, f"live {_fmt(got)}, lab {_fmt(want)}",
                       f"in QGroundControl set {name} to {_fmt(want)}",
                       "differs from the lab parameters"))
    for name, got, want in allowed_diffs:
        out.append(chk(f"allowed:{name}", INFO, f"live {_fmt(got)}, lab {_fmt(want)} (allowed difference)",
                       "", "GPS / barometer / height reference differ between indoor and outdoor set-ups"))
    return out


def read_params(device, names, timeout=1.0, total_timeout=120.0, connect=None):
    """Read `names` over MAVLink -> ({name: value}, error or ''). Read-only: PARAM_REQUEST_READ."""
    for extra in (os.environ.get("PYMAVLINK_PATH", ""), PYMAVLINK_DIR):
        if extra and os.path.isdir(extra) and extra not in sys.path:
            sys.path.append(extra)
    if connect is None:
        try:
            from pymavlink import mavutil
        except ImportError:
            return {}, "pymavlink not installed (sudo bash scripts/deps/px4.sh)"
        connect = mavutil.mavlink_connection
    path, baud = (device.rsplit(":", 1) + ["115200"])[:2] if ":" in device else (device, "115200")
    try:
        m = connect(path, baud=int(baud))
    except Exception as e:  # noqa: BLE001
        return {}, f"cannot open {path}: {e} (is another process holding the port?)"
    live = {}
    try:
        if m.wait_heartbeat(timeout=5) is None:
            return {}, "no heartbeat from the flight controller"
        t0 = time.monotonic()
        for name in names:
            if time.monotonic() - t0 > total_timeout:
                break
            m.mav.param_request_read_send(m.target_system, m.target_component,
                                          name.encode(), -1)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=max(0.05, end - time.monotonic()))
                if msg is None:
                    break
                pid = msg.param_id.decode() if isinstance(msg.param_id, bytes) else str(msg.param_id)
                if pid.rstrip("\x00") == name:
                    live[name] = decode_value(msg.param_value, msg.param_type)
                    break
    except Exception as e:  # noqa: BLE001
        return live, f"read failed: {e}"
    finally:
        try:
            m.close()
        except Exception:  # noqa: BLE001
            pass
    return live, ""


def run(device=USB_DEVICE, lab_path=LAB_PARAMS, reader=read_params):
    try:
        lab = load_lab(lab_path)
    except OSError as e:
        return [chk("params-vs-lab", INFO, f"cannot read {lab_path}: {e}")]
    live, err = reader(device, list(lab))
    if err and not live:
        return [chk("params-vs-lab", INFO, err, "connect the flight controller's USB cable; stop the MAVROS driver")]
    out = rows(lab, live)
    if err:
        out.insert(1, chk("params-read", INFO, err))
    return out


def report(rows_, as_json):
    if as_json:
        json.dump(rows_, sys.stdout, indent=2)
        print()
        return
    for c in rows_:
        print(f"[{c['status']:<4}] {c['name']:<28} {c['detail']}")
        if c["status"] == WARN and c["fix"]:
            print(f"         fix: {c['fix']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--device", default=USB_DEVICE, help="USB device or path:baud; default %(default)s")
    ap.add_argument("--lab", default=str(LAB_PARAMS), help="the lab .params file; default %(default)s")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    rows_ = run(a.device, a.lab)
    report(rows_, a.json)
    return 0   # differences are warnings, not failures


if __name__ == "__main__":
    sys.exit(main())
