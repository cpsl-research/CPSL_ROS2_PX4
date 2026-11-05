#!/usr/bin/env python3
"""Pinned PX4 firmware for the Pixhawk 6X: check, back up params, build (command), flash (guarded).

    python3 scripts/firmware.py --check [--json] [--no-probe]        # read-only
    python3 scripts/firmware.py --backup-params FILE [--diff LAB]    # read-only; starts its own temporary MAVROS
    python3 scripts/firmware.py --diff-params FILE LAB               # compare two QGC param files
    python3 scripts/firmware.py --build [--out DIR] [--json]         # PRINT the build commands
    python3 scripts/firmware.py --record [--out DIR]                 # record the built artifact's sha256
    python3 scripts/firmware.py --flash --dry-run [...]              # evaluate every guard, flash nothing
    python3 scripts/firmware.py --flash --apply --confirm-serial S --backup FILE [...]

Everything is pinned in ``firmware/versions.json``: PX4-Autopilot tag/commit (the submodule
``firmware/PX4-Autopilot``), the px4_msgs release it pairs with, the board target and its USB
identity. This tool never writes parameters (re-applying a param file is done in QGC), never
runs docker (``--build`` prints the commands), and ``--flash --apply`` is the only mode that
touches the board, through the pinned ``Tools/px_uploader.py``.

``--check`` rows are {name,status,detail,fix,why} (PASS/FAIL/WARN/INFO; exit 1 iff any FAIL):
board, pins, current_version, artifact, uploader, modemmanager.

``--flash`` refuses (every guard is a row; exit 1) unless ALL hold:
  identity        exactly one board, VID:PID 3185:0035, in APP mode (the bootloader enumerates with
                  the same VID:PID and is told apart only by its product string "PX4 BL FMU v6X.x")
  serial_confirm  --confirm-serial equals that board's USB serial
  backup          --backup FILE exists, was made by --backup-params this session (<= --backup-max-age
                  seconds ago, same board, unchanged since) and holds parameters
  driver_stopped  no MicroXRCEAgent / mavros_node / px4_control_node / QGroundControl running
  artifact        --artifact (default: the --out build) matches the sha256 recorded by --record,
                  names board_id 53, and was built from the pinned commit
  port_free       fuser/lsof finds no process holding the board's port
  uploader        Tools/px_uploader.py is in the submodule and pyserial is importable
ModemManager is reported as a WARN row (not a refusal): see host_setup.py --usb.

Python stdlib only (pymavlink is used for the version probe when installed).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PROC = "/proc"  # tests point this at a fake tree

ROOT = Path(__file__).resolve().parent.parent
PAIRING = ROOT / "firmware" / "versions.json"
BY_ID = "/dev/serial/by-id"
PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"
DRIVERS = ("MicroXRCEAgent", "mavros_node", "px4_control_node", "QGroundControl")
META = ".meta.json"        # sidecar of a param backup
RECORD = ".record.json"    # sidecar of a built artifact


def chk(name, status, detail, fix="", why=""):
    return {"name": name, "status": status, "detail": detail, "fix": fix, "why": why}


# ---------------------------------------------------------------------------
# pairing file
# ---------------------------------------------------------------------------

def load_pairing(path=PAIRING):
    """versions.json -> dict; raises ValueError naming what is wrong."""
    try:
        d = json.loads(Path(path).read_text())
    except (OSError, ValueError) as e:
        raise ValueError(f"cannot read {path}: {e}")
    try:
        for k in ("firmware", "px4_msgs", "board"):
            d[k]
        for k in ("tag", "commit", "path"):
            d["firmware"][k]
        d["px4_msgs"]["commit"]
        b = d["board"]
        for k in ("target", "board_id", "app_product", "bootloader_product"):
            b[k]
        b["usb"]["vid"], b["usb"]["pid"]
        d["toolchain"]["image"]
        d.setdefault("submodules", [])
    except (KeyError, TypeError) as e:
        raise ValueError(f"{path} is missing {e}")
    return d


def tag_tuple(tag):
    m = re.fullmatch(r"v?(\d+)\.(\d+)\.(\d+)", tag or "")
    return tuple(int(x) for x in m.groups()) if m else None


def artifact_path(pairing, out):
    t = pairing["board"]["target"]
    return Path(out) / t / f"{t}.px4"


# ---------------------------------------------------------------------------
# host access (faked in tests)
# ---------------------------------------------------------------------------

# scripts/deps/px4.sh installs pymavlink here (no apt package exists on Ubuntu 24.04 / Jazzy).
PYMAVLINK_DIR = "/opt/pymavlink"


class Host:
    def run(self, cmd, timeout=5):
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return p.returncode, p.stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            return None, ""

    def by_id(self):
        try:
            return [(n, os.path.join(BY_ID, n)) for n in sorted(os.listdir(BY_ID))]
        except OSError:
            return []

    def udev(self, path):
        """USB identity (ID_VENDOR_ID, ID_MODEL_ID, ID_MODEL, ID_SERIAL_SHORT): sysfs first
        (works in a container with no udevadm), udevadm as an optional extra."""
        props = self.sysfs(path)
        rc, out = self.run(["udevadm", "info", "-q", "property", "-n", path])
        if rc == 0:
            props = {**dict(l.split("=", 1) for l in out.splitlines() if "=" in l), **props}
        return props or None

    def sysfs(self, path):
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            import host_setup
            return host_setup.sysfs_usb_props(path)
        except Exception:  # noqa: BLE001 - identity then falls back to udevadm only
            return {}

    def ps(self):
        return self.run(["ps", "-eo", "pid=,args="])[1]

    def port_busy(self, path):
        """(True|False|None, who). None: cannot tell (neither fuser nor lsof)."""
        real = os.path.realpath(path)
        if shutil.which("fuser"):
            rc, out = self.run(["fuser", real])
            if rc == 0:
                return True, out
            if rc == 1:
                return False, ""
        if shutil.which("lsof"):
            rc, out = self.run(["lsof", "-t", real])
            if rc == 0 and out:
                return True, out
            if rc in (0, 1):
                return False, ""
        return self.proc_busy(path)

    def proc_busy(self, path):
        """No-binary fallback: scan PROC/<pid>/fd symlinks for the device node (and the
        by-id symlink's target). (None, why) if /proc cannot be read at all."""
        targets = {os.path.realpath(path), str(path)}
        try:
            pids = [n for n in os.listdir(PROC) if n.isdigit() and int(n) != os.getpid()]
        except OSError:
            return None, "neither fuser nor lsof, and /proc is unreadable"
        holders, readable = [], 0
        for pid in pids:
            fd_dir = os.path.join(PROC, pid, "fd")
            try:
                names = os.listdir(fd_dir)
            except OSError:  # exited, or another user's process
                continue
            readable += 1
            for n in names:
                try:
                    if os.readlink(os.path.join(fd_dir, n)) in targets:
                        holders.append(pid)
                        break
                except OSError:
                    continue
        if holders:
            return True, " ".join(holders)
        if pids and not readable:
            return None, "neither fuser nor lsof, and no /proc/<pid>/fd is readable"
        return False, ""

    # -- temporary MAVROS for --backup-params (all faked in tests) ----------------
    def ros_env(self, domain):
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(domain)
        env["ROS_LOCALHOST_ONLY"] = "1"
        return env

    def ros(self, cmd, env, timeout):
        """Run a ros2 CLI command -> (returncode|None, stdout+stderr)."""
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
            return p.returncode, (p.stdout or "") + (p.stderr or "")
        except (OSError, subprocess.TimeoutExpired) as e:
            return None, f"{type(e).__name__}: {e}"

    def spawn(self, cmd, env, logfile):
        return subprocess.Popen(cmd, env=env, stdin=subprocess.DEVNULL, stdout=open(logfile, "wb"),
                                stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self, proc, grace=10.0):
        """SIGINT the process group, then SIGKILL after ``grace`` s. Returns True once it is gone."""
        if proc is None:
            return True
        for sig, wait in ((signal.SIGINT, grace), (signal.SIGKILL, 5.0)):
            if proc.poll() is not None:
                break
            try:
                os.killpg(proc.pid, sig)
            except (OSError, ProcessLookupError):
                pass
            try:
                proc.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                pass
        return proc.poll() is not None

    def wait_port_free(self, path, timeout=10.0):
        end = time.monotonic() + timeout
        while True:
            busy, who = self.port_busy(path)
            if busy is False:
                return True, ""
            if time.monotonic() >= end:
                return False, who
            time.sleep(0.5)

    def sleep(self, s):
        time.sleep(s)

    def have_pyserial(self):
        try:
            import serial  # noqa: F401
            return True
        except ImportError:
            return False

    def git_head(self, repo):
        if not (Path(repo) / ".git").exists():     # else git would answer for the superproject
            return None
        rc, out = self.run(["git", "-C", str(repo), "rev-parse", "HEAD"])
        return out if rc == 0 and out else None

    def modemmanager_row(self, device):
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            import host_setup
            rows = host_setup.run_usb_checks(argparse.Namespace(device=device))
            return next(r for r in rows if r["name"] == "modemmanager")
        except Exception as e:  # noqa: BLE001 - a missing/odd helper must not break --check
            return chk("modemmanager", INFO, f"cannot run host_setup.py --usb checks ({e})")

    def probe_version(self, path, timeout=5.0):
        """Read-only MAVLink query -> (sw_version int, custom 8 bytes) or (None, reason)."""
        for extra in (os.environ.get("PYMAVLINK_PATH", ""), PYMAVLINK_DIR):
            if extra and os.path.isdir(extra) and extra not in sys.path:
                sys.path.append(extra)
        try:
            from pymavlink import mavutil
        except ImportError:
            return None, "pymavlink not installed (sudo bash scripts/deps/px4.sh)"
        try:
            m = mavutil.mavlink_connection(path, baud=115200)
            if m.wait_heartbeat(timeout=timeout) is None:
                return None, "no heartbeat"
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                    mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
            msg = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=timeout)
            m.close()
            if msg is None:
                return None, "no AUTOPILOT_VERSION reply"
            return msg.flight_sw_version, bytes(msg.flight_custom_version[:8])
        except Exception as e:  # noqa: BLE001
            return None, f"probe failed: {e}"


# ---------------------------------------------------------------------------
# board identification (pure given props)
# ---------------------------------------------------------------------------

def _norm(s):
    return re.sub(r"\s+", " ", (s or "").replace("_", " ")).strip()


def classify(product, pairing):
    """'app' | 'bootloader' | 'other' from the USB product string (VID:PID is shared)."""
    p = _norm(product).lower()
    b = pairing["board"]
    if p == _norm(b["bootloader_product"]).lower() or " bl " in f" {p} ":
        return "bootloader"
    if p.startswith(_norm(b["app_product"]).lower().rsplit(".", 1)[0]):
        return "app"
    return "other"


def scan_boards(h, pairing):
    """Every /dev/serial/by-id entry whose VID:PID is the board's, with its mode and serial."""
    usb = pairing["board"]["usb"]
    found, seen = [], []
    for name, path in h.by_id():
        seen.append(name)
        props = h.udev(path) or {}
        vid = (props.get("ID_VENDOR_ID") or "").lower()
        pid = (props.get("ID_MODEL_ID") or "").lower()
        if (vid, pid) != (usb["vid"].lower(), usb["pid"].lower()):
            continue
        product = props.get("ID_MODEL") or name
        found.append({"name": name, "path": path, "vid": vid, "pid": pid, "product": _norm(product),
                      "serial": props.get("ID_SERIAL_SHORT") or props.get("ID_USB_SERIAL_SHORT"),
                      "mode": classify(product, pairing)})
    return found, seen


# ---------------------------------------------------------------------------
# param files (QGC format: sysid compid NAME value type)
# ---------------------------------------------------------------------------

def parse_params(text):
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        if len(f) < 4:
            continue
        try:
            out[f[2]] = float(f[3])
        except ValueError:
            continue
    return out


def diff_params(a, b):
    """Compare by name and numeric value. a=this vehicle, b=reference."""
    changed = {n: (a[n], b[n]) for n in sorted(a.keys() & b.keys())
               if not math.isclose(a[n], b[n], rel_tol=1e-6, abs_tol=1e-9)}
    return {"only_in_vehicle": sorted(a.keys() - b.keys()),
            "only_in_reference": sorted(b.keys() - a.keys()),
            "changed": changed,
            "same": len(a.keys() & b.keys()) - len(changed)}


def diff_summary(d, limit=15):
    lines = [f"{d['same']} identical, {len(d['changed'])} differ, "
             f"{len(d['only_in_vehicle'])} only on the vehicle, {len(d['only_in_reference'])} only in the reference"]
    for n, (x, y) in list(d["changed"].items())[:limit]:
        lines.append(f"  differ  {n}: vehicle {x:g}  reference {y:g}")
    for key, label in (("only_in_vehicle", "vehicle-only"), ("only_in_reference", "reference-only")):
        if d[key]:
            lines.append(f"  {label}: {', '.join(d[key][:limit])}{' ...' if len(d[key]) > limit else ''}")
    return "\n".join(lines)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# artifact + backup records
# ---------------------------------------------------------------------------

def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def artifact_state(path, pairing):
    """(status, detail, info) for a built artifact against its record and the pairing."""
    p = Path(path)
    if not p.is_file():
        return "absent", f"{p} does not exist", {}
    digest = sha256(p)
    hdr = read_json(p) or {}
    info = {"sha256": digest, "size": p.stat().st_size, "board_id": hdr.get("board_id")}
    if hdr.get("board_id") != pairing["board"]["board_id"]:
        return "wrong_board", f"{p} names board_id {hdr.get('board_id')}, not {pairing['board']['board_id']}", info
    rec = read_json(str(p) + RECORD)
    if not rec:
        return "unrecorded", f"{p} sha256 {digest[:16]} has no record from --record", info
    if rec.get("sha256") != digest:
        return "mismatch", f"{p} sha256 {digest[:16]} differs from the recorded {str(rec.get('sha256'))[:16]}", info
    if rec.get("commit") != pairing["firmware"]["commit"]:
        return "stale", f"{p} was built from {str(rec.get('commit'))[:10]}, not the pinned {pairing['firmware']['commit'][:10]}", info
    return "ok", f"{p} {info['size']} bytes sha256 {digest}", info


def backup_state(path, board, max_age, now=None):
    """(ok, detail) for a --backup-params file against its sidecar and the board."""
    p = Path(path) if path else None
    if not p or not p.is_file():
        return False, "no param backup file given or it does not exist"
    meta = read_json(str(p) + META)
    if not meta:
        return False, f"{p} has no {META} sidecar (it was not made by --backup-params)"
    if meta.get("sha256") != sha256(p):
        return False, f"{p} changed since it was backed up"
    n = len(parse_params(p.read_text()))
    if n == 0 or meta.get("n_params") != n:
        return False, f"{p} holds {n} parameters (recorded {meta.get('n_params')})"
    age = (now if now is not None else time.time()) - float(meta.get("time", 0))
    if age > max_age or age < -60:
        return False, f"{p} is {age / 60:.0f} min old; max {max_age / 60:.0f} min (take a fresh one)"
    if not board or meta.get("serial") != board.get("serial"):
        return False, f"{p} was taken from serial {meta.get('serial')!r}, the board is {board and board.get('serial')!r}"
    return True, f"{p}: {n} parameters, {age / 60:.0f} min old, serial {meta.get('serial')!r}"


def find_holders(ps_text, names=DRIVERS, own_pid=None):
    """Names from ``names`` running, per ``ps -eo pid=,args=`` text (executable basename match)."""
    hit = set()
    own = str(own_pid if own_pid is not None else os.getpid())
    for line in ps_text.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) < 2 or parts[0] == own:
            continue
        try:
            argv = shlex.split(parts[1])
        except ValueError:
            argv = parts[1].split()
        cands = {os.path.basename(argv[0])} if argv else set()
        if argv and re.match(r"python|ros2$", os.path.basename(argv[0])):
            cands |= {os.path.basename(a) for a in argv[1:3]}
        hit |= {n for n in names if n in cands}
    return sorted(hit)


# ---------------------------------------------------------------------------
# --check
# ---------------------------------------------------------------------------

def custom_hash(custom):
    """First 10 hex digits of the git hash from AUTOPILOT_VERSION.flight_custom_version.

    The board sends the hash as a little-endian uint64 (hash prefix in the high
    bytes), so the wire bytes are reversed: 00 00 00 ff 07 04 c4 99 -> 99c40407ff.
    (Recorded from a v1.15.4 Aurelia X6 whose old printout was 000000ff0704c499.)"""
    return bytes(custom)[::-1].hex()[:10] if custom else ""


def mav_version(sw, custom):
    return (sw >> 24) & 0xFF, (sw >> 16) & 0xFF, (sw >> 8) & 0xFF, custom_hash(custom)


def run_checks(h, pairing, a):
    cs = []
    fw, msgs, b = pairing["firmware"], pairing["px4_msgs"], pairing["board"]
    usb = b["usb"]
    boards, seen = scan_boards(h, pairing)
    board = boards[0] if len(boards) == 1 else None
    ids = f"{usb['vid']}:{usb['pid']}"
    if not boards:
        cs.append(chk("board", FAIL, f"no {ids} device in {BY_ID} (seen: {', '.join(seen) or 'empty'})",
                      "connect the Pixhawk's USB-C port and power it", "only an FMU v6X is acted on"))
    elif len(boards) > 1:
        cs.append(chk("board", FAIL, f"{len(boards)} devices with {ids}: {', '.join(x['name'] for x in boards)}",
                      "connect exactly one flight controller", "the target must be unambiguous"))
    elif board["mode"] == "bootloader":
        cs.append(chk("board", WARN, f"{board['name']} is in the BOOTLOADER ({board['product']}), serial {board['serial']!r}",
                      "power-cycle the board to return to the application", "flashing starts from app mode"))
    elif board["mode"] == "app":
        cs.append(chk("board", PASS, f"{board['product']} {ids} app mode, serial {board['serial']!r}, {board['path']}"))
    else:
        cs.append(chk("board", FAIL, f"{board['name']} has {ids} but product {board['product']!r}",
                      "", "only FMU v6X is acted on"))

    sub = ROOT / fw["path"]
    head = h.git_head(sub)
    mhead = h.git_head(ROOT / msgs["path"])
    problems = []
    if head is None:
        problems.append(f"{fw['path']} is not checked out")
    elif head != fw["commit"]:
        problems.append(f"{fw['path']} is at {head[:10]}, pinned {fw['commit'][:10]}")
    if mhead is None:
        problems.append(f"{msgs['path']} is not checked out")
    elif mhead != msgs["commit"]:
        problems.append(f"{msgs['path']} is at {mhead[:10]}, pinned {msgs['commit'][:10]}")
    pin = f"PX4-Autopilot {fw['tag']} ({fw['commit'][:10]}) pairs with px4_msgs {msgs['branch']} ({msgs['commit'][:7]})"
    if problems:
        cs.append(chk("pins", WARN, pin + "; " + "; ".join(problems),
                      f"git submodule update --init --depth 1 {fw['path']} {msgs['path']}",
                      "firmware and px4_msgs must be the same message set"))
    else:
        cs.append(chk("pins", PASS, pin + "; both checked out at the pin"))

    want = tag_tuple(fw["tag"])
    if a.no_probe:
        cs.append(chk("current_version", INFO, f"not probed (--no-probe); pinned {fw['tag']}"))
    elif not board or board["mode"] != "app":
        cs.append(chk("current_version", INFO, f"no board in app mode to ask; pinned {fw['tag']}"))
    else:
        busy, who = h.port_busy(board["path"])
        if busy is not False:
            cs.append(chk("current_version", INFO,
                          f"port {'busy (' + who + ')' if busy else 'not checkable'}; pinned {fw['tag']}"))
        else:
            sw, custom = h.probe_version(board["path"])
            if sw is None:
                cs.append(chk("current_version", INFO, f"{custom}; pinned {fw['tag']}"))
            else:
                ma, mi, pa, hx = mav_version(sw, custom)
                have = f"v{ma}.{mi}.{pa}" + (f" (git {hx})" if hx else "")
                if (ma, mi, pa) == want:
                    cs.append(chk("current_version", PASS, f"board runs {have}, the pinned {fw['tag']}"))
                else:
                    cs.append(chk("current_version", WARN, f"board runs {have}; pinned {fw['tag']}",
                                  "python3 scripts/firmware.py --build", "px4_msgs only matches the pinned firmware"))

    art = Path(a.artifact) if a.artifact else artifact_path(pairing, a.out)
    st, detail, _ = artifact_state(art, pairing)
    if st == "ok":
        cs.append(chk("artifact", PASS, detail))
    elif st == "absent":
        cs.append(chk("artifact", INFO, "no build yet: " + detail, "python3 scripts/firmware.py --build",
                      "the artifact is built with PX4's own toolchain image"))
    else:
        cs.append(chk("artifact", WARN, detail, "python3 scripts/firmware.py --record" if st == "unrecorded"
                      else "python3 scripts/firmware.py --build", "flash refuses an artifact that does not match its record"))

    up = ROOT / fw["path"] / "Tools" / "px_uploader.py"
    if up.is_file() and h.have_pyserial():
        cs.append(chk("uploader", PASS, f"{up} present, pyserial importable"))
    else:
        miss = ([] if up.is_file() else [f"{up} missing"]) + ([] if h.have_pyserial() else ["pyserial not importable"])
        cs.append(chk("uploader", INFO, "; ".join(miss),
                      "git submodule update --init --depth 1 " + fw["path"] if not up.is_file()
                      else "python3 -m pip install --user pyserial", "px_uploader.py needs both"))
    cs.append(h.modemmanager_row(board["path"] if board else a.device))
    return cs


def msgs_series(pairing):
    """(major, minor) of the PX4 release the px4_msgs pin describes (branch 'release/1.15'), or None."""
    m = re.search(r"(\d+)\.(\d+)", pairing["px4_msgs"].get("branch", ""))
    return (int(m.group(1)), int(m.group(2))) if m else None


def versioned_topics(topics):
    """The /fmu/out/ names with a _vN suffix: PX4 >= 1.16 publishes them, a v1.15 px4_msgs lacks them."""
    return sorted(t for t in topics or [] if re.fullmatch(r"/fmu/out/\w+_v\d+", t))


def firmware_row(h, pairing, device=None, probe=True, topics=None):
    """The guide's ``firmware`` row: the FC's PX4 version against the px4_msgs pin. Read-only.

    The version comes from the MAVLink AUTOPILOT_VERSION probe (``h.probe_version``; INFO when the port is
    busy or pymavlink is absent), else is inferred from versioned /fmu/out topic names (``topics``).
    FAIL on a mismatch; ``manual`` lists both remedies without choosing."""
    msgs, fw = pairing["px4_msgs"], pairing["firmware"]
    series = msgs_series(pairing)
    want = f"v{series[0]}.{series[1]}" if series else msgs["branch"]
    pin = f"px4_msgs pin {msgs['branch']} ({msgs['commit'][:7]}) describes PX4 {want}"
    boards, _ = scan_boards(h, pairing)
    board = boards[0] if len(boards) == 1 and boards[0]["mode"] == "app" else None
    have, how, why_not = None, "", []
    if not probe:
        why_not.append("not probed (--no-probe)")
    elif not board and not device:
        why_not.append("no board in app mode to ask")
    else:
        path = board["path"] if board else device
        busy, who = h.port_busy(path)
        if busy is not False:
            why_not.append(f"port {'busy (' + who + ')' if busy else 'not checkable'}")
        else:
            sw, custom = h.probe_version(path)
            if sw is None:
                why_not.append(str(custom))
            else:
                ma, mi, pa, hx = mav_version(sw, custom)
                have, how = (ma, mi), f"v{ma}.{mi}.{pa}" + (f" (git {hx})" if hx else "")
    if have is None and versioned_topics(topics):
        have, how = (999, 0), "newer than v1.15 (inferred from versioned topic names, e.g. " + versioned_topics(topics)[0] + ")"
    if have is None:
        return chk("firmware", INFO, "; ".join(why_not) + f"; {pin}")
    if series and have == series:
        return chk("firmware", PASS, f"FC runs {how}; {pin}")
    row = chk("firmware", FAIL, f"FC runs {how}; {pin}",
              why="the px4_msgs build only matches the firmware it was generated from")
    row["manual"] = [
        f"Either reflash the FC to the pinned {fw['tag']}: README Firmware section "
        "(python3 scripts/firmware.py --build, then --flash --dry-run).",
        "Or re-pin px4_msgs to the release the FC runs, and update px4_control_node for its versioned topic names.",
        "This check does not choose between them.",
    ]
    return row


# ---------------------------------------------------------------------------
# --build (prints), --record
# ---------------------------------------------------------------------------

def build_commands(pairing, out, ccache):
    fw = pairing["firmware"]
    src = (ROOT / fw["path"]).resolve()
    out, ccache = Path(out).resolve(), Path(ccache).resolve()
    t = pairing["board"]["target"]
    q = shlex.quote
    mounts = [f"-v {q(f'{src}:{src}:rw')}", f"-v {q(f'{out}:{src}/build:rw')}", f"-v {q(f'{ccache}:/ccache:rw')}"]
    git = src / ".git"
    if git.is_file():     # checked-out submodule: its gitdir lives in the superproject, same path in the container
        m = re.match(r"gitdir:\s*(.+)", git.read_text())
        if m:
            gd = (src / m.group(1).strip()).resolve()
            mounts.append(f"-v {q(f'{gd}:{gd}:rw')}")
    docker = ("docker run --rm -u \"$(id -u):$(id -g)\" -e HOME=/tmp -e CI=true -e CCACHE_DIR=/ccache "
              "-e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory -e GIT_CONFIG_VALUE_0='*' "
              + " ".join(mounts) + f" -w {q(str(src))} {pairing['toolchain']['image']} make {t}")
    subs = " ".join(q(s) for s in pairing["submodules"])
    cmds = [f"mkdir -p {q(str(out))} {q(str(ccache))} {q(str(src / 'build'))}",
            f"git -C {q(str(src))} submodule update --init --depth 1 -- {subs}"]
    nx = pairing.get("nuttx")
    if nx:   # a depth-1 NuttX has no tags; PX4's version-header script needs an nuttx-X.Y.Z one
        repo, tag = src / nx["path"], nx["tag"]
        cmds.append(f"git -C {q(str(repo))} fetch --depth 1 --no-tags origin "
                    f"{q(f'refs/tags/{tag}:refs/tags/{tag}')}")
    return cmds + [docker,
                   f"python3 {q(str(Path(__file__).resolve()))} --record --out {q(str(out))}"]


def record_artifact(h, pairing, path):
    fw = pairing["firmware"]
    p = Path(path)
    if not p.is_file():
        return None, f"{p} does not exist (run the --build commands first)"
    hdr = read_json(p) or {}
    if hdr.get("board_id") != pairing["board"]["board_id"]:
        return None, f"{p} names board_id {hdr.get('board_id')}, not {pairing['board']['board_id']}"
    head = h.git_head(ROOT / fw["path"])
    if head != fw["commit"]:
        return None, f"{fw['path']} is at {head}, not the pinned {fw['commit']}"
    rec = {"sha256": sha256(p), "size": p.stat().st_size, "time": time.time(), "commit": head,
           "target": pairing["board"]["target"], "board_id": hdr["board_id"]}
    Path(str(p) + RECORD).write_text(json.dumps(rec, indent=2) + "\n")
    return rec, ""


# ---------------------------------------------------------------------------
# --backup-params (read-only)
# ---------------------------------------------------------------------------

BACKUP_NS = "fw_backup"
BACKUP_DOMAIN = 95            # isolated ROS domain (with ROS_LOCALHOST_ONLY=1); env CPSL_FW_BACKUP_DOMAIN or --backup-domain
BACKUP_PLUGINS = ("sys_status", "sys_time", "param")   # nothing that can command the vehicle


def backup_params_yaml(fcu_url):
    """mavros_node params for the temporary node: three plugins, no tf, no GCS bridge."""
    return ("/**:\n  ros__parameters:\n"
            f"    fcu_url: \"{fcu_url}\"\n    gcs_url: \"\"\n    tgt_system: 1\n    tgt_component: 1\n"
            "    fcu_protocol: \"v2.0\"\n    startup_px4_usb_quirk: false\n    plugin_allowlist:\n"
            + "".join(f"      - {p}\n" for p in BACKUP_PLUGINS)
            + "/**/local_position:\n  ros__parameters:\n    tf.send: false\n"
            "/**/global_position:\n  ros__parameters:\n    tf.send: false\n")


def backup_commands(fcu_url, cfg, dest_ns=BACKUP_NS):
    """The exact commands, in order: node, state poll, pull, read. None of them can write."""
    node = f"/{dest_ns}/mavros"
    return {
        "node": ["ros2", "run", "mavros", "mavros_node", "--ros-args", "-r", f"__ns:=/{dest_ns}",
                 "--params-file", str(cfg)],
        "state": ["ros2", "topic", "echo", "--once", "--field", "connected", f"{node}/state"],
        "pull": ["ros2", "service", "call", f"{node}/param/pull", "mavros_msgs/srv/ParamPull", "{force_pull: true}"],
        # read path that worked on the bench (px4-03): the param plugin's own ROS parameters, read
        # with the standard parameter services (the plugin's param/get service hung).
        "dump": ["ros2", "param", "dump", f"{node}/param"],
    }


def parse_ros_param_dump(text):
    """``ros2 param dump`` YAML (``NAME: value`` under ros__parameters) -> {NAME: float}; PX4 names only."""
    out = {}
    for line in text.splitlines():
        m = re.match(r"^\s+([A-Z][A-Z0-9_]*):\s*(\S+)\s*$", line)
        if not m:
            continue
        try:
            out[m.group(1)] = float(m.group(2))
        except ValueError:
            continue
    return out


def qgc_text(params):
    """QGC param-file text. The type column is a guess from the value (6 INT32, 9 REAL32): the
    diff and the flash guard compare names and values only."""
    lines = ["# Onboard parameters (backup via firmware.py --backup-params; types are approximate)",
             "# Vehicle-Id Component-Id Name Value Type"]
    for n in sorted(params):
        v = params[n]
        lines.append(f"1\t1\t{n}\t{int(v) if float(v).is_integer() else repr(v)}\t{6 if float(v).is_integer() else 9}")
    return "\n".join(lines) + "\n"


def do_backup(h, pairing, dest, domain=None, connect_timeout=30.0, dump_timeout=120.0):
    """Self-contained read-only backup: own temporary mavros_node (3 plugins), pull, read, ALWAYS stop.

    Refuses when the destination exists, the board is not exactly one device, the USB port is
    held (or cannot be checked), or any mavros_node already runs. The XRCE driver does not use
    the USB port, so it may keep running. Never sets/loads a parameter, arms, changes mode or publishes.
    """
    dest = Path(dest)
    if dest.exists():
        return None, f"{dest} exists; a backup is never overwritten (pick a new name)"
    boards, seen = scan_boards(h, pairing)
    if len(boards) != 1:
        return None, (f"need exactly one {pairing['board']['usb']['vid']}:{pairing['board']['usb']['pid']} device "
                      f"(found {len(boards)}; seen: {', '.join(seen) or 'empty'})")
    board = boards[0]
    busy, who = h.port_busy(board["path"])
    if busy is None:
        return None, f"cannot tell whether {board['path']} is free ({who}); install psmisc (fuser) or lsof"
    if busy:
        return None, f"{board['path']} is held by pid(s) {who}; stop whatever holds it (QGC? MAVROS?)"
    running = find_holders(h.ps(), names=("mavros_node",))
    if running:
        return None, "a mavros_node is already running; stop the MAVROS driver first"
    if domain is None:
        domain = os.environ.get("CPSL_FW_BACKUP_DOMAIN", BACKUP_DOMAIN)
    env = h.ros_env(domain)
    tmp = Path(tempfile.mkdtemp(prefix="fw_backup_"))
    cfg, log = tmp / "mavros.yaml", tmp / "mavros.log"
    fcu_url = f"{board['path']}:115200"
    cfg.write_text(backup_params_yaml(fcu_url))
    cmds = backup_commands(fcu_url, cfg)
    proc, err, text = None, "", ""
    try:
        proc = h.spawn(cmds["node"], env, log)
        end = time.monotonic() + connect_timeout
        connected = False
        while time.monotonic() < end:
            if proc.poll() is not None:
                err = f"temporary mavros_node exited (rc {proc.poll()}): {_tail(log)}"
                break
            rc, out = h.ros(cmds["state"], env, 8)
            if rc == 0 and "true" in out.lower():
                connected = True
                break
            h.sleep(1.0)
        if not err and not connected:
            err = f"mavros did not connect to {fcu_url} within {connect_timeout:.0f} s: {_tail(log)}"
        if not err:
            rc, out = h.ros(cmds["pull"], env, dump_timeout)
            m = re.search(r"param_received=(\d+)", out)
            if rc != 0 or "success=True" not in out or not m or int(m.group(1)) == 0:
                err = f"param/pull failed (rc {rc}): {out.strip()[-300:]}"
        if not err:
            rc, text = h.ros(cmds["dump"], env, dump_timeout)
            if rc != 0:
                err = f"reading the parameters failed (rc {rc}): {text.strip()[-300:]}"
    except Exception as e:  # noqa: BLE001 - cleanup below is unconditional
        err = f"backup failed: {type(e).__name__}: {e}"
    finally:
        gone = h.stop(proc)
        free, holder = h.wait_port_free(board["path"])
        shutil.rmtree(tmp, ignore_errors=True)
    if not gone or not free:
        extra = "temporary mavros_node would not die" if not gone else f"port still held by {holder}"
        return None, f"{err + '; ' if err else ''}CLEANUP FAILED: {extra} (kill it by hand)"
    if err:
        return None, err
    params = parse_ros_param_dump(text)
    if not params:
        return None, "the dump holds no parameters"
    dest.write_text(qgc_text(params))
    meta = {"time": time.time(), "sha256": sha256(dest), "n_params": len(params), "serial": board["serial"],
            "method": "mavros_node(sys_status,sys_time,param) param/pull + ros2 param dump"}
    Path(str(dest) + META).write_text(json.dumps(meta, indent=2) + "\n")
    return meta, ""


def _tail(path, n=300):
    try:
        return Path(path).read_text(errors="replace").strip()[-n:]
    except OSError:
        return ""


# ---------------------------------------------------------------------------
# --flash
# ---------------------------------------------------------------------------

def flash_command(pairing, board, artifact):
    up = ROOT / pairing["firmware"]["path"] / "Tools" / "px_uploader.py"
    ports = ",".join([board["path"], f"{BY_ID}/*PX4_BL*", f"{BY_ID}/*BL_FMU*"])
    return [sys.executable, str(up), "--port", ports, str(artifact)]


def flash_guards(h, pairing, a):
    rows = []
    usb = pairing["board"]["usb"]
    ids = f"{usb['vid']}:{usb['pid']}"
    boards, seen = scan_boards(h, pairing)
    board = boards[0] if len(boards) == 1 else None
    if board and board["mode"] == "app":
        rows.append(chk("identity", PASS, f"{board['product']} {ids} app mode, serial {board['serial']!r}"))
    else:
        why = (f"no {ids} device (seen: {', '.join(seen) or 'empty'})" if not boards else
               f"{len(boards)} devices with {ids}" if len(boards) > 1 else
               f"{board['name']} is {board['mode']} mode ({board['product']!r}), not the application")
        rows.append(chk("identity", FAIL, why, "", "only a single FMU v6X in app mode is flashed"))
        board = None

    typed = a.confirm_serial
    if board and typed is not None and typed == board["serial"]:
        rows.append(chk("serial_confirm", PASS, "typed serial matches the board"))
    else:
        rows.append(chk("serial_confirm", FAIL,
                        "no --confirm-serial given" if typed is None else "typed serial does not match the board's",
                        "re-run with --confirm-serial <the board's serial from --check>",
                        "a typed serial proves the operator is looking at this board"))

    ok, detail = backup_state(a.backup, board, a.backup_max_age)
    rows.append(chk("backup", PASS if ok else FAIL, detail,
                    "" if ok else "python3 scripts/firmware.py --backup-params FILE --diff LAB",
                    "a flash can change parameters; the backup is the only way to compare"))

    run = find_holders(h.ps())
    rows.append(chk("driver_stopped", FAIL if run else PASS,
                    ("running: " + ", ".join(run)) if run else "none of " + "/".join(DRIVERS) + " is running",
                    "stop the PX4 driver (xrce agent / mavros / console's PX4 start) and QGroundControl" if run else "",
                    "they hold the port or fight the reboot into the bootloader"))

    art = Path(a.artifact) if a.artifact else artifact_path(pairing, a.out)
    st, detail, _ = artifact_state(art, pairing)
    rows.append(chk("artifact", PASS if st == "ok" else FAIL, detail,
                    "" if st == "ok" else "python3 scripts/firmware.py --build  (then its --record line)",
                    "only the artifact whose sha256 was recorded after the pinned build is flashed"))

    if board:
        busy, who = h.port_busy(board["path"])
        rows.append(chk("port_free", PASS if busy is False else FAIL,
                        "no process holds the port" if busy is False else
                        f"port held by: {who}" if busy else who,
                        "" if busy is False else "stop whatever holds it (see driver_stopped)", "the uploader must open it"))
    else:
        rows.append(chk("port_free", FAIL, "no board to check", "", ""))

    up = ROOT / pairing["firmware"]["path"] / "Tools" / "px_uploader.py"
    okup = up.is_file() and h.have_pyserial()
    rows.append(chk("uploader", PASS if okup else FAIL,
                    f"{up} present, pyserial importable" if okup else
                    "; ".join(([] if up.is_file() else [f"{up} missing"]) + ([] if h.have_pyserial() else ["pyserial missing"])),
                    "" if okup else "git submodule update --init --depth 1 firmware/PX4-Autopilot; python3 -m pip install --user pyserial",
                    "the pinned px_uploader.py does the flashing"))
    mm = h.modemmanager_row(board["path"] if board else a.device)
    rows.append(dict(mm, status=WARN if mm["status"] == WARN else mm["status"]))
    return rows, board, art


def flash_main(h, pairing, a):
    rows, board, art = flash_guards(h, pairing, a)
    refused = [r for r in rows if r["status"] == FAIL]
    if a.json:
        json.dump({"guards": rows, "ok": not refused,
                   "command": None if refused else flash_command(pairing, board, art)}, sys.stdout, indent=2)
        print()
    else:
        print_rows(rows)
    if refused:
        print("REFUSED: " + ", ".join(r["name"] for r in refused), file=sys.stderr)
        return 1
    cmd = flash_command(pairing, board, art)
    if a.dry_run:
        if not a.json:
            print("dry run, would run: " + " ".join(shlex.quote(c) for c in cmd))
        return 0
    print("run: " + " ".join(shlex.quote(c) for c in cmd))
    print("The uploader reboots the board into its bootloader; follow its prompts. Do not touch the cable.")
    rc = subprocess.call(cmd)
    print("uploader exited %d; re-run: python3 scripts/firmware.py --check" % rc)
    return rc


def print_rows(rows):
    for c in rows:
        print(f"[{c['status']:<4}] {c['name']:<15} {c['detail']}")
        if c["status"] in (FAIL, WARN) and c["fix"]:
            print(f"         fix: {c['fix']}")


def main(argv=None, host=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    m = ap.add_mutually_exclusive_group(required=True)
    m.add_argument("--check", action="store_true", help="read-only report")
    m.add_argument("--backup-params", metavar="FILE", help="read-only param dump (QGC format) via its own temporary MAVROS; the USB port must be free")
    m.add_argument("--diff-params", nargs=2, metavar=("FILE", "REFERENCE"), help="compare two param files")
    m.add_argument("--build", action="store_true", help="PRINT the build commands (nothing is run)")
    m.add_argument("--record", action="store_true", help="record the built artifact's sha256")
    m.add_argument("--flash", action="store_true", help="guarded flash; needs --dry-run or --apply")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--no-probe", action="store_true", help="--check: do not open the serial port to read the version")
    ap.add_argument("--backup-domain", type=int, default=None, help="--backup-params: ROS_DOMAIN_ID (default $CPSL_FW_BACKUP_DOMAIN or 95; localhost only)")
    ap.add_argument("--diff", metavar="LAB_FILE", help="--backup-params: also diff against this QGC param file")
    ap.add_argument("--out", default=str(ROOT / "firmware" / "out"), help="build output dir (gitignored)")
    ap.add_argument("--ccache", default=None, help="--build: ccache dir (default <out>/ccache)")
    ap.add_argument("--artifact", help="the .px4 file (default <out>/<target>/<target>.px4)")
    ap.add_argument("--device", default="/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00")
    ap.add_argument("--backup", metavar="FILE", help="--flash: the param backup made this session")
    ap.add_argument("--backup-max-age", type=float, default=3600.0, help="seconds (default %(default)s)")
    ap.add_argument("--confirm-serial", help="--flash: the board's serial, typed")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-run", "-n", action="store_true")
    g.add_argument("--apply", action="store_true")
    a = ap.parse_args(argv)
    if a.flash and not (a.dry_run or a.apply):
        ap.error("--flash needs --dry-run or --apply")
    if (a.dry_run or a.apply) and not a.flash:
        ap.error("--dry-run/--apply belong to --flash")
    h = host or Host()

    if a.diff_params:
        mine, ref = (parse_params(Path(p).read_text()) for p in a.diff_params)
        d = diff_params(mine, ref)
        print(json.dumps(d, indent=2) if a.json else diff_summary(d))
        return 0
    try:
        pairing = load_pairing()
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    if a.check:
        rows = run_checks(h, pairing, a)
        if a.json:
            json.dump(rows, sys.stdout, indent=2)
            print()
        else:
            print_rows(rows)
        return 1 if any(r["status"] == FAIL for r in rows) else 0
    if a.backup_params:
        meta, err = do_backup(h, pairing, a.backup_params, domain=a.backup_domain)
        if err:
            print(f"error: {err}", file=sys.stderr)
            return 1
        print(f"backed up {meta['n_params']} parameters to {a.backup_params} (serial {meta['serial']!r})")
        if a.diff:
            d = diff_params(parse_params(Path(a.backup_params).read_text()), parse_params(Path(a.diff).read_text()))
            print(diff_summary(d))
        return 0
    if a.build:
        cmds = build_commands(pairing, a.out, a.ccache or Path(a.out) / "ccache")
        art = str(artifact_path(pairing, a.out))
        if a.json:
            json.dump({"commands": cmds, "artifact": art, "image": pairing["toolchain"]["image"]}, sys.stdout, indent=2)
            print()
        else:
            print("Run these yourself (or via a Runner); this tool runs none of them.\n"
                  "Needs docker and network; the build takes a while. Artifact: " + art)
            print("\n".join(cmds))
        return 0
    if a.record:
        art = Path(a.artifact) if a.artifact else artifact_path(pairing, a.out)
        rec, err = record_artifact(h, pairing, art)
        if err:
            print(f"error: {err}", file=sys.stderr)
            return 1
        print(f"recorded {art}: {rec['size']} bytes sha256 {rec['sha256']}")
        return 0
    return flash_main(h, pairing, a)


if __name__ == "__main__":
    sys.exit(main())
