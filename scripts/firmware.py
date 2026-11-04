#!/usr/bin/env python3
"""Pinned PX4 firmware for the Pixhawk 6X: check, back up params, build (command), flash (guarded).

    python3 scripts/firmware.py --check [--json] [--no-probe]        # read-only
    python3 scripts/firmware.py --backup-params FILE [--diff LAB]    # read-only (needs MAVROS up)
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
import subprocess
import sys
import time
from pathlib import Path

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
        rc, out = self.run(["udevadm", "info", "-q", "property", "-n", path])
        if rc != 0:
            return None
        return dict(l.split("=", 1) for l in out.splitlines() if "=" in l)

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
        return None, "neither fuser nor lsof could answer"

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
        try:
            from pymavlink import mavutil
        except ImportError:
            return None, "pymavlink not installed"
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

def mav_version(sw, custom):
    return (sw >> 24) & 0xFF, (sw >> 16) & 0xFF, (sw >> 8) & 0xFF, custom.hex() if custom else ""


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
    return [f"mkdir -p {q(str(out))} {q(str(ccache))} {q(str(src / 'build'))}",
            f"git -C {q(str(src))} submodule update --init --depth 1 -- {subs}",
            docker,
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

def backup_command(dest):
    return ["ros2", "run", "mavros", "mav", "param", "dump", "-f", "-qgc", str(dest)]


def do_backup(h, pairing, dest, runner=subprocess.run):
    dest = Path(dest)
    if dest.exists():
        return None, f"{dest} exists; a backup is never overwritten (pick a new name)"
    try:
        r = runner(backup_command(dest), capture_output=True, text=True, timeout=180)
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, f"could not run ros2/mavros: {e}"
    if r.returncode != 0 or not dest.is_file():
        return None, f"mavros param dump failed (rc {r.returncode}): {(r.stderr or r.stdout).strip()[-300:]}"
    n = len(parse_params(dest.read_text()))
    if n == 0:
        return None, f"{dest} holds no parameters (is MAVROS connected to the board?)"
    boards, _ = scan_boards(h, pairing)
    serial = boards[0]["serial"] if len(boards) == 1 else None
    meta = {"time": time.time(), "sha256": sha256(dest), "n_params": n, "serial": serial}
    Path(str(dest) + META).write_text(json.dumps(meta, indent=2) + "\n")
    return meta, ""


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
    m.add_argument("--backup-params", metavar="FILE", help="read-only param dump via MAVROS (QGC format)")
    m.add_argument("--diff-params", nargs=2, metavar=("FILE", "REFERENCE"), help="compare two param files")
    m.add_argument("--build", action="store_true", help="PRINT the build commands (nothing is run)")
    m.add_argument("--record", action="store_true", help="record the built artifact's sha256")
    m.add_argument("--flash", action="store_true", help="guarded flash; needs --dry-run or --apply")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--no-probe", action="store_true", help="--check: do not open the serial port to read the version")
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
        meta, err = do_backup(h, pairing, a.backup_params)
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
