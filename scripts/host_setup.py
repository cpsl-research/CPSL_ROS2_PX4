#!/usr/bin/env python3
"""Host checks for a Pixhawk: the direct Ethernet link (uXRCE-DDS), or its USB port (MAVLink, --usb).

    python3 scripts/host_setup.py --iface enp3s0 --json       # checks, read-only
    python3 scripts/host_setup.py --iface enp3s0              # same, human readable
    python3 scripts/host_setup.py --iface enp3s0 --apply --dry-run   # print what --apply would run
    python3 scripts/host_setup.py --iface enp3s0 --apply      # ask once, then run (sudo per command)
    python3 scripts/host_setup.py --iface enp3s0 --undo       # remove what --apply created

Checks (each {name,status,detail,fix,why}; status PASS / FAIL / WARN / INFO; exit 1 iff any FAIL):

  interface  the interface exists and has carrier.
  address    the interface holds --host-ip with --prefix.
  profile    the NetworkManager profile --con-name exists. INFO when nmcli cannot be read
             (e.g. in a container); an unreadable state is never PASS.
  fcu        the flight controller (--fcu-ip) answers one ping.

``--apply`` never runs as root: it calls sudo per command, after one confirmation. It is
idempotent and never rewrites a profile it did not create: a profile is ours only when it
carries connection.stable-id ``console-px4``. With NetworkManager it adds a manual-IPv4
profile with ``ipv4.never-default yes`` (this link has no route to anywhere else); without
nmcli it falls back to ``ip addr add``. ``--undo`` removes only what ``--apply`` made:
``nmcli con delete <con-name>`` (our profile only), or ``ip addr del`` for the fallback.

USB (``--usb``, for backend:=mavros; read-only, never needs root):

    python3 scripts/host_setup.py --usb --json                 # checks
    python3 scripts/host_setup.py --usb --device /dev/serial/by-id/<id>[:baud]
    python3 scripts/host_setup.py --usb --apply                # PRINT the ModemManager fix and its undo
    python3 scripts/host_setup.py --usb --undo                 # PRINT the undo

  usb_device    --device (default the Pixhawk 6X's by-id name; a MAVROS ``path:baud`` is accepted)
                exists and is a character device. WARN when it is not a /dev/serial/by-id/ name
                (ttyACM numbers change when the FC re-enumerates).
  modemmanager  ModemManager is not running, or udev tells it to ignore the device
                (ID_MM_DEVICE_IGNORE=1, read with ``udevadm info``). Otherwise WARN: it probes the
                port at every (re)enumeration and MAVROS can fail with "Device or resource busy".
                INFO when this cannot be read (e.g. no systemctl in a container).

With --usb, --apply and --undo only print commands (a udev rule under /etc/udev/rules.d,
then a udev reload/trigger); nothing is run.

Guided setup (``--guide``; read-only, prints nothing it would run):

    python3 scripts/host_setup.py --iface enp3s0 --guide --json [--no-probe]

emits the Pixhawk-side setup (CPSL_Manuals UAVs/PX4_X500.md sections 5-6) as rows in step order, each
with a ``step`` (1..n). Rows may carry two more optional fields: ``kind`` (``check``: a live probe, whose
``fix`` is a shell command only; ``manual``: an instruction with no command) and ``manual`` (instruction
lines; a manual row may also carry ``literal`` {name,text}, a file's contents, and ``confirmed_by``, the
names of the check rows that confirm it). A row without ``kind`` is a check. Besides the checks above:

  fcu-address   (manual + live) ping of --fcu-ip fails but the link has carrier: look for the FCU on
                --iface only, from /proc/net/arp and ``arping -I <iface>`` to --fcu-ip and the PX4
                defaults 192.168.0.3/4 (never an off-subnet ping). FAIL "FCU answers at <ip>, expected
                ..."; INFO when nothing is found or arping lacks CAP_NET_RAW.
  usb           the by-id device (as --usb).
  xrce-session  the agent has a client: ``ros2 topic list`` shows /fmu/out/*, or --agent-log shows a
                session. INFO when ROS / the agent is not running here.
  firmware      the FC's PX4 version (read-only MAVLink AUTOPILOT_VERSION probe, via firmware.py, which
                owns firmware/versions.json) against the px4_msgs pin; FAIL on a mismatch, with both
                remedies in ``manual``. INFO when the port is busy or pymavlink is absent (--no-probe: skip).

Python stdlib only (the firmware probe uses pymavlink through firmware.py when it is installed).
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import stat
import subprocess
import sys

MARK = "console-px4"          # connection.stable-id on every profile this tool creates
USB_DEVICE = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"   # Pixhawk 6X (VID 3185, PID 0035)
USB_IDS = ("3185", "0035")
MM_RULES = "/etc/udev/rules.d/60-px4-fmu-no-modemmanager.rules"
MM_WHY = ("ModemManager probes serial ports as modems. The probe holds a port open briefly, and "
          "closing a tty triggers another one, so MAVROS can fail to open the flight controller's "
          "port with \"Device or resource busy\".")
PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"


def sh(cmd, timeout=5):
    """Run a read-only command; return (rc, stdout) or (None, '') when it cannot run."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return None, ""


def chk(name, status, detail, fix="", why=""):
    return {"name": name, "status": status, "detail": detail, "fix": fix, "why": why}


def nm_profile(con_name):
    """('absent'|'ours'|'foreign'|'unreadable', detail)."""
    if not shutil.which("nmcli"):
        return "unreadable", "nmcli not installed"
    rc, out = sh(["nmcli", "-t", "-f", "NAME", "connection", "show"])
    if rc != 0:
        return "unreadable", "nmcli cannot read NetworkManager"
    if con_name not in out.splitlines():
        return "absent", ""
    rc, sid = sh(["nmcli", "-g", "connection.stable-id", "connection", "show", con_name])
    return ("ours", "") if sid == MARK else ("foreign", "exists without our marker")


def run_checks(a):
    cs = []
    base = f"/sys/class/net/{a.iface}"
    rc, _ = sh(["test", "-d", base])
    present = rc == 0
    carrier = False
    if present:
        rc, c = sh(["cat", f"{base}/carrier"])
        carrier = c == "1"
    if not present:
        cs.append(chk("interface", FAIL, f"{a.iface} does not exist",
                      "pass --iface with the wired interface facing the Pixhawk",
                      "the FCU link is a direct Ethernet cable"))
    elif not carrier:
        cs.append(chk("interface", FAIL, f"{a.iface} has no carrier",
                      "plug in the cable and power the Pixhawk", "no link, no uXRCE-DDS"))
    else:
        cs.append(chk("interface", PASS, f"{a.iface} present, carrier up"))

    want = f"{a.host_ip}/{a.prefix}"
    _, addrs = sh(["ip", "-o", "-4", "addr", "show", "dev", a.iface])
    held = [w for w in addrs.split() if "/" in w]
    if want in held:
        cs.append(chk("address", PASS, f"{a.iface} holds {want}"))
    else:
        cs.append(chk("address", FAIL, f"{a.iface} holds {', '.join(held) or 'no IPv4 address'}, not {want}",
                      f"python3 scripts/host_setup.py --iface {a.iface} --apply",
                      "the Pixhawk expects its peer at this address"))

    state, why = nm_profile(a.con_name)
    if state == "ours":
        cs.append(chk("profile", PASS, f"NetworkManager profile {a.con_name} present (created by this tool)"))
    elif state == "foreign":
        cs.append(chk("profile", WARN, f"profile {a.con_name} {why}; this tool will not touch it",
                      "choose another --con-name, or manage that profile yourself"))
    elif state == "absent":
        cs.append(chk("profile", WARN, f"no NetworkManager profile {a.con_name}",
                      f"python3 scripts/host_setup.py --iface {a.iface} --apply",
                      "without a profile the address is lost on reboot"))
    else:
        cs.append(chk("profile", INFO, f"cannot read NetworkManager ({why})"))

    rc, _ = sh(["ping", "-c", "1", "-W", "1", a.fcu_ip], timeout=4)
    if rc == 0:
        cs.append(chk("fcu", PASS, f"{a.fcu_ip} answers ping"))
    elif rc is None:
        cs.append(chk("fcu", INFO, "ping unavailable"))
    else:
        cs.append(chk("fcu", FAIL, f"{a.fcu_ip} does not answer ping",
                      "power the Pixhawk; check its Ethernet IP and the address above",
                      "uXRCE-DDS needs a reachable FCU"))
    return cs


def apply_plan(a):
    """Commands (argv lists, run under sudo) to reach the wanted state; [] when nothing to do."""
    want = f"{a.host_ip}/{a.prefix}"
    _, addrs = sh(["ip", "-o", "-4", "addr", "show", "dev", a.iface])
    if want in addrs.split():
        return []
    if shutil.which("nmcli") and nm_profile(a.con_name)[0] != "unreadable":
        state = nm_profile(a.con_name)[0]
        if state == "foreign":
            return None  # refuse
        if state == "ours":
            return [["nmcli", "con", "up", a.con_name]]
        return [["nmcli", "con", "add", "type", "ethernet", "ifname", a.iface,
                 "con-name", a.con_name, "connection.stable-id", MARK,
                 "ipv4.method", "manual", "ipv4.addresses", want,
                 "ipv4.never-default", "yes", "ipv6.method", "disabled",
                 "connection.autoconnect", "yes"],
                ["nmcli", "con", "up", a.con_name]]
    return [["ip", "addr", "add", want, "dev", a.iface]]


def undo_plan(a):
    state = nm_profile(a.con_name)[0] if shutil.which("nmcli") else "unreadable"
    if state == "ours":
        return [["nmcli", "con", "delete", a.con_name]]
    if state == "foreign":
        return None
    want = f"{a.host_ip}/{a.prefix}"
    _, addrs = sh(["ip", "-o", "-4", "addr", "show", "dev", a.iface])
    return [["ip", "addr", "del", want, "dev", a.iface]] if want in addrs.split() else []


def undo_text(a):
    return [" ".join(c) for c in (undo_plan(a) or [])] or ["(nothing to undo)"]


def execute(cmds, dry_run):
    for c in cmds:
        line = "sudo " + " ".join(c)
        print(("would run: " if dry_run else "run: ") + line)
        if not dry_run:
            r = subprocess.run(["sudo"] + c)
            if r.returncode != 0:
                print(f"FAILED ({r.returncode}): {line}", file=sys.stderr)
                return 1
    return 0


# ---------------------------------------------------------------------------
# --usb: the Pixhawk's USB port (MAVLink, backend:=mavros)
# ---------------------------------------------------------------------------

def device_path(dev):
    """A MAVROS fcu_url ``/dev/...:baud`` -> the device path; a bare path is returned as is."""
    head, sep, tail = dev.rpartition(":")
    return head if sep and tail.isdigit() else dev


def udev_props(path):
    """{KEY: value} from ``udevadm info``, or None when udev cannot be read."""
    rc, out = sh(["udevadm", "info", "-q", "property", "-n", path])
    if rc != 0:
        return None
    return dict(line.split("=", 1) for line in out.splitlines() if "=" in line)


def sysfs_usb_props(path, sysfs="/sys"):
    """USB identity of a tty device from sysfs, as udev-style keys; {} when not found.

    No udevadm needed (a container has none). ``path`` may be a /dev/serial/by-id symlink:
    its target ttyACMn is looked up at ``<sysfs>/class/tty/ttyACMn/device``, then parents are
    walked up to the USB device directory holding idVendor/idProduct.
    """
    try:
        node = os.path.basename(os.path.realpath(path))
        d = os.path.realpath(os.path.join(sysfs, "class", "tty", node, "device"))
    except OSError:
        return {}
    root = os.path.realpath(sysfs)
    while d.startswith(root + os.sep) and not os.path.exists(os.path.join(d, "idVendor")):
        d = os.path.dirname(d)
    if not d.startswith(root + os.sep):
        return {}

    def rd(name):
        try:
            with open(os.path.join(d, name)) as f:
                return f.read().strip()
        except OSError:
            return ""
    out = {"ID_VENDOR_ID": rd("idVendor").lower(), "ID_MODEL_ID": rd("idProduct").lower()}
    if rd("product"):
        out["ID_MODEL"] = rd("product")
    if rd("serial"):
        out["ID_SERIAL_SHORT"] = rd("serial")
    return out if out["ID_VENDOR_ID"] and out["ID_MODEL_ID"] else {}


def mm_rule_lines(ids):
    vid, pid = ids
    return ["# Keep ModemManager off the USB port of the PX4 flight controller: it probes it",
            "# as a modem, and a probe in flight makes MAVROS fail to open the port.",
            f'ATTRS{{idVendor}}=="{vid}", ATTRS{{idProduct}}=="{pid}", ENV{{ID_MM_DEVICE_IGNORE}}="1"']


UDEV_RELOAD = ("sudo udevadm control --reload-rules"
               " && sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb")


def mm_fix_command(ids):
    body = " ".join(shlex.quote(line) for line in mm_rule_lines(ids))
    return f"printf '%s\\n' {body} | sudo tee {MM_RULES} >/dev/null && {UDEV_RELOAD}"


def mm_undo_command():
    return f"sudo rm -f {MM_RULES} && {UDEV_RELOAD}"


def usb_ids(props):
    if props and props.get("ID_VENDOR_ID") and props.get("ID_MODEL_ID"):
        return props["ID_VENDOR_ID"], props["ID_MODEL_ID"]
    return USB_IDS


def run_usb_checks(a):
    cs = []
    path = device_path(a.device)
    by_id = "/dev/serial/by-id"
    present = os.path.exists(path)
    props = udev_props(path) if present else None
    if not present:
        try:
            seen = sorted(os.listdir(by_id))
        except OSError:
            seen = []
        cs.append(chk("usb_device", FAIL,
                      f"{path} does not exist ({by_id}: {', '.join(seen) or 'empty'})",
                      "connect the Pixhawk's USB-C port to this machine, then ls /dev/serial/by-id/",
                      "MAVROS reads the flight controller through this port"))
    else:
        real = os.path.realpath(path)
        try:
            is_chr = stat.S_ISCHR(os.stat(real).st_mode)
        except OSError:
            is_chr = False
        if not is_chr:
            cs.append(chk("usb_device", FAIL, f"{path} -> {real} is not a character device",
                          "pass --device with the flight controller's /dev/serial/by-id/ name"))
        elif not path.startswith(by_id + "/"):
            cs.append(chk("usb_device", WARN, f"{path} exists but is not a {by_id}/ name",
                          f"use the {by_id}/ name in fcu_url",
                          "ttyACM numbers change when the flight controller re-enumerates"))
        else:
            cs.append(chk("usb_device", PASS, f"{path} -> {real}"))

    ids = usb_ids(props or (sysfs_usb_props(path) if present else None))
    fix = mm_fix_command(ids)
    rc, out = sh(["systemctl", "is-active", "ModemManager"])
    if rc is None or (rc != 0 and out not in ("inactive", "failed", "unknown")):
        cs.append(chk("modemmanager", INFO, "cannot tell whether ModemManager runs (systemctl unavailable)",
                      "", MM_WHY))
    elif out != "active":
        cs.append(chk("modemmanager", PASS, "ModemManager is not running", "", MM_WHY))
    elif not present:
        if os.path.exists(MM_RULES):
            cs.append(chk("modemmanager", PASS, "ModemManager is running; the ignore rule is installed "
                          "(no flight controller plugged in to confirm)", "", MM_WHY))
        else:
            cs.append(chk("modemmanager", WARN, "ModemManager is running and no ignore rule is installed "
                          "for the flight controller", fix, MM_WHY))
    elif props is None:
        cs.append(chk("modemmanager", INFO, f"ModemManager is running; cannot read udev properties of {path}",
                      "", MM_WHY))
    elif props.get("ID_MM_DEVICE_IGNORE") == "1":
        cs.append(chk("modemmanager", PASS, f"ModemManager is told to ignore {path}", "", MM_WHY))
    else:
        cs.append(chk("modemmanager", WARN,
                      f"ModemManager is running and will probe {path}; MAVROS started at the wrong "
                      "moment fails with \"Device or resource busy\".", fix, MM_WHY))
    return cs


def usb_main(a):
    checks = run_usb_checks(a)
    report(checks, a.json)
    rc = 1 if any(c["status"] == FAIL for c in checks) else 0
    if a.apply:
        mm = next(c for c in checks if c["name"] == "modemmanager")
        if mm["status"] == PASS:
            print("nothing to do: " + mm["detail"])
        else:
            print("run this yourself (needs sudo; this tool runs nothing):")
            print("  " + mm_fix_command(usb_ids(udev_props(device_path(a.device)))))
            print("undo with:")
            print("  " + mm_undo_command())
    elif a.undo:
        if os.path.exists(MM_RULES):
            print("run this yourself (needs sudo; this tool runs nothing):")
            print("  " + mm_undo_command())
        else:
            print(f"nothing to undo ({MM_RULES} does not exist)")
    return rc


# ---------------------------------------------------------------------------
# --guide: the Pixhawk-side setup as ordered rows
# ---------------------------------------------------------------------------

FCU_DEFAULTS = ("192.168.0.3", "192.168.0.4")      # PX4's own Ethernet defaults (net.cfg not read)
CMD_WORDS = ("python3", "sudo", "printf", "git", "ip", "nmcli", "ros2", "ls")
NET_CFG = ["DEVICE=eth0", "BOOTPROTO=fallback", "IPADDR=10.41.10.2", "NETMASK=255.255.255.0",
           "ROUTER=10.41.10.254", "DNS=10.41.10.254"]


def step_row(row, step, kind=None):
    """Tag a check row with its step. fix is a command only: a hint moves to ``manual``."""
    row = dict(row, step=step)
    if kind:
        row["kind"] = kind
    fix = row.get("fix", "")
    if fix and fix.split()[0] not in CMD_WORDS:
        row["manual"] = row.get("manual", []) + [fix]
        row["fix"] = ""
    row.setdefault("kind", "check")
    return row


def manual_row(name, step, lines, confirmed_by=(), literal=None, detail=""):
    row = {"name": name, "status": INFO, "detail": detail or "manual step", "fix": "", "why": "",
           "kind": "manual", "step": step, "manual": list(lines), "confirmed_by": list(confirmed_by)}
    if literal:
        row["literal"] = literal
    return row


def read_arp(iface, path="/proc/net/arp"):
    """IPs with a complete neighbour entry on ``iface`` (kernel ARP table; read-only)."""
    try:
        lines = open(path).read().splitlines()[1:]
    except OSError:
        return []
    out = []
    for ln in lines:
        f = ln.split()
        if len(f) >= 6 and f[5] == iface and f[2] == "0x2":
            out.append(f[0])
    return out


def arping(iface, ip):
    """True / False (no answer) / None (cannot run: no arping, or no CAP_NET_RAW)."""
    try:
        p = subprocess.run(["arping", "-I", iface, "-c", "1", "-w", "1", ip],
                           capture_output=True, text=True, timeout=4)
    except (OSError, subprocess.TimeoutExpired):
        return None
    err = (p.stderr + p.stdout).lower()
    if "not permitted" in err or "permission" in err or "cap_net_raw" in err or "socket" in err:
        return None
    return p.returncode == 0


FCU_ADDR_MANUAL = [
    "In QGC go to Analyze Tools > MAVLink Console and run ifconfig. 192.168.0.3 means net.cfg was not "
    "read; 0.0.0.0 means the BOOTPROTO=fallback DHCP attempt is still pending, which takes about a "
    "minute after boot.",
    "Recheck net.cfg against the block above (" + ", ".join(NET_CFG) + ").",
    "Reboot and wait about a minute.",
]


def fcu_address_row(a, ping_ok, carrier, arp=read_arp, probe=arping):
    """Where is the FCU when --fcu-ip does not answer? Looks on ``a.iface`` only; read-only."""
    def row(status, detail):
        return dict(chk("fcu-address", status, detail), kind="manual", step=2, manual=list(FCU_ADDR_MANUAL),
                    confirmed_by=["fcu"])
    if ping_ok:
        return row(PASS, f"FCU answers at {a.fcu_ip}")
    if not carrier:
        return row(INFO, f"no carrier on {a.iface}; nothing to look for")
    found = [ip for ip in arp(a.iface) if ip != a.host_ip]
    cannot = False
    for ip in (a.fcu_ip,) + FCU_DEFAULTS:
        r = probe(a.iface, ip)
        if r is None:
            cannot = True
            break
        if r and ip not in found:
            found.append(ip)
    if a.fcu_ip in found and len(found) == 1:
        return row(WARN, f"{a.fcu_ip} answers ARP on {a.iface} but not ping")
    others = [ip for ip in found if ip != a.fcu_ip]
    if others:
        return row(FAIL, f"FCU answers at {others[0]}, expected {a.fcu_ip}")
    if cannot:
        return row(INFO, f"nothing in the ARP table of {a.iface}; arping unavailable "
                         "(needs arping and CAP_NET_RAW, which console's privileged container has)")
    return row(INFO, f"nothing answers on {a.iface} at {a.fcu_ip} or {' / '.join(FCU_DEFAULTS)}")


def xrce_session_row(a, topics, agent_running, log_text=""):
    """topics: list from ``ros2 topic list`` or None when ROS is not usable here."""
    if topics and any(t.startswith("/fmu/out/") for t in topics):
        return chk("xrce-session", PASS, "ros2 topic list shows /fmu/out/*: the agent has a client")
    if log_text and any(w in log_text.lower() for w in ("session established", "create_client", "client created")):
        return chk("xrce-session", PASS, "the agent log shows a client session")
    if topics is None and not agent_running:
        return chk("xrce-session", INFO, "ROS and the MicroXRCEAgent are not running here")
    if not agent_running:
        return chk("xrce-session", INFO, "no MicroXRCEAgent running here, so no client can be seen",
                   why="start it with the PX4 stack")
    if topics is None:
        return chk("xrce-session", INFO, "MicroXRCEAgent runs but ros2 is not usable here to list topics "
                   "(pass --agent-log with the agent's verbose output)")
    return chk("xrce-session", FAIL, "MicroXRCEAgent runs but no client session / no /fmu/out/* topics",
               why="the FCU is not talking uXRCE-DDS to the agent (UXRCE_DDS_* params, reboot, link)")


def ros_topics():
    rc, out = sh(["ros2", "topic", "list"], timeout=8)
    return out.splitlines() if rc == 0 else None


def agent_running():
    return "MicroXRCEAgent" in sh(["ps", "-eo", "args="])[1]


def firmware_guide_row(a, topics):
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import firmware
        return firmware.firmware_row(firmware.Host(), firmware.load_pairing(), device=device_path(a.device),
                                     probe=not a.no_probe, topics=topics)
    except Exception as e:  # noqa: BLE001 - a missing pairing file must not break the guide
        return chk("firmware", INFO, f"cannot run the firmware.py version check ({e})")


def guide_rows(a):
    base = {r["name"]: r for r in run_checks(a)}
    topics = ros_topics()
    log = ""
    if a.agent_log:
        try:
            log = open(a.agent_log).read()[-20000:]
        except OSError:
            pass
    carrier = base["interface"]["status"] == PASS
    usb = next(r for r in run_usb_checks(a) if r["name"] == "usb_device")
    rows = [
        manual_row("companion-address", 1,
                   [f"Give the companion's wired interface (--iface {a.iface}) the static address "
                    f"{a.host_ip}/{a.prefix}, no default route."],
                   ["interface", "address", "profile"]),
        step_row(base["interface"], 1), step_row(base["address"], 1), step_row(base["profile"], 1),
        manual_row("fcu-netcfg", 2,
                   ["On the Pixhawk's SD card create/edit etc/net.cfg with this content, then reboot the Pixhawk."],
                   ["fcu"], literal={"name": "net.cfg", "text": "\n".join(NET_CFG) + "\n"}),
        step_row(base["fcu"], 2),
        fcu_address_row(a, base["fcu"]["status"] == PASS, carrier),
        manual_row("dds-cfg", 3,
                   ["In QGC (Vehicle Setup > Parameters) set UXRCE_DDS_CFG = Ethernet, then reboot the Pixhawk.",
                    "UXRCE_DDS_PRT and UXRCE_DDS_AG_IP only appear after this reboot."],
                   ["xrce-session"]),
        step_row(xrce_session_row(a, topics, agent_running(), log), 3),
        manual_row("dds-params", 4,
                   ["In QGC set UXRCE_DDS_PRT = 8888 and UXRCE_DDS_AG_IP = 170461697 (= " + a.host_ip + "), "
                    "then reboot the Pixhawk.",
                    "Start the agent (MicroXRCEAgent udp4 -p 8888) before or after the reboot."],
                   ["xrce-session"]),
        step_row(dict(usb, name="usb"), 5),
        step_row(firmware_guide_row(a, topics), 5),
    ]
    return rows


def guide_main(a):
    rows = guide_rows(a)
    if a.json:
        json.dump(rows, sys.stdout, indent=2)
        print()
    else:
        for r in rows:
            print(f"step {r['step']} [{r['status']:<4}] {r['name']:<18} {r['detail']}")
            for ln in r.get("manual", []):
                print(f"         - {ln}")
            if r.get("literal"):
                print("         " + r["literal"]["name"] + ":\n           " + r["literal"]["text"].replace("\n", "\n           "))
            if r["status"] in (FAIL, WARN) and r.get("fix"):
                print(f"         fix: {r['fix']}")
    return 1 if any(r["status"] == FAIL for r in rows) else 0


def report(checks, as_json):
    if as_json:
        json.dump(checks, sys.stdout, indent=2)
        print()
        return
    for c in checks:
        print(f"[{c['status']:<4}] {c['name']:<12} {c['detail']}")
        if c["status"] in (FAIL, WARN) and c["fix"]:
            print(f"         fix: {c['fix']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--iface", help="wired interface facing the Pixhawk (required unless --usb)")
    ap.add_argument("--usb", action="store_true",
                    help="check the Pixhawk's USB port for MAVROS instead of the Ethernet link")
    ap.add_argument("--device", default=USB_DEVICE,
                    help="--usb: device path or MAVROS fcu_url (path:baud); default %(default)s")
    ap.add_argument("--host-ip", default="10.41.10.1")
    ap.add_argument("--fcu-ip", default="10.41.10.2")
    ap.add_argument("--prefix", type=int, default=24)
    ap.add_argument("--con-name", default=None, help="NetworkManager profile (default console-px4-<iface>)")
    ap.add_argument("--guide", action="store_true", help="the Pixhawk-side setup as ordered rows (read-only)")
    ap.add_argument("--no-probe", action="store_true", help="--guide: do not open the serial port for the PX4 version")
    ap.add_argument("--agent-log", help="--guide: MicroXRCEAgent log (verbose) to look for a client session in")
    ap.add_argument("--json", action="store_true", help="machine-readable report (read-only)")
    ap.add_argument("-n", "--dry-run", action="store_true", help="print commands, change nothing")
    ap.add_argument("--apply", action="store_true", help="configure the link (sudo per command)")
    ap.add_argument("--undo", action="store_true", help="remove what --apply created")
    a = ap.parse_args(argv)
    if a.apply and a.undo:
        ap.error("--apply and --undo are exclusive")
    if a.json and (a.apply or a.undo) and not (a.dry_run or a.usb):
        ap.error("--json reports only; combine with --apply/--undo only together with --dry-run")
    if a.guide:
        if a.apply or a.undo or a.usb:
            ap.error("--guide only reports; it excludes --apply/--undo/--usb")
        if not a.iface:
            ap.error("--guide needs --iface")
        a.con_name = a.con_name or f"console-px4-{a.iface}"
        return guide_main(a)
    if a.usb:
        if a.json and (a.apply or a.undo):
            ap.error("--json reports only; run --usb --apply/--undo without --json")
        return usb_main(a)
    if not a.iface:
        ap.error("--iface is required (or --usb for the USB port)")
    a.con_name = a.con_name or f"console-px4-{a.iface}"

    checks = run_checks(a)
    if a.json:
        json.dump(checks, sys.stdout, indent=2)
        print()
    else:
        for c in checks:
            print(f"[{c['status']:<4}] {c['name']:<9} {c['detail']}")
            if c["status"] in (FAIL, WARN) and c["fix"]:
                print(f"         fix: {c['fix']}")

    rc = 1 if any(c["status"] == FAIL for c in checks) else 0
    if not (a.apply or a.undo):
        return rc
    cmds = apply_plan(a) if a.apply else undo_plan(a)
    if cmds is None:
        print(f"refusing: profile {a.con_name} was not created by this tool", file=sys.stderr)
        return 1
    if not cmds:
        print("nothing to do" if a.apply else "nothing to undo")
        return rc
    if a.dry_run:
        execute(cmds, True)
        if a.apply:
            nm = cmds[0][0] == "nmcli"
            print("undo with: " + ("sudo nmcli con delete " + a.con_name if nm else
                                   f"sudo ip addr del {a.host_ip}/{a.prefix} dev {a.iface}"))
        return rc
    print("will run, each under sudo:")
    execute(cmds, True)
    if input("proceed? [y/N] ").strip().lower() != "y":
        return rc
    return execute(cmds, False)


if __name__ == "__main__":
    sys.exit(main())
