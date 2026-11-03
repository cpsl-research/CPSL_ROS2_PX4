#!/usr/bin/env python3
"""Give this host a static address on the direct Ethernet link to a Pixhawk (uXRCE-DDS).

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

Python stdlib only.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys

MARK = "console-px4"          # connection.stable-id on every profile this tool creates
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--iface", required=True, help="wired interface facing the Pixhawk")
    ap.add_argument("--host-ip", default="10.41.10.1")
    ap.add_argument("--fcu-ip", default="10.41.10.2")
    ap.add_argument("--prefix", type=int, default=24)
    ap.add_argument("--con-name", default=None, help="NetworkManager profile (default console-px4-<iface>)")
    ap.add_argument("--json", action="store_true", help="machine-readable report (read-only)")
    ap.add_argument("-n", "--dry-run", action="store_true", help="print commands, change nothing")
    ap.add_argument("--apply", action="store_true", help="configure the link (sudo per command)")
    ap.add_argument("--undo", action="store_true", help="remove what --apply created")
    a = ap.parse_args(argv)
    a.con_name = a.con_name or f"console-px4-{a.iface}"
    if a.json and (a.apply or a.undo) and not a.dry_run:
        ap.error("--json reports only; combine with --apply/--undo only together with --dry-run")
    if a.apply and a.undo:
        ap.error("--apply and --undo are exclusive")

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
