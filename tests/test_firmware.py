"""Pure-logic tests for scripts/firmware.py (fakes only: no board, no docker, no serial port)."""
import argparse
import json
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import firmware as fw  # noqa: E402

P = fw.load_pairing()
GOOD = {"ID_VENDOR_ID": "3185", "ID_MODEL_ID": "0035", "ID_MODEL": "PX4_FMU_v6X.x", "ID_SERIAL_SHORT": "0"}
BL = dict(GOOD, ID_MODEL="PX4_BL_FMU_v6X.x")


class FakeHost(fw.Host):
    def __init__(self, entries=None, ps="", busy=False, pyserial=True, mm=None, heads=None, version=None):
        self.entries = {"usb-Auterion_PX4_FMU_v6X.x_0-if00": GOOD} if entries is None else entries
        self._ps, self.busy, self.pyserial, self.version = ps, busy, pyserial, version
        self.heads = heads or {}
        self.mm = mm or fw.chk("modemmanager", fw.PASS, "not running")
        self.probed = False

    def by_id(self):
        return [(n, f"/dev/serial/by-id/{n}") for n in self.entries]

    def udev(self, path):
        return self.entries[path.rsplit("/", 1)[1]]

    def ps(self):
        return self._ps

    def port_busy(self, path):
        return (True, "123") if self.busy else (False, "")

    def have_pyserial(self):
        return self.pyserial

    def git_head(self, repo):
        return self.heads.get(Path(repo).name)

    def modemmanager_row(self, device):
        return self.mm

    def probe_version(self, path, timeout=5.0):
        self.probed = True
        return self.version or (None, "pymavlink not installed")


@pytest.fixture(autouse=True)
def uploader(monkeypatch, tmp_path):
    root = tmp_path / "root"
    (root / "firmware/PX4-Autopilot/Tools").mkdir(parents=True)
    (root / "firmware/PX4-Autopilot/Tools/px_uploader.py").write_text("")
    monkeypatch.setattr(fw, "ROOT", root)


def make_artifact(tmp_path, board_id=53, record=True, commit=None):
    art = tmp_path / "out" / "px4_fmu-v6x_default" / "px4_fmu-v6x_default.px4"
    art.parent.mkdir(parents=True)
    art.write_text(json.dumps({"board_id": board_id, "image": "x"}))
    if record:
        Path(str(art) + fw.RECORD).write_text(json.dumps({"sha256": fw.sha256(art), "commit": commit or P["firmware"]["commit"]}))
    return art


def make_backup(tmp_path, serial="0", age=0, n=2):
    f = tmp_path / "bk.params"
    f.write_text("".join(f"1\t1\tP{i}\t{i}\t6\n" for i in range(n)))
    Path(str(f) + fw.META).write_text(json.dumps({"time": time.time() - age, "sha256": fw.sha256(f), "n_params": n, "serial": serial}))
    return f


def args(tmp_path, **kw):
    d = dict(artifact=None, out=str(tmp_path / "out"), backup=None, backup_max_age=3600.0, confirm_serial="0",
             device="/dev/x", no_probe=False)
    d.update(kw)
    return argparse.Namespace(**d)


def ready(tmp_path, **kw):
    make_artifact(tmp_path)
    return args(tmp_path, backup=str(make_backup(tmp_path)), **kw)


def refused(rows):
    return {r["name"] for r in rows if r["status"] == fw.FAIL}


# --- pairing -----------------------------------------------------------------

def test_real_pairing_file():
    assert P["firmware"]["tag"] == "v1.15.4" and P["firmware"]["commit"].startswith("99c40407ff")
    assert P["px4_msgs"]["commit"].startswith("a1045ec")
    assert P["board"]["board_id"] == 53 and (P["board"]["usb"]["vid"], P["board"]["usb"]["pid"]) == ("3185", "0035")
    assert len(P["submodules"]) == 7


def test_pairing_missing_key(tmp_path):
    f = tmp_path / "v.json"
    f.write_text(json.dumps({"firmware": {"tag": "v1"}}))
    with pytest.raises(ValueError):
        fw.load_pairing(f)
    with pytest.raises(ValueError):
        fw.load_pairing(tmp_path / "nope.json")


def test_tag_tuple():
    assert fw.tag_tuple("v1.15.4") == (1, 15, 4) and fw.tag_tuple("main") is None


# --- board identification ----------------------------------------------------

def test_app_vs_bootloader_same_vidpid():
    assert fw.classify("PX4_FMU_v6X.x", P) == "app"
    assert fw.classify("PX4 BL FMU v6X.x", P) == "bootloader"
    assert fw.classify("PX4_BL_FMU_v6X.x", P) == "bootloader"
    assert fw.classify("FTDI_Thing", P) == "other"


def test_scan_ignores_other_vidpid():
    h = FakeHost({"usb-other": {"ID_VENDOR_ID": "0403", "ID_MODEL_ID": "6001"},
                  "usb-Auterion_PX4_FMU_v6X.x_0-if00": GOOD})
    boards, seen = fw.scan_boards(h, P)
    assert len(boards) == 1 and boards[0]["mode"] == "app" and boards[0]["serial"] == "0" and len(seen) == 2


def test_scan_without_udev_props_is_not_a_board():
    boards, _ = fw.scan_boards(FakeHost({"usb-Auterion_PX4_FMU_v6X.x_0-if00": {}}), P)
    assert boards == []


# --- params ------------------------------------------------------------------

QGC = "# Vehicle-Id Component-Id Name Value Type\n1\t1\tA\t1.5\t9\n1\t1\tB\t2\t6\n1\t1\tC\t3\t6\n"


def test_parse_params_skips_comments():
    assert fw.parse_params(QGC) == {"A": 1.5, "B": 2.0, "C": 3.0}


def test_diff_by_name_and_value_not_text():
    a = fw.parse_params(QGC)
    b = fw.parse_params("1\t1\tA\t1.5000000000000001\t9\n1\t1\tB\t5\t6\n1\t1\tD\t1\t6\n")
    d = fw.diff_params(a, b)
    assert d["changed"] == {"B": (2.0, 5.0)} and d["only_in_vehicle"] == ["C"] and d["only_in_reference"] == ["D"] and d["same"] == 1
    assert "1 differ" in fw.diff_summary(d)


# --- helpers -----------------------------------------------------------------

def test_find_holders():
    ps = "10 /opt/x/MicroXRCEAgent udp4 -p 8888\n11 python3 /ws/install/mavros_node --a\n12 /bin/vim MicroXRCEAgent.txt\n" \
         "13 /usr/bin/QGroundControl-x86_64.AppImage\n14 /usr/bin/python3 scripts/firmware.py --check\n"
    assert fw.find_holders(ps, own_pid=14) == ["MicroXRCEAgent", "mavros_node"]
    assert fw.find_holders("1 bash\n") == []


def test_artifact_states(tmp_path):
    assert fw.artifact_state(tmp_path / "nope", P)[0] == "absent"
    art = make_artifact(tmp_path)
    assert fw.artifact_state(art, P)[0] == "ok"
    art.write_text(json.dumps({"board_id": 53, "image": "changed"}))
    assert fw.artifact_state(art, P)[0] == "mismatch"


def test_build_commands_print_only(tmp_path):
    cmds = fw.build_commands(P, tmp_path / "o", tmp_path / "c")
    docker = next(c for c in cmds if c.startswith("docker run"))
    assert "px4io/px4-dev-nuttx-focal:2022-08-12" in docker and docker.endswith("make px4_fmu-v6x_default")
    assert any("submodule update --init --depth 1" in c for c in cmds)
    assert "--record" in cmds[-1]


def test_build_fetches_nuttx_tag_after_submodules_before_docker(tmp_path):
    cmds = fw.build_commands(P, tmp_path / "o", tmp_path / "c")
    tag = P["nuttx"]["tag"]
    i = next(i for i, c in enumerate(cmds) if "fetch --depth 1 --no-tags origin" in c)
    assert f"refs/tags/{tag}:refs/tags/{tag}" in cmds[i] and P["nuttx"]["path"] in cmds[i]
    assert "submodule update" in cmds[i - 1] and cmds[i + 1].startswith("docker run")
    P2 = {k: v for k, v in P.items() if k != "nuttx"}
    assert not any(" fetch --depth" in c for c in fw.build_commands(P2, tmp_path / "o", tmp_path / "c"))


# --- flash guards ------------------------------------------------------------

def test_all_guards_pass(tmp_path):
    a = ready(tmp_path)
    rows, board, _ = fw.flash_guards(FakeHost(), P, a)
    assert refused(rows) == set() and board["serial"] == "0"


def test_refuse_no_backup(tmp_path):
    make_artifact(tmp_path)
    rows, _, _ = fw.flash_guards(FakeHost(), P, args(tmp_path))
    assert refused(rows) == {"backup"}


@pytest.mark.parametrize("kw", [dict(age=7200), dict(serial="9"), dict(n=0)])
def test_refuse_bad_backup(tmp_path, kw):
    make_artifact(tmp_path)
    a = args(tmp_path, backup=str(make_backup(tmp_path, **kw)))
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"backup"}


def test_refuse_backup_edited_after(tmp_path):
    a = ready(tmp_path)
    Path(a.backup).write_text(Path(a.backup).read_text() + "1\t1\tZ\t1\t6\n")
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"backup"}


def test_refuse_driver_running(tmp_path):
    a = ready(tmp_path)
    rows, _, _ = fw.flash_guards(FakeHost(ps="9 MicroXRCEAgent udp4\n"), P, a)
    assert refused(rows) == {"driver_stopped"}


def test_refuse_wrong_vidpid_or_absent(tmp_path):
    a = ready(tmp_path)
    h = FakeHost({"usb-x": {"ID_VENDOR_ID": "26ac", "ID_MODEL_ID": "0011", "ID_MODEL": "PX4"}})
    assert {"identity", "serial_confirm", "backup", "port_free"} <= refused(fw.flash_guards(h, P, a)[0])


def test_refuse_bootloader_mode(tmp_path):
    a = ready(tmp_path)
    rows, _, _ = fw.flash_guards(FakeHost({"usb-Auterion_PX4_BL_FMU_v6X.x_0-if00": BL}), P, a)
    assert "identity" in refused(rows)


def test_refuse_two_boards(tmp_path):
    a = ready(tmp_path)
    h = FakeHost({"usb-a": GOOD, "usb-b": GOOD})
    assert "identity" in refused(fw.flash_guards(h, P, a)[0])


def test_refuse_hash_mismatch_and_wrong_board_and_stale(tmp_path):
    a = ready(tmp_path)
    art = Path(a.out) / "px4_fmu-v6x_default" / "px4_fmu-v6x_default.px4"
    art.write_text(json.dumps({"board_id": 53, "image": "tampered"}))
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"artifact"}
    art.write_text(json.dumps({"board_id": 50}))
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"artifact"}


def test_refuse_unrecorded_and_stale_commit(tmp_path):
    a = args(tmp_path, backup=str(make_backup(tmp_path)))
    make_artifact(tmp_path, record=False)
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"artifact"}
    (tmp_path / "out" / "px4_fmu-v6x_default" / "px4_fmu-v6x_default.px4").unlink()
    make_artifact_dir = tmp_path / "out"
    import shutil
    shutil.rmtree(make_artifact_dir)
    make_artifact(tmp_path, commit="0" * 40)
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"artifact"}


def test_refuse_busy_port(tmp_path):
    a = ready(tmp_path)
    assert refused(fw.flash_guards(FakeHost(busy=True), P, a)[0]) == {"port_free"}


@pytest.mark.parametrize("typed", [None, "1", ""])
def test_refuse_missing_or_wrong_serial(tmp_path, typed):
    a = ready(tmp_path, confirm_serial=typed)
    assert refused(fw.flash_guards(FakeHost(), P, a)[0]) == {"serial_confirm"}


def test_refuse_no_pyserial(tmp_path):
    a = ready(tmp_path)
    assert refused(fw.flash_guards(FakeHost(pyserial=False), P, a)[0]) == {"uploader"}


def test_modemmanager_is_only_a_warning(tmp_path):
    a = ready(tmp_path)
    rows, _, _ = fw.flash_guards(FakeHost(mm=fw.chk("modemmanager", fw.WARN, "probes", "fix")), P, a)
    assert refused(rows) == set()


def test_flash_main_dry_run_runs_nothing(tmp_path, monkeypatch, capsys):
    called = []
    monkeypatch.setattr(fw.subprocess, "call", lambda *a, **k: called.append(a))
    a = ready(tmp_path, dry_run=True, apply=False, json=False)
    assert fw.flash_main(FakeHost(), P, a) == 0 and not called
    assert "px_uploader.py" in capsys.readouterr().out


def test_flash_main_refused_never_calls_uploader(tmp_path, monkeypatch):
    called = []
    monkeypatch.setattr(fw.subprocess, "call", lambda *a, **k: called.append(a))
    a = args(tmp_path, dry_run=False, apply=True, json=False)
    assert fw.flash_main(FakeHost(), P, a) == 1 and not called


def test_flash_main_apply_calls_uploader_once(tmp_path, monkeypatch):
    called = []
    monkeypatch.setattr(fw.subprocess, "call", lambda cmd, **k: called.append(cmd) or 0)
    a = ready(tmp_path, dry_run=False, apply=True, json=False)
    assert fw.flash_main(FakeHost(), P, a) == 0 and len(called) == 1
    assert called[0][-1].endswith("px4_fmu-v6x_default.px4") and called[0][1].endswith("Tools/px_uploader.py")


# --- check -------------------------------------------------------------------

def test_check_rows_shape_and_no_probe(tmp_path):
    h = FakeHost(heads={"PX4-Autopilot": P["firmware"]["commit"], "px4_msgs": P["px4_msgs"]["commit"]})
    rows = fw.run_checks(h, P, args(tmp_path, no_probe=True))
    assert [r["name"] for r in rows] == ["board", "pins", "current_version", "artifact", "uploader", "modemmanager"]
    assert all(set(r) >= {"name", "status", "detail", "fix", "why"} for r in rows)
    assert not h.probed
    assert rows[0]["status"] == fw.PASS and rows[1]["status"] == fw.PASS and rows[2]["status"] == fw.INFO


def test_check_version_pass_and_warn(tmp_path):
    sw = lambda a, b, c: (a << 24) | (b << 16) | (c << 8)
    ok = fw.run_checks(FakeHost(version=(sw(1, 15, 4), bytes.fromhex("99c40407ff000000"))), P, args(tmp_path))
    assert ok[2]["status"] == fw.PASS
    old = fw.run_checks(FakeHost(version=(sw(1, 17, 0), b"")), P, args(tmp_path))
    assert old[2]["status"] == fw.WARN and "v1.17.0" in old[2]["detail"]


def test_check_busy_port_not_probed(tmp_path):
    h = FakeHost(busy=True)
    rows = fw.run_checks(h, P, args(tmp_path))
    assert rows[2]["status"] == fw.INFO and not h.probed


def test_check_no_board_fails(tmp_path):
    rows = fw.run_checks(FakeHost({}), P, args(tmp_path, no_probe=True))
    assert rows[0]["status"] == fw.FAIL


def test_pins_mismatch_warns(tmp_path):
    rows = fw.run_checks(FakeHost(heads={"px4_msgs": "b" * 40}), P, args(tmp_path, no_probe=True))
    assert rows[1]["status"] == fw.WARN


def test_no_param_write_calls_in_tool():
    src = (Path(fw.__file__)).read_text()
    for bad in ("PARAM_SET", "param set", "param load", "param_set"):
        assert bad not in src


# --- --backup-params orchestration (process spawning faked) ---------------------
class FakeProc:
    pid = 4242

    def __init__(self, dead=False):
        self.dead = dead

    def poll(self):
        return 1 if self.dead else None


DUMP = "/fw_backup/mavros/param:\n  ros__parameters:\n    SYS_AUTOSTART: 4001\n    COM_OF_LOSS_T: 1.5\n    use_sim_time: false\n"
PULL = "response:\nmavros_msgs.srv.ParamPull_Response(success=True, param_received=2)\n"


class BackupHost(FakeHost):
    def __init__(self, state=(0, "true\n"), pull=(0, PULL), dump=(0, DUMP), dead=False, stop_ok=True, free_after=True, **kw):
        super().__init__(**kw)
        self.state, self.pull, self.dump = state, pull, dump
        self.dead, self.stop_ok, self.free_after = dead, stop_ok, free_after
        self.cmds, self.spawned, self.stopped, self.env = [], [], False, None

    def ros_env(self, domain):
        self.env = {"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"}
        return self.env

    def ros(self, cmd, env, timeout):
        self.cmds.append(cmd)
        return {"echo": self.state, "call": self.pull, "dump": self.dump}[
            "echo" if cmd[1] == "topic" else "call" if cmd[1] == "service" else "dump"]

    def spawn(self, cmd, env, logfile):
        self.spawned.append(cmd)
        return FakeProc(self.dead)

    def stop(self, proc, grace=10.0):
        self.stopped = True
        return self.stop_ok

    def wait_port_free(self, path, timeout=10.0):
        return (True, "") if self.free_after else (False, "99")

    def sleep(self, s):
        pass


def run_backup(tmp_path, h, **kw):
    return fw.do_backup(h, P, tmp_path / "bk.params", connect_timeout=kw.pop("connect_timeout", 3.0), **kw)


def test_backup_ok_writes_file_and_sidecar(tmp_path):
    h = BackupHost()
    meta, err = run_backup(tmp_path, h, domain=77)
    assert err == "" and meta["n_params"] == 2 and meta["serial"] == "0"
    assert fw.parse_params((tmp_path / "bk.params").read_text()) == {"SYS_AUTOSTART": 4001.0, "COM_OF_LOSS_T": 1.5}
    assert (tmp_path / ("bk.params" + fw.META)).is_file() and h.stopped
    assert h.env == {"ROS_DOMAIN_ID": "77", "ROS_LOCALHOST_ONLY": "1"}
    assert fw.backup_state(str(tmp_path / "bk.params"), {"serial": "0"}, 3600)[0]


def test_backup_node_isolated_and_allowlisted(tmp_path):
    y = fw.backup_params_yaml("/dev/x:115200")
    assert "- param" in y and "- command" not in y and "- setpoint_raw" not in y and "tf.send: false" in y
    node = fw.backup_commands("/dev/x:115200", "c.yaml")["node"]
    assert "__ns:=/fw_backup" in node and node[-2:] == ["--params-file", "c.yaml"]


def test_backup_never_writes(tmp_path):
    h = BackupHost()
    run_backup(tmp_path, h)
    flat = [" ".join(c) for c in h.cmds + h.spawned]
    for bad in ("param set", "param load", "param/set", "ParamSet", "ParamPush", "param/push", "arming", "set_mode",
                "setpoint", "topic pub", "CommandLong", "cmd/"):
        assert not any(bad in c for c in flat), bad
    assert any("param/pull" in c for c in flat) and any(c.startswith("ros2 param dump") for c in flat)
    text = Path(fw.__file__).read_text()
    assert "PARAM_SET" not in text and "param set" not in text and "param load" not in text


@pytest.mark.parametrize("kw,needle", [
    (dict(state=(1, "")), "did not connect"),
    (dict(dead=True), "exited"),
    (dict(pull=(0, "success=False, param_received=0")), "param/pull failed"),
    (dict(pull=(None, "TimeoutExpired")), "param/pull failed"),
    (dict(dump=(1, "boom")), "reading the parameters failed"),
    (dict(dump=(0, "nothing here")), "no parameters"),
])
def test_backup_errors_still_clean_up(tmp_path, kw, needle):
    h = BackupHost(**kw)
    meta, err = run_backup(tmp_path, h, connect_timeout=0.05)
    assert meta is None and needle in err
    assert h.stopped
    assert not (tmp_path / "bk.params").exists() and not (tmp_path / ("bk.params" + fw.META)).exists()


def test_backup_cleans_up_on_exception(tmp_path):
    class Boom(BackupHost):
        def ros(self, cmd, env, timeout):
            raise RuntimeError("kaboom")
    h = Boom()
    meta, err = run_backup(tmp_path, h)
    assert meta is None and "kaboom" in err and h.stopped


@pytest.mark.parametrize("kw", [dict(stop_ok=False), dict(free_after=False)])
def test_backup_reports_failed_cleanup(tmp_path, kw):
    meta, err = run_backup(tmp_path, BackupHost(**kw))
    assert meta is None and "CLEANUP FAILED" in err


def test_backup_refuses_busy_port_without_spawning(tmp_path):
    h = BackupHost(busy=True)
    meta, err = run_backup(tmp_path, h)
    assert meta is None and "held by" in err and h.spawned == [] and not h.stopped


def test_backup_refuses_unknown_port_state(tmp_path):
    h = BackupHost()
    h.port_busy = lambda path: (None, "no fuser")
    meta, err = run_backup(tmp_path, h)
    assert meta is None and "cannot tell" in err and h.spawned == []


def test_backup_refuses_running_mavros_but_not_xrce(tmp_path):
    h = BackupHost(ps="10 /opt/ros/jazzy/lib/mavros/mavros_node --ros-args\n")
    assert "mavros_node is already running" in run_backup(tmp_path, h)[1] and h.spawned == []
    h = BackupHost(ps="11 MicroXRCEAgent udp4 -p 8888\n12 python3 px4_control_node\n")
    assert run_backup(tmp_path, h)[1] == ""


def test_backup_refuses_existing_file_and_missing_board(tmp_path):
    (tmp_path / "bk.params").write_text("x")
    h = BackupHost()
    assert "never overwritten" in run_backup(tmp_path, h)[1] and h.spawned == []
    (tmp_path / "bk.params").unlink()
    h = BackupHost(entries={})
    assert "exactly one" in run_backup(tmp_path, h)[1] and h.spawned == []


# ---------------------------------------------------------------------------
# identity without udevadm (sysfs): the console container has none
# ---------------------------------------------------------------------------

def fake_sysfs(tmp, vid="3185", pid="0035", product="PX4 FMU v6X.x", serial="0", tty="ttyACM0",
               name="usb-Auterion_PX4_FMU_v6X.x_0-if00"):
    """/dev/serial/by-id/<name> -> ttyACM0 -> /sys/class/tty/ttyACM0/device -> usb interface dir."""
    sysd = tmp / "sys"
    usb = sysd / "devices/pci0000:00/0000:00:14.0/usb1/1-2"
    iface = usb / "1-2:1.0"
    ttyd = iface / "tty" / tty
    ttyd.mkdir(parents=True)
    for k, v in (("idVendor", vid), ("idProduct", pid), ("product", product), ("serial", serial)):
        if v is not None:
            (usb / k).write_text(v + "\n")
    (ttyd / "device").symlink_to(iface)
    (sysd / "class/tty").mkdir(parents=True)
    (sysd / "class/tty" / tty).symlink_to(ttyd)
    dev = tmp / "dev"
    (dev / "serial/by-id").mkdir(parents=True)
    (dev / tty).write_text("")
    link = dev / "serial/by-id" / name
    link.symlink_to(Path("../..") / tty)
    return sysd, link


def test_sysfs_props_for_this_board(tmp_path):
    import host_setup
    sysd, link = fake_sysfs(tmp_path)
    assert host_setup.sysfs_usb_props(str(link), sysfs=str(sysd)) == {
        "ID_VENDOR_ID": "3185", "ID_MODEL_ID": "0035", "ID_MODEL": "PX4 FMU v6X.x", "ID_SERIAL_SHORT": "0"}
    assert host_setup.sysfs_usb_props(str(tmp_path / "dev/missing"), sysfs=str(sysd)) == {}


class NoUdevadm(fw.Host):
    def __init__(self, by_id, sysd):
        self._by, self._sys = by_id, sysd

    def run(self, cmd, timeout=5):
        return None, ""  # no udevadm installed

    def by_id(self):
        return [(p.name, str(p)) for p in self._by]

    def sysfs(self, path):
        import host_setup
        return host_setup.sysfs_usb_props(path, sysfs=str(self._sys))


def test_scan_finds_board_with_no_udevadm(tmp_path):
    sysd, link = fake_sysfs(tmp_path)
    found, seen = fw.scan_boards(NoUdevadm([link], sysd), P)
    assert len(found) == 1, "regression: 'need exactly one 3185:0035 device (found 0)' in the container"
    assert (found[0]["vid"], found[0]["pid"], found[0]["serial"], found[0]["mode"]) == ("3185", "0035", "0", "app")


def test_scan_bootloader_by_product_with_no_udevadm(tmp_path):
    sysd, link = fake_sysfs(tmp_path, product="PX4 BL FMU v6X.x", name="usb-Auterion_PX4_BL_FMU_v6X.x_0-if00")
    found, _ = fw.scan_boards(NoUdevadm([link], sysd), P)
    assert [f["mode"] for f in found] == ["bootloader"]


def test_scan_ignores_other_usb_device_with_no_udevadm(tmp_path):
    sysd, link = fake_sysfs(tmp_path, vid="0403", pid="6001", product="FT232R", name="usb-FTDI_FT232R-if00")
    assert fw.scan_boards(NoUdevadm([link], sysd), P)[0] == []


def test_udevadm_is_an_extra_not_required(tmp_path):
    sysd, link = fake_sysfs(tmp_path)

    class WithUdev(NoUdevadm):
        def run(self, cmd, timeout=5):
            return 0, "ID_MODEL=PX4_FMU_v6X.x\nID_VENDOR_ID=3185\nID_MODEL_ID=0035\nID_MM_DEVICE_IGNORE=1"
    props = WithUdev([link], sysd).udev(str(link))
    assert props["ID_MM_DEVICE_IGNORE"] == "1" and props["ID_SERIAL_SHORT"] == "0"


def test_probe_reports_how_to_install_pymavlink(monkeypatch):
    monkeypatch.setitem(sys.modules, "pymavlink", None)  # import raises ImportError
    rc, why = fw.Host().probe_version("/dev/null")
    assert rc is None and "pymavlink not installed" in why and "deps/px4.sh" in why


def test_probe_finds_pymavlink_under_PYMAVLINK_PATH(tmp_path, monkeypatch):
    pkg = tmp_path / "pymavlink"
    pkg.mkdir()
    (pkg / "__init__.py").write_text("")
    (pkg / "mavutil.py").write_text("def mavlink_connection(*a, **k):\n    raise RuntimeError('boom')\n")
    monkeypatch.delitem(sys.modules, "pymavlink", raising=False)
    monkeypatch.setenv("PYMAVLINK_PATH", str(tmp_path))
    monkeypatch.setattr(sys, "path", list(sys.path))
    rc, why = fw.Host().probe_version("/dev/null")
    assert rc is None and "boom" in why  # imported, then failed at connect: the import works
    sys.modules.pop("pymavlink", None); sys.modules.pop("pymavlink.mavutil", None)


# -- Host.port_busy without fuser/lsof: scan a fake /proc ------------------------
def fake_proc(tmp_path, holds=None, readable=True):
    proc = tmp_path / "proc"
    proc.mkdir()
    (proc / "self").mkdir()  # non-numeric entries are skipped
    for pid, target in {"100": "/dev/null", "200": holds or "/dev/null"}.items():
        fd = proc / pid / "fd"
        fd.mkdir(parents=True)
        if readable:
            (fd / "3").symlink_to(target)
        else:
            (proc / pid / "fd").chmod(0)
    return proc


@pytest.fixture
def nobin(monkeypatch):
    monkeypatch.setattr(fw.shutil, "which", lambda name: None)


def test_port_busy_proc_fallback_finds_holder_via_by_id_target(tmp_path, monkeypatch, nobin):
    dev = tmp_path / "ttyACM0"
    dev.write_text("")
    byid = tmp_path / "by-id-x"
    byid.symlink_to(dev)
    monkeypatch.setattr(fw, "PROC", str(fake_proc(tmp_path, holds=str(dev))))
    assert fw.Host().port_busy(str(byid)) == (True, "200")


def test_port_busy_proc_fallback_free(tmp_path, monkeypatch, nobin):
    dev = tmp_path / "ttyACM0"
    dev.write_text("")
    monkeypatch.setattr(fw, "PROC", str(fake_proc(tmp_path)))
    assert fw.Host().port_busy(str(dev)) == (False, "")


def test_port_busy_proc_unreadable_is_unknown(tmp_path, monkeypatch, nobin):
    monkeypatch.setattr(fw, "PROC", str(tmp_path / "missing"))
    assert fw.Host().port_busy("/dev/ttyACM0")[0] is None
    import os
    if os.geteuid() != 0:  # chmod 0 is not enforced for root
        monkeypatch.setattr(fw, "PROC", str(fake_proc(tmp_path, readable=False)))
        assert fw.Host().port_busy("/dev/ttyACM0")[0] is None


def test_px4_deps_script_installs_and_checks_psmisc():
    sh = (Path(__file__).resolve().parent.parent / "scripts/deps/px4.sh").read_text()
    assert "\n    psmisc\n" in sh


def test_mav_version_hash_matches_artifact():
    # recorded from the Pixhawk 6X (X500) running the v1.15.4 artifact (git_hash 99c40407ff):
    # old decoding printed 000000ff0704c499, i.e. the wire bytes below
    custom = bytes.fromhex("000000ff0704c499")
    assert fw.mav_version(0x010F04FF, custom) == (1, 15, 4, "99c40407ff")
    assert fw.mav_version(0x010F04FF, b"") == (1, 15, 4, "")
