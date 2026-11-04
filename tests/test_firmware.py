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
