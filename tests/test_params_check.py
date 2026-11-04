"""params_check.py: parsing, comparison, allowed differences and the read path (fakes only)."""
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import params_check as pc  # noqa: E402
import host_setup as hs  # noqa: E402

LAB = {"EKF2_OF_CTRL": 1.0, "EKF2_RNG_CTRL": 2.0, "EKF2_HGT_REF": 2.0, "EKF2_GPS_CTRL": 0.0,
       "EKF2_GPS_CHECK": 0.0, "EKF2_BARO_CTRL": 0.0, "EKF2_OF_POS_X": 0.12}


def test_lab_file_parses_and_has_the_families():
    lab = pc.load_lab()
    assert len(lab) > 100
    assert lab["EKF2_OF_CTRL"] == 1.0 and "UAVCAN_ENABLE" in lab and "BAT1_N_CELLS" in lab
    assert not [n for n in lab if not n.startswith(("EKF2_", "UAVCAN_", "BAT", "SENS_"))]


def test_allowed_diffs_are_info_not_warn():
    live = dict(LAB, EKF2_HGT_REF=1.0, EKF2_GPS_CTRL=7.0, EKF2_GPS_CHECK=245.0, EKF2_BARO_CTRL=1.0)
    r = pc.rows(LAB, live)
    assert r[0]["status"] == "PASS" and "4 allowed" in r[0]["detail"]
    assert {x["name"] for x in r[1:]} == {"allowed:EKF2_HGT_REF", "allowed:EKF2_GPS_CTRL",
                                          "allowed:EKF2_GPS_CHECK", "allowed:EKF2_BARO_CTRL"}
    assert all(x["status"] == "INFO" for x in r[1:])


def test_real_diff_is_warn_with_qgc_fix_and_row_shape():
    live = dict(LAB, EKF2_OF_CTRL=0.0, EKF2_OF_POS_X=0.0)
    r = pc.rows(LAB, live)
    assert r[0]["status"] == "WARN" and "2 of 7" in r[0]["detail"]
    row = next(x for x in r if x["name"] == "param:EKF2_OF_CTRL")
    assert row["detail"] == "live 0, lab 1" and row["fix"] == "in QGroundControl set EKF2_OF_CTRL to 1"
    assert all(set(x) == {"name", "status", "detail", "fix", "why"} for x in r)


def test_unreadable_params_are_counted_not_failed():
    live = {k: v for k, v in LAB.items() if k != "EKF2_RNG_CTRL"}
    r = pc.rows(LAB, live)
    assert r[0]["status"] == "PASS" and "1 unreadable" in r[0]["detail"]
    assert pc.rows(LAB, {})[0]["status"] == "INFO"


def test_integer_params_decode_from_the_float_field():
    as_float = struct.unpack("<f", struct.pack("<i", 245))[0]
    assert pc.decode_value(as_float, 6) == 245.0
    assert pc.decode_value(0.23, 9) == 0.23


def test_run_with_fake_reader_never_writes(tmp_path):
    f = tmp_path / "lab.params"
    f.write_text("# c\n1\t1\tEKF2_OF_CTRL\t1\t6\n1\t1\tEKF2_HGT_REF\t2\t6\n")
    r = pc.run("dev", f, reader=lambda dev, names: ({"EKF2_OF_CTRL": 0.0, "EKF2_HGT_REF": 1.0}, ""))
    assert [x["name"] for x in r] == ["params-vs-lab", "param:EKF2_OF_CTRL", "allowed:EKF2_HGT_REF"]
    r = pc.run("dev", f, reader=lambda dev, names: ({}, "no heartbeat from the flight controller"))
    assert len(r) == 1 and r[0]["status"] == "INFO"


class FakeMav:
    def __init__(self):
        self.sent = []
        self.mav = SimpleNamespace(param_request_read_send=lambda *a: self.sent.append(a))
        self.target_system = self.target_component = 1
        self._last = None
        self.mav.param_request_read_send = self._req

    def _req(self, ts, tc, pid, idx):
        self._last = pid.decode()
        self.sent.append(self._last)

    def wait_heartbeat(self, timeout=0):
        return object()

    def recv_match(self, type=None, blocking=True, timeout=0):
        v = struct.unpack("<f", struct.pack("<i", 3))[0]
        return SimpleNamespace(param_id=self._last.encode(), param_value=v, param_type=6)

    def close(self):
        pass


def test_read_params_only_sends_param_request_read():
    fm = FakeMav()
    live, err = pc.read_params("dev", ["UAVCAN_ENABLE"], connect=lambda p, baud: fm)
    assert err == "" and live == {"UAVCAN_ENABLE": 3.0} and fm.sent == ["UAVCAN_ENABLE"]
    assert not hasattr(fm.mav, "param_set_send")


def test_host_setup_flag_delegates(monkeypatch, capsys):
    monkeypatch.setattr(pc, "read_params", lambda dev, names, **k: ({}, "no heartbeat"))
    assert hs.main(["--params-check", "--json"]) == 0
    assert '"INFO"' in capsys.readouterr().out
