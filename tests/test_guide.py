"""Tests for host_setup.py --guide and firmware.firmware_row (fakes only: no board, no network, no serial)."""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import firmware as fw  # noqa: E402
import host_setup as hs  # noqa: E402
from test_firmware import FakeHost  # noqa: E402

P = fw.load_pairing()
A = argparse.Namespace(iface="enp3s0", host_ip="10.41.10.1", fcu_ip="10.41.10.2", prefix=24,
                       device=hs.USB_DEVICE, no_probe=True, agent_log=None, con_name="c")


def fh(**kw):
    h = FakeHost(**kw)
    h.probe_version = lambda path, timeout=5.0: h.version
    return h


def ver(ma, mi, pa):
    return (ma << 24 | mi << 16 | pa << 8), b"\x01" * 8


def test_firmware_mismatch_is_fail_with_both_remedies():
    r = fw.firmware_row(fh(version=ver(1, 17, 0)), P)
    assert r["status"] == "FAIL" and "v1.17.0" in r["detail"] and "release/1.15" in r["detail"]
    text = " ".join(r["manual"])
    assert "reflash" in text.lower() and "re-pin" in text.lower() and "does not choose" in text


def test_firmware_match_pass_and_no_manual():
    r = fw.firmware_row(fh(version=ver(1, 15, 4)), P)
    assert r["status"] == "PASS" and "manual" not in r


def test_firmware_info_when_busy_no_probe_or_no_pymavlink():
    assert fw.firmware_row(fh(busy=True), P)["status"] == "INFO"
    assert "no-probe" in fw.firmware_row(fh(version=ver(1, 17, 0)), P, probe=False)["detail"]
    assert "pymavlink" in fw.firmware_row(fh(version=(None, "pymavlink not installed")), P)["detail"]


def test_firmware_never_opens_port_when_busy_or_no_probe():
    h = fh(busy=True, version=ver(1, 17, 0))
    called = []
    h.probe_version = lambda *a, **k: called.append(1)
    fw.firmware_row(h, P)
    fw.firmware_row(h, P, probe=False)
    assert not called


def test_firmware_inferred_from_versioned_topics():
    topics = ["/fmu/out/vehicle_status_v1", "/fmu/out/battery_status"]
    r = fw.firmware_row(fh(busy=True), P, topics=topics)
    assert r["status"] == "FAIL" and "inferred" in r["detail"]
    assert fw.firmware_row(fh(busy=True), P, topics=["/fmu/out/vehicle_status"])["status"] == "INFO"


def test_check_rows_unchanged_by_the_new_fields():
    rows = fw.run_checks(fh(version=ver(1, 15, 4)), P, argparse.Namespace(
        no_probe=True, artifact=None, out="/nonexistent", device=hs.USB_DEVICE))
    assert all(set(r) == {"name", "status", "detail", "fix", "why"} for r in rows)


def test_fcu_address_found_elsewhere_is_fail():
    r = hs.fcu_address_row(A, False, True, arp=lambda i: [], probe=lambda i, ip: ip == "192.168.0.3")
    assert r["status"] == "FAIL" and "192.168.0.3, expected 10.41.10.2" in r["detail"]
    assert r["kind"] == "manual" and r["confirmed_by"] == ["fcu"] and "ifconfig" in " ".join(r["manual"])


def test_fcu_address_from_arp_table_and_ignores_own_address():
    r = hs.fcu_address_row(A, False, True, arp=lambda i: ["10.41.10.1", "192.168.0.4"], probe=lambda i, ip: False)
    assert r["status"] == "FAIL" and "192.168.0.4" in r["detail"]


def test_fcu_address_nothing_found_and_no_cap_are_info():
    assert hs.fcu_address_row(A, False, True, arp=lambda i: [], probe=lambda i, ip: False)["status"] == "INFO"
    r = hs.fcu_address_row(A, False, True, arp=lambda i: [], probe=lambda i, ip: None)
    assert r["status"] == "INFO" and "CAP_NET_RAW" in r["detail"]


def test_fcu_address_skips_probing_when_ping_ok_or_no_carrier():
    def boom(*a):
        raise AssertionError("probed")
    assert hs.fcu_address_row(A, True, True, arp=boom, probe=boom)["status"] == "PASS"
    assert hs.fcu_address_row(A, False, False, arp=boom, probe=boom)["status"] == "INFO"


def test_read_arp_only_that_interface_and_complete(tmp_path):
    f = tmp_path / "arp"
    f.write_text("IP address HW type Flags HW address Mask Device\n"
                 "192.168.0.3 0x1 0x2 aa:bb:cc:dd:ee:ff * enp3s0\n"
                 "192.168.0.9 0x1 0x2 aa:bb:cc:dd:ee:00 * wlan0\n"
                 "192.168.0.4 0x1 0x0 00:00:00:00:00:00 * enp3s0\n")
    assert hs.read_arp("enp3s0", str(f)) == ["192.168.0.3"]


def test_xrce_session_rows():
    assert hs.xrce_session_row(A, ["/fmu/out/x"], True)["status"] == "PASS"
    assert hs.xrce_session_row(A, None, False)["status"] == "INFO"
    assert hs.xrce_session_row(A, [], False)["status"] == "INFO"
    assert hs.xrce_session_row(A, [], True)["status"] == "FAIL"
    assert hs.xrce_session_row(A, None, True)["status"] == "INFO"
    assert hs.xrce_session_row(A, None, True, "... session established ...")["status"] == "PASS"


def test_guide_rows_order_kinds_and_fix_is_a_command(monkeypatch):
    monkeypatch.setattr(hs, "ros_topics", lambda: None)
    monkeypatch.setattr(hs, "agent_running", lambda: False)
    monkeypatch.setattr(hs, "read_arp", lambda i, path="": [])
    monkeypatch.setattr(hs, "arping", lambda i, ip: None)
    rows = hs.guide_rows(A)
    steps = [r["step"] for r in rows]
    assert steps == sorted(steps) and {1, 2, 3, 4} <= set(steps)
    names = [r["name"] for r in rows]
    assert names.index("companion-address") < names.index("fcu-netcfg") < names.index("dds-cfg") < names.index("dds-params")
    assert {"usb", "xrce-session", "firmware", "fcu-address"} <= set(names)
    by = {r["name"]: r for r in rows}
    assert all(r["kind"] in ("check", "manual") for r in rows)
    for r in rows:
        if r["kind"] == "manual":
            assert not r["fix"] and r["manual"]
        elif r["fix"]:
            assert r["fix"].split()[0] in hs.CMD_WORDS
    assert by["fcu-netcfg"]["literal"]["text"].splitlines()[:2] == ["DEVICE=eth0", "BOOTPROTO=fallback"]
    assert by["dds-params"]["confirmed_by"] == ["xrce-session"]
    assert "UXRCE_DDS_AG_IP = 170461697" in " ".join(by["dds-params"]["manual"])


def test_step_row_moves_hint_out_of_fix():
    r = hs.step_row(hs.chk("interface", "FAIL", "d", "plug in the cable"), 1)
    assert r["fix"] == "" and r["manual"] == ["plug in the cable"] and r["kind"] == "check"
    r = hs.step_row(hs.chk("address", "FAIL", "d", "python3 scripts/host_setup.py --apply"), 1)
    assert r["fix"].startswith("python3") and "manual" not in r


def test_no_write_operations_in_guide_code():
    src = (Path(hs.__file__).read_text() + Path(fw.__file__).read_text())
    assert "PARAM_SET" not in src and "param set" not in src.replace("never write", "")

