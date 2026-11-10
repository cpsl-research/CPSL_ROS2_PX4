"""FC EVENT / STATUSTEXT logging must survive events of different severities (fakes only).

Regression: px4_control_node died ~1 s after launch on the first FC EVENT whose severity
differed from an earlier one, because rclpy raises ValueError("Logger severity cannot be
changed between calls") when one call site logs at two severities, and the callback logged
through `getattr(logger, level)(...)`.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "px4_controller"))
from px4_controller import px4_events as pe  # noqa: E402
from px4_controller import telemetry_logic as tl  # noqa: E402

ARMING_SUMMARY = 11047904


class CallSiteLogger:
    """Mimics rclpy's per-call-site severity check."""

    def __init__(self):
        self.sites, self.lines = {}, []

    def _log(self, sev, msg):
        f = sys._getframe(2)
        site = (f.f_code.co_filename, f.f_code.co_name, f.f_lineno)
        if self.sites.setdefault(site, sev) != sev:
            raise ValueError("Logger severity cannot be changed between calls.")
        self.lines.append((sev, msg))

    def info(self, m): self._log("info", m)
    def warn(self, m): self._log("warn", m)
    def error(self, m): self._log("error", m)


def test_getattr_pattern_reproduces_the_crash():
    lg = CallSiteLogger()

    def old(sev):
        getattr(lg, pe.log_level(sev))("x")
    old(6)
    with pytest.raises(ValueError):
        old(4)


def test_emit_survives_every_severity_repeatedly():
    lg = CallSiteLogger()
    for sev in (6, 4, 3, 8, 0, 7, 6, 4, 2, 5):
        pe.emit(lg, sev, "m")
    assert [s for s, _ in lg.lines].count("warn") == 2 and len(lg.lines) == 10


def test_real_rclpy_logger_if_present():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.impl.rcutils_logger import RcutilsLogger
    lg = RcutilsLogger(name="t")
    for sev in (6, 4, 3, 6, 4):
        pe.emit(lg, sev, "m")


@pytest.mark.parametrize("raw", [b"", b"\x01", bytes(40), bytes([255] * 40), bytes(range(1, 26))])
def test_arming_summary_decodes_and_feeds_health(raw):
    log = pe.EventLog()
    rec = log.add(8, ARMING_SUMMARY, raw)
    assert rec["text"] == "Arming check summary event"
    items = tl.health_items({"fc_events": log.recent(rec["t"] + 1, 120)},
                            {"vehicle_status": 0.1, "odometry": None, "local_position": 5})
    for it in items:
        bytes([it["level"]])
