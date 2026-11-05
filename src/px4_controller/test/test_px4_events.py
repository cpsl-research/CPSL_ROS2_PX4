"""Unit tests for px4_controller.px4_events (no ROS, no aircraft). Ids are from the bench.

    python3 -m pytest src/px4_controller/test/test_px4_events.py
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from px4_controller import px4_events as pe  # noqa: E402
from px4_controller import telemetry_logic as tl  # noqa: E402

BENCH = {
    133277: ('commander_arm_denied_resolve_failures',
             'Arming denied: Resolve system health failures first'),
    10011251: ('check_modes_global_pos', 'No valid global position estimate'),
    377561: ('check_modes_mission', 'No valid mission available'),
    79408: ('check_modes_home_position', 'Home position not set'),
    453929: ('check_modes_manual_control', 'No manual control input'),
    6801787: ('check_rc_dl_no_dllink', 'No connection to the ground control station'),
}


def test_bench_ids_decode():
    for eid, (name, text) in BENCH.items():
        assert pe.event_name(eid) == name
        assert pe.event_text(eid, bytes(25)) == text


def test_arguments_are_unpacked_little_endian():
    # check_mag_consistency: modes u32, health index u8, arg0 int16 -> "{3}"
    raw = struct.pack('<IBh', 1, 3, 17) + bytes(30)
    assert pe.event_text(5273740, raw) == 'Compass inconsistent by 17 degrees'
    raw = struct.pack('<IBh', 1, 3, -4)
    assert pe.event_text(5273740, raw) == 'Compass inconsistent by -4 degrees'


def test_short_or_unknown():
    assert pe.event_text(5273740, b'') == 'Compass inconsistent by ? degrees'
    assert pe.event_text(1, b'') == 'PX4 event 1'
    assert pe.event_name(1) is None


def test_log_level_and_names():
    assert [pe.log_level(s) for s in (0, 3, 4, 6)] == ['error', 'error', 'warn', 'info']
    assert pe.severity_name(3) == 'ERROR' and pe.severity_name(99) == '99'


def test_event_log_keeps_latest_and_ages():
    log = pe.EventLog(keep=3)
    for i, eid in enumerate((79408, 453929, 6801787, 133277)):
        log.add(6 if i < 3 else 3, eid, b'', now=100.0 + i)
    rec = log.recent(now=110.0, max_age_s=120.0)
    assert [r['id'] for r in rec] == [133277, 6801787, 453929]    # newest first, 3 kept
    assert rec[0]['level'] == 'ERROR' and abs(rec[0]['age'] - 7.0) < 1e-9
    assert log.recent(now=300.0, max_age_s=120.0) == []
    assert 'ERROR 7s ago: Arming denied' in pe.format_events(rec)


def test_link_item_shows_events():
    log = pe.EventLog()
    log.add(3, 133277, b'', now=10.0)
    log.add(6, 79408, b'', now=12.0)
    inputs = {'fc_events': log.recent(15.0, 120.0)}
    link = [i for i in tl.health_items(inputs, {'vehicle_status': 0.1}) if i['name'] == 'link'][0]
    assert 'FC event: Home position not set' in link['message']
    assert link['values']['fc_event_severity'] == '6'
    assert 'ERROR 5s ago: Arming denied: Resolve system health failures first' \
        in link['values']['fc_events']
    plain = [i for i in tl.health_items({}, {'vehicle_status': 0.1}) if i['name'] == 'link'][0]
    assert 'fc_event' not in plain['values']


def test_table_matches_generator_shape():
    from px4_controller.px4_event_table import ENUMS, EVENTS
    assert len(EVENTS) > 300 and 133277 in EVENTS
    for _, _, args in EVENTS.values():
        for _, t in args:
            assert t in ENUMS or t in pe._STRUCT
