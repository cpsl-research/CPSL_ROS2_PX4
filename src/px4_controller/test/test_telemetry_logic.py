"""Unit tests for telemetry_logic and the node's message building.

No ROS graph, no px4_msgs, no aircraft.

    python3 -m pytest src/px4_controller/test/test_telemetry_logic.py
"""
import math
import os
import sys
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from px4_controller import telemetry_logic as tl  # noqa: E402,I100


def _gps(fix_type=3, **kw):
    d = {'fix_type': fix_type, 'latitude_deg': 32.7, 'longitude_deg': -117.2,
         'altitude_ellipsoid_m': 30.0, 'eph': 0.5, 'epv': 0.8, 'hdop': 0.9, 'satellites_used': 11}
    d.update(kw)
    return NS(**d)


def test_gps_fix_3d():
    f = tl.gps_fix_fields(_gps())
    assert f['status'] == tl.NAV_FIX
    assert (f['latitude'], f['longitude'], f['altitude']) == (32.7, -117.2, 30.0)
    assert f['covariance'] == [0.25, 0, 0, 0, 0.25, 0, 0, 0, pytest.approx(0.64)]
    assert f['covariance_type'] == 2


@pytest.mark.parametrize('fix_type,status', [
    (0, -1), (1, -1), (2, 0), (3, 0), (4, 2), (5, 2), (6, 2), (8, -1), (99, -1)])
def test_gps_status_from_fix_type(fix_type, status):
    assert tl.gps_fix_fields(_gps(fix_type))['status'] == status


def test_gps_no_fix_is_nan_and_unknown_covariance():
    f = tl.gps_fix_fields(_gps(1))
    assert all(math.isnan(f[k]) for k in ('latitude', 'longitude', 'altitude'))
    assert f['covariance_type'] == 0 and f['covariance'] == [0.0] * 9


def test_gps_nan_accuracy_is_unknown_covariance():
    f = tl.gps_fix_fields(_gps(eph=float('nan')))
    assert f['status'] == 0 and f['covariance_type'] == 0


def test_range_valid_invalid():
    assert tl.range_fields(1.25, True) == {'range': pytest.approx(1.25)}
    for d, v in ((1.0, False), (float('nan'), True), (float('inf'), True), (-0.1, True)):
        assert math.isnan(tl.range_fields(d, v)['range'])


def test_gps_health_and_flags():
    assert tl.gps_health(_gps(hdop=float('nan')))['satellites'] == 11
    assert math.isnan(tl.gps_health(_gps(hdop=float('nan')))['hdop'])
    lp = NS(xy_valid=True, z_valid=False, v_xy_valid=1, v_z_valid=0)
    assert tl.ekf_validity(lp) == {'xy_valid': True, 'z_valid': False,
                                   'v_xy_valid': True, 'v_z_valid': False}
    assert tl.fusion_flags(NS(cs_opt_flow=1, cs_rng_hgt=0)) == {'opt_flow': True, 'rng_hgt': False}


def _by_name(items):
    return {i['name']: i for i in items}


GOOD = {
    'ekf': {'xy_valid': True, 'z_valid': True, 'v_xy_valid': True, 'v_z_valid': True},
    'fusion': {'opt_flow': True, 'rng_hgt': True},
    'gps': {'fix_type': 3, 'satellites': 12, 'hdop': 0.8},
    'battery_warning': 0,
    'failsafe_flags': (),
}
FRESH = {'vehicle_status': 0.1, 'odometry': 0.02, 'local_position': 0.05}


def test_health_all_ok():
    items = _by_name(tl.health_items(GOOD, FRESH))
    assert set(items) == {'ekf', 'fusion', 'gps', 'battery', 'failsafe', 'link'}
    assert all(i['level'] == tl.OK for i in items.values())
    assert items['gps']['values'] == {'fix_type': '3', 'satellites': '12', 'hdop': '0.8'}


def test_health_nothing_heard_is_stale():
    items = _by_name(tl.health_items({}, {'vehicle_status': 0.1, 'odometry': None}))
    for n in ('ekf', 'fusion', 'gps', 'battery', 'failsafe'):
        assert items[n]['level'] == tl.STALE and items[n]['values'] == {}
    assert items['link']['level'] == tl.ERROR and items['link']['values']['odometry'] == 'never'


def test_health_degraded():
    inp = dict(GOOD)
    inp['ekf'] = dict(GOOD['ekf'], xy_valid=False, v_xy_valid=False)
    inp['fusion'] = {'opt_flow': False, 'rng_hgt': True}
    inp['gps'] = {'fix_type': 1, 'satellites': 2, 'hdop': float('nan')}
    inp['battery_warning'] = 2
    inp['failsafe_flags'] = ('a_flag', 'b_flag')
    items = _by_name(tl.health_items(inp, dict(FRESH, odometry=1.5)))
    assert items['ekf']['level'] == tl.WARN and 'xy' in items['ekf']['message']
    assert items['fusion']['level'] == tl.WARN
    assert items['gps']['level'] == tl.WARN and items['gps']['values']['hdop'] == 'nan'
    assert items['battery']['level'] == tl.ERROR and 'critical' in items['battery']['message']
    assert items['failsafe']['values'] == {'count': '2', 'flags': 'a_flag,b_flag'}
    assert items['link']['level'] == tl.WARN


def test_node_builds_standard_messages():
    """The node's gps/fix, range and status/health message building, on stub publishers."""
    px = pytest.importorskip('px4_controller.px4_control_node')
    n = object.__new__(px.PX4ControlNode)

    class Pub:
        sent = None

        def publish(self, m):
            self.sent = m
    n.gps_fix_publisher, n.range_publisher, n.health_publisher = Pub(), Pub(), Pub()
    n.health_inputs, n.rx_s, n.next_range_due_s = {}, {}, None
    n.have_vehicle_status = True
    n.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: __import__(
        'builtin_interfaces.msg', fromlist=['Time']).Time()))
    n.on_gps_fix(tl.gps_fix_fields(_gps(5)), tl.gps_health(_gps(5)))
    assert n.gps_fix_publisher.sent.status.status == 2
    assert n.health_inputs['gps']['satellites'] == 11
    n.on_range(tl.range_fields(2.0, True))
    assert n.range_publisher.sent.range == pytest.approx(2.0)
    n.on_range(tl.range_fields(3.0, True))              # inside the 10 Hz limit: dropped
    assert n.range_publisher.sent.range == pytest.approx(2.0)
    n.on_health_input('ekf', GOOD['ekf'])
    n.rx_s['vehicle_status'] = n.rx_s['odometry'] = time.monotonic()
    n.publish_health()
    arr = n.health_publisher.sent
    names = {s.name: s for s in arr.status}
    assert set(names) == {'px4/' + k for k in
                          ('ekf', 'fusion', 'gps', 'battery', 'failsafe', 'link')}
    assert names['px4/ekf'].level == b'\x00' and names['px4/fusion'].level == b'\x03'
    assert {kv.key for kv in names['px4/gps'].values} == {'fix_type', 'satellites', 'hdop'}
