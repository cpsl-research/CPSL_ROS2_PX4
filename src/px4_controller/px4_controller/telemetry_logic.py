"""Pure decoders for the console's extra telemetry topics (no ROS, no px4_msgs).

Each function takes a message-like object (anything with the PX4 field names,
so a SimpleNamespace works in tests) and returns plain numbers or dicts; the
node builds the standard ROS messages from them. See README "Console
interface": gps/fix, range, status/health.
"""
import math

NAN = float('nan')

# sensor_msgs/NavSatStatus.status
NAV_NO_FIX = -1
NAV_FIX = 0
NAV_SBAS_FIX = 1
NAV_GBAS_FIX = 2

# px4_msgs/SensorGps.fix_type
_GPS_STATUS = {0: NAV_NO_FIX, 1: NAV_NO_FIX, 2: NAV_FIX, 3: NAV_FIX,
               4: NAV_GBAS_FIX, 5: NAV_GBAS_FIX, 6: NAV_GBAS_FIX, 8: NAV_NO_FIX}

# diagnostic_msgs/DiagnosticStatus.level
OK, WARN, ERROR, STALE = 0, 1, 2, 3

LINK_WARN_S = 1.0     # a source silent this long is WARN, ERROR after LINK_ERROR_S
LINK_ERROR_S = 3.0


def _f(x):
    """float(x), NaN for None/inf/NaN."""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return NAN
    return v if math.isfinite(v) else NAN


def gps_fix_fields(gps):
    """px4_msgs/SensorGps (/fmu/out/vehicle_gps_position) -> NavSatFix fields.

    status from fix_type (0/1/unknown/8 NO_FIX, 2D/3D FIX, RTCM/RTK GBAS_FIX).
    Without a fix latitude/longitude/altitude are NaN. altitude is height above
    the WGS84 ellipsoid (what NavSatFix defines). The position covariance is
    diagonal from eph/epv (variance = sigma^2); NaN sigma gives covariance_type
    UNKNOWN (0), else DIAGONAL_KNOWN (2). Satellites and hdop have no NavSatFix
    field: they are in status/health.
    """
    status = _GPS_STATUS.get(int(gps.fix_type), NAV_NO_FIX)
    has_fix = status != NAV_NO_FIX
    eph, epv = _f(gps.eph), _f(gps.epv)
    known = has_fix and not math.isnan(eph) and not math.isnan(epv)
    return {
        'status': status,
        'latitude': _f(gps.latitude_deg) if has_fix else NAN,
        'longitude': _f(gps.longitude_deg) if has_fix else NAN,
        'altitude': _f(gps.altitude_ellipsoid_m) if has_fix else NAN,
        'covariance': ([eph * eph, 0.0, 0.0, 0.0, eph * eph, 0.0, 0.0, 0.0, epv * epv]
                       if known else [0.0] * 9),
        'covariance_type': 2 if known else 0,
    }


def range_fields(dist_bottom, valid):
    """vehicle_local_position.dist_bottom/dist_bottom_valid -> Range.range.

    NaN (REP-117: invalid) when the estimator says the distance is not valid or
    the value is not finite or is negative. This is the EKF's height above
    ground (a rangefinder when one is fused), not a raw sensor reading.
    """
    d = _f(dist_bottom)
    if not valid or math.isnan(d) or d < 0.0:
        return {'range': NAN}
    return {'range': d}


def ekf_validity(lp):
    """vehicle_local_position -> dict of xy/z/v_xy/v_z validity bools."""
    return {k: bool(getattr(lp, k)) for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid')}


def fusion_flags(flags):
    """estimator_status_flags -> {'opt_flow': cs_opt_flow, 'rng_hgt': cs_rng_hgt}."""
    return {'opt_flow': bool(flags.cs_opt_flow), 'rng_hgt': bool(flags.cs_rng_hgt)}


def gps_health(gps):
    """Decode SensorGps to {'fix_type', 'satellites', 'hdop'} (hdop NaN when not finite)."""
    return {'fix_type': int(gps.fix_type), 'satellites': int(gps.satellites_used),
            'hdop': _f(gps.hdop)}


BATTERY_WARNING_NAMES = {0: 'none', 1: 'low', 2: 'critical', 3: 'emergency', 4: 'failed',
                         10: 'over_temperature'}


def _kv(**kw):
    return {k: ('nan' if isinstance(v, float) and math.isnan(v) else str(v))
            for k, v in kw.items()}


def health_items(inputs, ages):
    """Health items for status/health, each {name, level, message, values}.

    inputs: dict with any of 'ekf' (ekf_validity), 'fusion' (fusion_flags),
    'gps' (gps_health), 'battery_warning' (int), 'failsafe_flags' (tuple);
    a missing key means "never heard": that item is STALE. ages: seconds since
    the last message per source name, None = never heard.
    """
    items = []

    ekf = inputs.get('ekf')
    if ekf is None:
        items.append({'name': 'ekf', 'level': STALE, 'message': 'no vehicle_local_position',
                      'values': {}})
    else:
        bad = [k[:-6] for k, v in ekf.items() if not v]
        items.append({'name': 'ekf', 'level': WARN if bad else OK,
                      'message': ('invalid: ' + ','.join(bad)) if bad else 'xy z v_xy v_z valid',
                      'values': _kv(**ekf)})

    fus = inputs.get('fusion')
    if fus is None:
        items.append({'name': 'fusion', 'level': STALE, 'message': 'no estimator_status_flags',
                      'values': {}})
    else:
        items.append({'name': 'fusion', 'level': OK if fus['opt_flow'] else WARN,
                      'message': 'optical flow %s, range height %s' % (
                          'fused' if fus['opt_flow'] else 'not fused',
                          'fused' if fus['rng_hgt'] else 'not fused'),
                      'values': _kv(**fus)})

    gps = inputs.get('gps')
    if gps is None:
        items.append({'name': 'gps', 'level': STALE, 'message': 'no vehicle_gps_position',
                      'values': {}})
    else:
        ok = gps['fix_type'] >= 3
        items.append({'name': 'gps', 'level': OK if ok else WARN,
                      'message': '%d satellites, fix_type %d' % (
                          gps['satellites'], gps['fix_type']),
                      'values': _kv(**gps)})

    bw = inputs.get('battery_warning')
    if bw is None:
        items.append({'name': 'battery', 'level': STALE, 'message': 'no battery_status',
                      'values': {}})
    else:
        level = OK if bw == 0 else (WARN if bw == 1 else ERROR)
        items.append({'name': 'battery', 'level': level,
                      'message': 'warning ' + BATTERY_WARNING_NAMES.get(bw, str(bw)),
                      'values': _kv(warning=bw)})

    flags = inputs.get('failsafe_flags')
    if flags is None:
        items.append({'name': 'failsafe', 'level': STALE, 'message': 'no failsafe_flags',
                      'values': {}})
    else:
        items.append({'name': 'failsafe', 'level': WARN if flags else OK,
                      'message': '%d flag(s) set' % len(flags),
                      'values': _kv(count=len(flags), flags=','.join(flags))})

    link_vals, level = {}, OK
    for src in sorted(ages):
        a = ages[src]
        link_vals[src] = 'never' if a is None else '%.2f' % a
        if a is None or a > LINK_ERROR_S:
            level = max(level, ERROR)
        elif a > LINK_WARN_S:
            level = max(level, WARN)
    items.append({'name': 'link', 'level': level,
                  'message': ('all sources fresh' if level == OK
                              else 'a source is late or silent'),
                  'values': link_vals})
    return items
