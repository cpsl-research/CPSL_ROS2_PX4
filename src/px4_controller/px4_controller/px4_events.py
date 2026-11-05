"""Decode PX4 MAVLink EVENTs (id + packed arguments) into text, and keep the latest few.

No ROS here. The id table is generated (px4_event_table.py, see
scripts/gen_px4_events.py); an id that is not in it is shown as `PX4 event <id>`.

Transport: the MAVROS backend gets EVENT as `<mavros>/status_event`. The xrce backend has
none: PX4 v1.15 does not bridge the `event` topic in dds_topics.yaml, so over XRCE the
refusal reason stays in PX4's own log (see README).
"""
import collections
import re
import struct
import time

from px4_controller.px4_event_table import ENUMS, EVENTS

SEVERITY_NAMES = ('EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG')
KEEP_EVENTS = 5

_STRUCT = {'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h', 'uint32_t': 'I',
           'int32_t': 'i', 'uint64_t': 'Q', 'int64_t': 'q', 'float': 'f'}
_PLACEHOLDER = re.compile(r'\{(\d+)(?::\.(\d+))?([^}]*)\}')
_UNITS = {'m': ' m', 'm/s': ' m/s', 'm_v': ' m'}
_TAG = re.compile(r'<[^>]+>')


def severity_name(severity):
    return SEVERITY_NAMES[severity] if 0 <= severity < len(SEVERITY_NAMES) else str(severity)


def log_level(severity):
    """Logger method name for a MAVLink severity (0-3 error, 4 warn, else info)."""
    return 'error' if severity <= 3 else ('warn' if severity == 4 else 'info')


def _enum_text(name, value):
    _, bitfield, entries = ENUMS[name]
    if not bitfield:
        return entries.get(value, str(value))
    names = [n for bit, n in sorted(entries.items()) if bit and value & bit]
    return '|'.join(names) if names else str(value)


def decode_arguments(types, raw):
    """Unpack `raw` bytes by the argument type names; a short buffer gives fewer values."""
    values, off = [], 0
    for t in types:
        base = ENUMS[t][0] if t in ENUMS else t
        code = _STRUCT.get(base)
        if code is None:
            break
        size = struct.calcsize('<' + code)
        if off + size > len(raw):
            break
        v = struct.unpack_from('<' + code, bytes(raw), off)[0]
        values.append(_enum_text(t, v) if t in ENUMS else v)
        off += size
    return values


def event_text(event_id, raw_args=b''):
    """Human text for an event id and its packed argument bytes."""
    entry = EVENTS.get(int(event_id))
    if entry is None:
        return 'PX4 event %d' % event_id
    _, message, args = entry
    vals = decode_arguments([t for _, t in args], raw_args)

    def sub(m):
        i = int(m.group(1)) - 1
        if i >= len(vals):
            return '?'
        v = vals[i]
        if isinstance(v, float) and m.group(2) is not None:
            v = '%.*f' % (int(m.group(2)), v)
        return '%s%s' % (v, _UNITS.get(m.group(3), ''))
    return _TAG.sub('', _PLACEHOLDER.sub(sub, message))


def event_name(event_id):
    entry = EVENTS.get(int(event_id))
    return entry[0] if entry else None


class EventLog:
    """Latest `keep` decoded events; add() returns the decoded record."""

    def __init__(self, keep=KEEP_EVENTS):
        self.items = collections.deque(maxlen=keep)

    def add(self, severity, event_id, raw_args=b'', now=None):
        rec = {'t': time.monotonic() if now is None else now, 'severity': int(severity),
               'id': int(event_id), 'name': event_name(event_id),
               'text': event_text(event_id, raw_args)}
        self.items.append(rec)
        return rec

    def recent(self, now, max_age_s):
        """Newest first, each {level, severity, id, text, age} within max_age_s."""
        out = []
        for r in reversed(self.items):
            age = now - r['t']
            if age <= max_age_s:
                out.append(dict(r, level=severity_name(r['severity']), age=age))
        return out


def format_events(recent):
    """One line per event for the health item: `ERROR 12s ago: text`."""
    return ' | '.join('%s %.0fs ago: %s' % (r['level'], r['age'], r['text']) for r in recent)
