"""Bounded monotone jaw-width tables; separate opening/closing hysteresis."""
from bisect import bisect_left

from .common import identifier, number, require_schema, review


def table(rows, name, minimum=3):
    if not isinstance(rows, list) or not minimum <= len(rows) <= 100:
        raise ValueError(f'{name} requires {minimum}–100 measurement points')
    points = sorted((number(r['position_rad'], 'position_rad', 0, 5),
                     number(r['width_mm'], 'width_mm', 0, 200)) for r in rows)
    if any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(points, points[1:])):
        raise ValueError(f'{name}: angle and width must both be strictly increasing')
    if points[-1][0] - points[0][0] < .1:
        raise ValueError(f'{name}: measurement span below 0.1rad')
    return points


def interpolate(points, value, inverse=False):
    pairs = [(w, q) for q, w in points] if inverse else points
    value = number(value, 'query')
    xs = [p[0] for p in pairs]
    if not xs[0] <= value <= xs[-1]:
        raise ValueError('query outside measured range; extrapolation is forbidden')
    i = bisect_left(xs, value)
    if i == 0:return pairs[0][1]
    x0, y0 = pairs[i - 1];x1, y1 = pairs[i]
    return y0 + (y1 - y0) * (value - x0) / (x1 - x0)


def analyze(data):
    require_schema(data, 'b601-j7-width-data/1.0')
    result = review('b601-j7-width-review/1.0', data)
    result.update({k:identifier(data.get(k), k) for k in ('calibration_id', 'device_id', 'gripper_id')})
    tolerance = number(data['max_validation_error_mm'], 'max_validation_error_mm', .01, 5)
    directions = {}
    for direction in ('opening', 'closing'):
        rows = table(data['measurements'][direction], direction)
        checks = data['validation'][direction]
        if not isinstance(checks, list) or not 2 <= len(checks) <= 100:
            raise ValueError('each direction needs at least two independent validation points')
        used = set();errors = []
        for row in checks:
            q = number(row['position_rad'], 'validation position_rad', 0, 5)
            if q in used or any(abs(q - x) < 1e-8 for x, _ in rows):
                raise ValueError('validation positions must be distinct from fit positions and each other')
            used.add(q)
            width = number(row['width_mm'], 'validation width_mm', 0, 200)
            predicted = interpolate(rows, q)
            errors.append(dict(position_rad=q, measured_width_mm=width,
                               predicted_width_mm=predicted, absolute_error_mm=abs(predicted - width)))
        maximum = max(e['absolute_error_mm'] for e in errors)
        directions[direction] = dict(table_rad_mm=rows, validation=errors,
            max_error_mm=maximum, validation_passed=maximum <= tolerance,
            position_range_rad=[rows[0][0], rows[-1][0]], width_range_mm=[rows[0][1], rows[-1][1]])
    result.update(directions=directions, max_validation_error_mm=tolerance,
                  validation_passed=all(v['validation_passed'] for v in directions.values()),
                  calibration_ready=False, method='bounded piecewise linear; no extrapolation',
                  limitations=['input provenance and physical measurements require onsite review',
                               'no force estimate, force control, current threshold or runtime API change'])
    return result


def query(report, direction, *, width_mm=None, position_rad=None):
    require_schema(report, 'b601-j7-width-review/1.0')
    if not report['validation_passed'] or direction not in ('opening', 'closing'):
        raise ValueError('a passing table and explicit opening/closing direction are required')
    if (width_mm is None) == (position_rad is None):
        raise ValueError('provide exactly one of width_mm or position_rad')
    points = report['directions'][direction]['table_rad_mm']
    if width_mm is not None:
        return dict(width_mm=width_mm, position_rad=interpolate(points, width_mm, inverse=True))
    return dict(position_rad=position_rad, width_mm=interpolate(points, position_rad))
