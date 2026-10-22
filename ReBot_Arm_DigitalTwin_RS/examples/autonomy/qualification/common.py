"""Shared input checks for the offline review file contracts."""
import json
import math
from pathlib import Path


def number(value, name, low=None, high=None):
    try:valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:valid = False
    if not valid:
        raise ValueError(f'{name} must be finite numeric data')
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(f'{name} outside [{low}, {high}]')
    return float(value)


def identifier(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 160:
        raise ValueError(f'{name} must be a nonempty identifier, at most 160 characters')
    return value


def source(data):
    value = data.get('source')
    if not isinstance(value, dict) or value.get('kind') not in ('synthetic', 'hardware', 'fake'):
        raise ValueError('source.kind must explicitly be synthetic, hardware or fake')
    return dict(value)


def review(schema, data):
    return dict(schema_version=schema, source=source(data), hardware_execution_allowed=False,
                physical_validation_passed=False, object_verified=False)


def require_schema(data, schema):
    if not isinstance(data, dict) or data.get('schema_version') != schema:
        raise ValueError(f'expected {schema}')


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 4 * 1024 * 1024:
        raise ValueError('offline input exceeds 4MB')
    def reject(value):
        raise ValueError(f'non-finite JSON number: {value}')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(path.read_text(), parse_constant=reject, object_pairs_hook=unique)
