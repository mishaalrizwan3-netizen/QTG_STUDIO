"""Per-test operator settings; no simulator or hardware operations."""
import json
import math
import os
from pathlib import Path
import re
import tempfile

FIELDS = ('Flt cond', 'Alt[ft]', 'IAS[kts]', 'Flaps', 'Latitude[deg]',
          'Longitude[deg]', 'Heading[deg]', 'Pitch[deg]', 'Bank[deg]',
          'Throttle[%]', 'Temperature[C]')

RANGES = {'Alt[ft]': (-1500, 60000), 'IAS[kts]': (0, 400), 'Flaps': (0, 10),
          'Latitude[deg]': (-90, 90), 'Longitude[deg]': (-180, 180),
          'Heading[deg]': (0, 360), 'Pitch[deg]': (-90, 90), 'Bank[deg]': (-90, 90),
          'Throttle[%]': (0, 100), 'Temperature[C]': (-100, 60)}


def validate(values, *, require_complete=False):
    missing = [key for key in FIELDS if not values.get(key, '').strip()]
    if require_complete and missing:
        raise ValueError('Initial conditions not set: ' + ', '.join(missing))
    for key, (low, high) in RANGES.items():
        raw = values.get(key, '').strip()
        if not raw:
            continue
        try:
            number = float(raw)
        except ValueError as exc:
            raise ValueError(key + ' must be a number') from exc
        if not math.isfinite(number) or not low <= number <= high:
            raise ValueError(key + ' must be between ' + str(low) + ' and ' + str(high))
        if key == 'Flaps' and number != int(number):
            raise ValueError('Flaps must be a whole-number handle index')
    return missing


def catalog_defaults(spec):
    """Only values actually present in this test's catalog entry are defaults."""
    values = dict.fromkeys(FIELDS, '')
    for item in spec.initial_conditions.split('|'):
        if ':' in item:
            key, value = item.split(':', 1)
            if key.strip() in values:
                values[key.strip()] = value.strip()
    return values


def _path(root, reference):
    if not re.fullmatch(r'[0-9]+(?:\.[a-z]+)*\.\([0-9]+\)', reference):
        raise ValueError('Invalid test reference')
    return Path(root) / (reference + '.json')


def load(root, reference):
    path = _path(root, reference)
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    if (not isinstance(data, dict) or data.get('reference') != reference
            or not isinstance(data.get('initial_conditions'), dict)):
        raise ValueError('Saved initial conditions do not match this test')
    values = data['initial_conditions']
    if any(key not in FIELDS or not isinstance(value, str) for key, value in values.items()):
        raise ValueError('Saved initial conditions contain unsupported fields')
    return values


def load_visible_fields(root, reference):
    path = _path(root, reference)
    if not path.exists():
        return list(FIELDS)
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    visible = data.get('visible_fields', list(FIELDS))
    if not isinstance(visible, list) or any(field not in FIELDS for field in visible) or len(set(visible)) != len(visible):
        raise ValueError('Saved initial condition fields are invalid')
    return visible


def save(root, reference, values, *, visible_fields=None):
    path = _path(root, reference)
    if set(values) != set(FIELDS) or any(not isinstance(v, str) for v in values.values()):
        raise ValueError('Initial conditions must contain the expected fields')
    validate(values)
    path.parent.mkdir(parents=True, exist_ok=True)
    if visible_fields is None:
        visible_fields = load_visible_fields(root, reference)
    if any(field not in FIELDS for field in visible_fields) or len(set(visible_fields)) != len(visible_fields):
        raise ValueError('Invalid initial condition fields')
    data = json.dumps({'reference': reference, 'initial_conditions': values,
                       'visible_fields': list(visible_fields)}, indent=2) + '\n'
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                         prefix=path.stem + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return path
