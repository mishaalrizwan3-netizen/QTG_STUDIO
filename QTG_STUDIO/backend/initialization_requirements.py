"""Evidence inventory only: not imported by runtime initialization.

CONFIGURED means an existing test-specific baseline policy is documented, not
that all physical conditions are known or that a future manager may execute it.
Missing fields and conflicts remain explicit. No qualification tolerances are
reused as initialization tolerances. Replay targets always come from the chosen
Manual reference, not the Manual/PID defaults recorded here.
"""
import ast
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path

from backend.catalog import TESTS


@dataclass(frozen=True)
class Requirement:
    value: object
    unit: str
    telemetry: str
    source: str | None
    missing_reason: str | None = None
    # Existing verification behavior only; never automatically applied.
    tolerance: float | None = None
    tolerance_source: str | None = None
    scope: str = 'Manual baseline; Replay uses selected Manual reference'


@dataclass(frozen=True)
class InitializationPolicy:
    reference: str
    name: str
    modes: tuple[str, ...]
    modes_source: str | None
    status: str
    requirements: dict[str, Requirement]
    notes: tuple[str, ...] = ()

    @property
    def missing_requirements(self):
        return tuple(key for key, item in self.requirements.items() if item.value is None)


FIELDS = {
    'on_ground': ('bool', 'sim_on_ground'),
    'alt_ft': ('ft MSL', 'pressure_alt (driver PLANE ALTITUDE/MSL)'),
    'ias_kt': ('kt', 'airspeed_indicated'),
    'ground_speed': ('kt', 'ground_speed'),
    'lat': ('deg', 'plane_latitude'), 'lon': ('deg', 'plane_longitude'),
    'heading_deg': ('deg true', 'true_heading'),
    'pitch_deg': ('deg', 'pitch_angle'), 'bank_deg': ('deg', 'bank_angle'),
    'throttle': ('%', 'engine_throttle_percent'),
    'engine_rpm': ('rpm', 'engine_rpm'), 'prop_rpm': ('rpm', 'prop_rpms[0]'),
    'manifold_pressure': ('psi', 'engine_manifold_pressure_psi_1'),
    'mixture': ('%', 'engine_mixture_percent'), 'propeller_lever': ('%', 'engine_propeller_percent'),
    'temperature_c': ('deg C', 'ambient_temp'),
    'wind_speed': ('kt', 'ambient_wind_spd'), 'wind_direction': ('deg', 'ambient_wind_dir'),
    'barometer': ('hPa', 'baro_pressure'),
    'weight': ('kg', 'total_weight'), 'cg': ('% MAC', 'cg_percent'),
    'flaps': ('handle index', 'measured_flaps_handle_index'),
    'elevator_trim': ('rad', 'trim_elevator_angle_rad'),
    'aileron_trim': ('%', 'trim_aileron_percent'), 'rudder_trim': ('%', 'trim_rudder_percent'),
}


def _literal(path, name):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise ValueError(f'Missing literal {name} in {path}')


def build_registry(root=None):
    """Read current evidence without importing procedures or hardware modules."""
    root = Path(root) if root is not None else Path(__file__).resolve().parents[1]
    policies = {}
    for spec in TESTS:
        fields = {key: Requirement(None, unit, channel, None,
                    'No approved test-specific target established in inspected configuration')
                  for key, (unit, channel) in FIELDS.items()}
        policies[spec.reference] = InitializationPolicy(spec.reference, spec.title, (), None,
                                                       'UNCONFIGURED', fields)

    def populate(ref, values, source, modes, status, notes):
        old = policies[ref]
        fields = dict(old.requirements)
        for key, value in values.items():
            if key in fields:
                unit, channel = FIELDS[key]
                fields[key] = Requirement(value, unit, channel, source)
        policies[ref] = InitializationPolicy(ref, old.name, modes,
            'qtg_common_engine.py:initialize_mode; backend/control.py:validate_method',
            status, fields, tuple(notes))

    accel_source = 'test_suites/engine_acceleration/procedure.py:DEFAULT_INITIAL_CONDITIONS,DEFAULT_CONDITIONS'
    path = root / 'test_suites/engine_acceleration/procedure.py'
    accel = _literal(path, 'DEFAULT_INITIAL_CONDITIONS')
    accel['temperature_c'] = float(_literal(path, 'DEFAULT_CONDITIONS')['Temperature[C]'])
    accel['on_ground'] = False
    populate('1.f.(1)', accel, accel_source, ('manual', 'recorded_replay'), 'CONFIGURED', (
        'Airborne: catalog Approach or Landing and existing airborne engine policy.',
        'Coordinate conflict: definition.json gives 32.235/74.135; procedure gives 32.224/74.1095. No runtime precedence changed.',
        'CONFIGURED documents existing baseline only; missing fields still need reference-specific evidence.',))
    source = 'test_suites/engine_deceleration/initialization.json'
    decel = json.loads((root / source).read_text(encoding='utf-8'))
    if decel.get('test_reference') != '1.f.(2)':
        raise ValueError('Deceleration configuration reference mismatch')
    populate('1.f.(2)', decel, source, ('manual', 'recorded_replay'), 'CONFIGURED', (
        'Existing file-backed defaults; operator overrides remain authoritative at runtime.',
        'Zero IAS does not establish zero ground speed or an event-start state.',
        'CONFIGURED documents existing baseline only; missing fields still need reference-specific evidence.',))
    for ref in ('1.f.(1)', '1.f.(2)'):
        policy = policies[ref]
        fields = dict(policy.requirements)
        for key, tolerance in (('alt_ft', 25.), ('throttle', 1.), ('flaps', .1)):
            fields[key] = replace(fields[key], tolerance=tolerance,
                tolerance_source='backend/initializer.py:apply_manual_conditions (flaps UP check skipped)')
        fields['engine_rpm'] = replace(fields['engine_rpm'],
            tolerance=400. if ref == '1.f.(1)' else 60.,
            tolerance_source='test_suites/_shared/engine_initialization.py:install startup_rpm_tolerance',
            missing_reason='No fixed RPM target: selected Manual snapshot rpm required for Replay',
            scope='Recorded Replay final startup check only; not a fixed Manual target')
        policies[ref] = replace(policy, requirements=fields, notes=policy.notes + (
            'Replay RPM/prop targets: selected Manual snapshot; never infer a fixed target from tolerance.',
            'Acceleration engine_restore_rpm uses 350 RPM normally and 400 RPM engine-only fallback; Deceleration uses 60 RPM for both shafts.',
            'Other channel tolerances remain unspecified here; catalog manoeuvre-result tolerances are not initialization limits.',))
    populate('1.c.(1)', dict(alt_ft=3000., ias_kt=80., flaps=0),
             'test_suites/normal_climb/definition.json:initial_conditions_catalog',
             ('manual', 'recorded_replay', 'pid'), 'UNCONFIGURED', (
                 'Partial catalog example; heading, attitude, weather, weight, CG and trim not fully established.',
                 'PID-only defaults are recorded separately below; not promoted to Manual/Replay defaults.',))
    climb = policies['1.c.(1)']
    # Evidence from options(); these values apply only to PID defaults.
    tree = ast.parse((root / 'test_suites/normal_climb/procedure.py').read_text(encoding='utf-8'))
    options = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'options')
    assignment = options.body[0].value
    pid_values = {k.arg: ast.literal_eval(k.value) for k in assignment.keywords}
    fields = dict(climb.requirements)
    for key in ('lat', 'lon', 'throttle', 'temperature_c'):
        unit, channel = FIELDS[key]
        fields[key] = Requirement(pid_values[key], unit, channel,
            'test_suites/normal_climb/procedure.py:options', scope='PID default only')
    policies['1.c.(1)'] = InitializationPolicy(climb.reference, climb.name, climb.modes,
        climb.modes_source, climb.status, fields, climb.notes)
    return policies


def validate_policy(policy):
    """Schema validation only. Incompleteness is reported, never a runtime gate."""
    errors = []
    if not policy.reference or not policy.name:
        errors.append('Reference and name required')
    if policy.status not in ('CONFIGURED', 'UNCONFIGURED'):
        errors.append('Invalid policy status')
    if any(mode not in ('manual', 'recorded_replay', 'pid') for mode in policy.modes):
        errors.append('Invalid initialization mode')
    if policy.modes and not policy.modes_source:
        errors.append('Supported modes require evidence')
    if set(policy.requirements) != set(FIELDS):
        errors.append('Requirement fields do not match schema')
    for key, item in policy.requirements.items():
        if not item.unit or not item.telemetry or not item.scope:
            errors.append(f'{key}: unit, telemetry and scope required')
        if item.value is None:
            if not item.missing_reason:
                errors.append(f'{key}: missing target must have a reason')
        elif not item.source:
            errors.append(f'{key}: target requires source')
        elif isinstance(item.value, (float, int)) and not math.isfinite(item.value):
            errors.append(f'{key}: nonfinite target')
        if item.tolerance is not None and (
                not math.isfinite(item.tolerance) or item.tolerance < 0 or not item.tolerance_source):
            errors.append(f'{key}: tolerance requires finite nonnegative value and evidence')
    if policy.status == 'CONFIGURED' and (not policy.modes or any(
            policy.requirements.get(key) is None or policy.requirements[key].value is None
            for key in ('on_ground', 'alt_ft', 'ias_kt', 'throttle', 'flaps'))):
        errors.append('Configured baseline requires modes, flight condition, altitude, IAS, throttle and flaps')
    return errors


REQUIREMENTS = build_registry()
