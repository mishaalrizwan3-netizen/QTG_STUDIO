"""Resolved Manual targets and reviewed applicability; no hardware or lifecycle."""
from dataclasses import dataclass, replace
import json
import math
import os
from types import MappingProxyType

from backend.initialization_plan_safety import PlanBlocked, identity
from backend.initialization_requirements import REQUIREMENTS, FIELDS
from backend.verifier import validate_conditions
from backend.registry import manual_configuration_resolver


MANUAL_PLANS = 'QTG_UNIVERSAL_MANUAL_PLANS'
AUTOMATIC_PLANS = 'QTG_UNIVERSAL_AUTOMATIC_PLANS'


def automatic_plans():
    """Explicit test -> execution mode -> reviewed plan; no implicit activation."""
    try:
        plans = json.loads(os.environ.get(AUTOMATIC_PLANS, '{}'))
        if not isinstance(plans, dict):
            raise ValueError('expected a test/mode mapping')
        for reference, modes in plans.items():
            if reference not in REQUIREMENTS or not isinstance(modes, dict) or not modes:
                raise ValueError('unknown reference or empty mode mapping')
            for mode, path in modes.items():
                if (mode not in ('recorded_replay', 'pid') or not isinstance(path, str)
                        or not path.strip() or (mode == 'pid' and reference != '1.c.(1)')):
                    raise ValueError('unsupported mode or missing plan path')
        return plans
    except (ValueError, TypeError) as exc:
        raise PlanBlocked(AUTOMATIC_PLANS + ': invalid explicit Automatic selection: ' + str(exc)) from exc


def manual_plans():
    """An explicit reference -> reviewed-plan path mapping; empty by default."""
    try:
        plans = json.loads(os.environ.get(MANUAL_PLANS, '{}'))
        if not isinstance(plans, dict) or any(
                ref not in REQUIREMENTS or not isinstance(path, str) or not path.strip()
                for ref, path in plans.items()):
            raise ValueError('expected known references and nonempty plan paths')
        return plans
    except (ValueError, TypeError) as exc:
        raise PlanBlocked(MANUAL_PLANS + ': invalid explicit Manual selection: ' + str(exc)) from exc


def resolve_configuration(context):
    """Resolve the existing source once. Zero is data, never a missing-value flag."""
    reference = context.get('test_reference')
    try:
        resolver = manual_configuration_resolver(reference)
    except ValueError as exc:
        raise PlanBlocked(str(exc)) from exc
    if context.get('weather_overrides'):
        raise PlanBlocked('weather_overrides has no existing Manual command interface; use the supported resolved initial conditions and review the change')
    source_hash = None
    if resolver is not None:
        try:
            conditions, source_hash, origins = resolver(context)
        except (ValueError, OSError) as exc:
            raise PlanBlocked(str(exc)) from exc
    else:
        configured = context.get('initial_conditions')
        if not isinstance(configured, dict):
            raise PlanBlocked('Selected Manual test has no resolved GUI/session initial conditions')
        # Validate explicitly supplied attitude too; do not turn validator defaults into targets.
        for key in ('heading_deg', 'pitch_deg', 'bank_deg', 'on_ground'):
            if key not in configured:
                raise PlanBlocked(key + ': configured initial condition missing')
        if type(configured['on_ground']) is not bool:
            raise PlanBlocked('on_ground: configured target must be a boolean')
        conditions = validate_conditions(configured)
        origins = {key: 'resolved GUI/session initial_conditions.' + key for key in conditions}
    if type(conditions.get('on_ground')) is not bool:
        raise PlanBlocked('on_ground: configured target must be a boolean')
    for key in FIELDS:
        if key in conditions and key != 'on_ground' and conditions[key] is not None:
            if isinstance(conditions[key], bool):
                raise PlanBlocked(key + ': configured target must be numeric, not a boolean')
            conditions[key] = float(conditions[key])
            if not math.isfinite(conditions[key]):
                raise PlanBlocked(key + ': configured target is nonfinite')
    if any(type(value) not in (str, bool, int, float, type(None)) for value in conditions.values()):
        raise PlanBlocked('Resolved configuration requires immutable scalar conditions')
    configuration = dict(source_sha256=source_hash, resolved_sha256=identity(conditions),
        overrides_sha256=identity(context.get('initial_condition_overrides') or {}),
        weather_overrides_sha256=identity(context.get('weather_overrides') or {}))
    if resolver is None:
        configuration['test_reference'] = reference
    provenance = {key: dict(source=origins[key], unit=FIELDS[key][0])
                  for key in conditions if key in FIELDS}
    return conditions, configuration, provenance


@dataclass(frozen=True)
class ResolvedConfiguration:
    reference: str
    conditions: object
    configuration: object
    provenance: object

    @classmethod
    def capture(cls, context):
        conditions, configuration, provenance = resolve_configuration(context)
        # Only schema measurements are accepted by commands/verification; all are scalar.
        return cls(context['test_reference'], MappingProxyType(dict(conditions)),
                   MappingProxyType(dict(configuration)), MappingProxyType({
                       key: MappingProxyType(dict(value)) for key, value in provenance.items()}))


ENGINE_MANDATORY = frozenset(('on_ground', 'alt_ft', 'ias_kt', 'throttle', 'flaps',
    'temperature_c', 'wind_speed', 'wind_direction', 'barometer', 'lat', 'lon',
    'heading_deg', 'pitch_deg', 'bank_deg'))


@dataclass(frozen=True)
class ManualPolicy:
    mandatory: frozenset
    physical_controls: bool
    stability_kind: str
    limits: object = None
    duration_s: float | None = None
    min_samples: int | None = None
    max_gap_s: float | None = None


def reviewed_manual_policy(document, reference, conditions=None):
    """Approval of the entire document also covers these applicability decisions."""
    spec = document.get('manual_policy')
    engine = reference in ('1.f.(1)', '1.f.(2)')
    if spec is None and engine:
        # Preserve the previous reviewed engine contract. This proves paused state only.
        return ManualPolicy(ENGINE_MANDATORY, True, 'paused_state')
    if (not isinstance(spec, dict) or spec.get('status') != 'CONFIGURED'
            or spec.get('preparation') != 'existing_manual'):
        raise PlanBlocked('A reviewed CONFIGURED manual_policy selecting existing_manual preparation is required')
    mandatory = spec.get('mandatory_checks')
    if (not isinstance(mandatory, list) or not mandatory or len(set(mandatory)) != len(mandatory)
            or any(key not in FIELDS for key in mandatory)):
        raise PlanBlocked('manual_policy: explicit valid mandatory_checks required')
    if engine and not ENGINE_MANDATORY <= set(mandatory):
        raise PlanBlocked('Existing engine mandatory readiness checks cannot be excluded')
    conditional = spec.get('wind_direction_applicability')
    if conditional is not None:
        if (not isinstance(conditional, dict)
                or conditional.get('kind') != 'not_applicable_when_configured_calm'
                or not isinstance(conditional.get('source'), str)
                or not conditional['source'].strip()):
            raise PlanBlocked('Reviewed calm-wind applicability kind/source required')
        speed = (conditions or {}).get('wind_speed')
        if type(speed) not in (int, float) or not math.isfinite(speed) or speed < 0:
            raise PlanBlocked('Calm-wind applicability requires a valid configured wind_speed')
        if speed == 0:
            exclusions = document.get('exclusions', {})
            reason = exclusions.get('wind_direction')
            if ('wind_speed' not in document.get('checks', {})
                    or 'wind_direction' in document.get('checks', {})
                    or not isinstance(reason, str) or not reason.strip()):
                raise PlanBlocked('Calm wind requires a wind-speed check and explicit reviewed direction exclusion')
            mandatory = (set(mandatory) | {'wind_speed'}) - {'wind_direction'}
        else:
            mandatory = set(mandatory) | {'wind_speed', 'wind_direction'}
    physical = spec.get('physical_controls')
    if physical not in ('read_only', 'not_required') or (engine and physical != 'read_only'):
        raise PlanBlocked('manual_policy: explicit physical-controls applicability required')
    stability = spec.get('stability')
    if not isinstance(stability, dict) or not isinstance(stability.get('source'), str) or not stability['source'].strip():
        raise PlanBlocked('manual_policy: reviewed stability scope/source required')
    kind = stability.get('kind')
    if kind == 'paused_state':
        return ManualPolicy(frozenset(mandatory), physical == 'read_only', kind)
    if kind != 'running_window':
        raise PlanBlocked('Unsupported Manual stability scope')
    limits = stability.get('limits')
    positive = lambda n: type(n) in (int, float) and math.isfinite(n) and n > 0
    if (not isinstance(limits, dict) or not limits or any(
            key not in document['checks'] or type(limit) not in (int, float)
            or not math.isfinite(limit) or limit < 0 for key, limit in limits.items())
            or not positive(stability.get('duration_s')) or not positive(stability.get('max_gap_s'))
            or type(stability.get('min_samples')) is not int or stability['min_samples'] < 3):
        raise PlanBlocked('Running stability requires reviewed channels, range limits, duration, sample count and maximum gap')
    return ManualPolicy(frozenset(mandatory), physical == 'read_only', kind,
        MappingProxyType(dict(limits)), stability['duration_s'], stability['min_samples'], stability['max_gap_s'])


def manual_registry(reference, conditions, policy):
    """Attempt-local reviewed policy; never promote global baseline registrations."""
    baseline = REQUIREMENTS[reference]
    if baseline.status == 'CONFIGURED':
        return REQUIREMENTS
    fields = {key: replace(item, value=conditions.get(key),
              source='resolved GUI/session configuration' if conditions.get(key) is not None else item.source,
              missing_reason=None if conditions.get(key) is not None else item.missing_reason)
              for key, item in baseline.requirements.items()}
    reviewed = replace(baseline, status='CONFIGURED', modes=('manual',),
        modes_source='explicitly approved Manual policy; attempt-local', requirements=fields)
    return {**REQUIREMENTS, reference: reviewed}


def automatic_registry(reference, mode, conditions, document):
    """Review mode applicability locally; targets always come from the recording."""
    if mode == 'pid' and reference != '1.c.(1)':
        raise PlanBlocked('PID Automatic is supported only for normal climb')
    baseline = REQUIREMENTS[reference]
    if reference in ('1.f.(1)', '1.f.(2)'):
        if mode != 'recorded_replay':
            raise PlanBlocked('Engine Automatic supports Recorded Replay only')
        return REQUIREMENTS, ENGINE_MANDATORY | {'engine_rpm', 'prop_rpm'}
    spec = document.get('automatic_policy', {})
    expected = 'existing_climb_pid' if mode == 'pid' else 'existing_recorded_replay'
    mandatory = spec.get('mandatory_checks')
    if (spec.get('status') != 'CONFIGURED' or spec.get('preparation') != expected
            or spec.get('stability_scope') != 'paused_state'
            or not isinstance(spec.get('source'), str) or not spec['source'].strip()
            or not isinstance(mandatory, list) or len(set(mandatory)) != len(mandatory)
            or not ENGINE_MANDATORY <= set(mandatory) or not set(mandatory) <= FIELDS.keys()):
        raise PlanBlocked('Explicit reviewed Automatic policy, preparation, paused scope and mandatory checks required')
    fields = {key: replace(item, value=conditions.get(key),
              source='selected Manual recording-start snapshot' if conditions.get(key) is not None else item.source,
              missing_reason=None if conditions.get(key) is not None else item.missing_reason)
              for key, item in baseline.requirements.items()}
    reviewed = replace(baseline, status='CONFIGURED', modes=(mode,),
        modes_source='approved Automatic policy; attempt-local', requirements=fields)
    return {**REQUIREMENTS, reference: reviewed}, frozenset(mandatory)
