"""V15.12.5 climb BRUNNER compatibility adapter for QTG derivatives.

The reference is the *user-supplied* compiled Super_Mushak_V15_12_5.exe,
PyInstaller's embedded Python 3.14 main module. The top-level settings and
method call names were checked against its actual marshalled bytecode.

This module is intended to reproduce V15.12.5's BRUNNER movement/endpoint
acceptance behavior. It does NOT prove physical tracking and must not be
interpreted as an EASA/QTG pass; V15.12.5's tolerances are permissive.

The original CLS2Sim UDP transport and the parent control/force/P3D replay
state machine are left unchanged. In particular: NO SimConnect flight-control
substitution, NO position offsets, NO disabled feedback/stale-telemetry guards.
"""
import math
import time

from climb_brunner import ClimbBrunnerApp
import Super_Mushak_Logger_V15_FULL_CAUSAL_STATE as climb
from climb_brunner_transport import SID_AP_POSITION, packet_setting_float

CLIMB_V15125_SETTINGS = {
    # Independently read from the uploaded working climb EXE's module assignments.
    'CLS_TRACK_PRECHECK_TOL': 0.12,
    'CLS_FOLLOW_WARN_ERROR': 0.20,
    'CLS_FOLLOW_ABORT_ERROR': 0.90,
}
# EXE and original logger share: AP forces 1800/1500/1500,
# AP speeds Pitch=300 Roll=200 Yaw selectable, precheck 6s/4s/0.4s,
# endpoint limit 0.95, command-settle delay 0.75, and force interlocks.


def enable_v15125_climb_settings():
    """Configure existing logger module for the compiled V15.12.5 thresholds.

    Call *before* creating a GUI, not while any BRUNNER test is running.
    This is process-wide because the inherited legacy logger reads globals.
    """
    for key, value in CLIMB_V15125_SETTINGS.items():
        setattr(climb, key, value)


class ClimbV15125BrunnerApp(ClimbBrunnerApp):
    """Reusable BRUNNER/P3D replay base implementing climb V15.12.5 precheck.

    Differences from the newer source are confined to two preflight methods
    and three EXE-verified threshold constants. All other automatic recording,
    force safety, telemetry and replay methods remain inherited unchanged.
    """
    def __init__(self, *args, **kwargs):
        enable_v15125_climb_settings()
        super().__init__(*args, **kwargs)

    def _command_and_wait_tracking_target(self, axis, name, target,
                                          hold_sec=None):
        """V15.12.5: send AP-position packet then assess measured position.

        The compiled climb method used cls.send(), NOT send_with_ack().
        This is why it accepted movement that stricter engine versions reject.
        The hardware watchdog, AP-enable and live feedback checks elsewhere
        in the logger still apply.
        """
        if hold_sec is None:
            hold_sec = climb.CLS_TRACK_PRECHECK_HOLD_SEC
        target = max(-climb.CLS_TRACK_PRECHECK_LIMIT,
                     min(climb.CLS_TRACK_PRECHECK_LIMIT, float(target)))
        self.cls.send(packet_setting_float(axis, SID_AP_POSITION, target))
        start = time.monotonic()
        best_actual = None
        best_error = float('inf')
        while time.monotonic() - start < float(hold_sec):
            actual = self._axis_actual_position(name)
            if actual is not None:
                actual = float(actual)
                err = abs(target - actual)
                if err < best_error:
                    best_error = err
                    best_actual = actual
                if err <= climb.CLS_TRACK_PRECHECK_TOL:
                    print(f'[TRACK PRECHECK] {name}: target={target:+.3f} '
                          f'actual={actual:+.3f} error={err:.3f} -> REACHED',
                          flush=True)
                    return {'target': target, 'actual': actual,
                            'error': err, 'reached': True}
            time.sleep(0.05)
        actual = self._axis_actual_position(name)
        if actual is not None:
            actual = float(actual)
            err = abs(target - actual)
        else:
            actual = best_actual
            err = best_error
        actual_text = f'{actual:+.3f}' if actual is not None else 'NA'
        err_text = f'{err:.3f}' if math.isfinite(err) else 'NA'
        print(f'[TRACK PRECHECK] {name}: target={target:+.3f} '
              f'actual={actual_text} error={err_text} -> NOT REACHED', flush=True)
        return {'target': target, 'actual': actual, 'error': err,
                'reached': False}

    def _run_tracking_endpoint_precheck(self, initial_pitch, initial_roll,
                                        initial_yaw):
        """V15.12.5 full endpoint/return sequence for each active axis.

        The compiled climb EXE does NOT include the newer source's
        `CLS_MIN_MEANINGFUL_MOTION_NORM` shortcut. This method therefore
        checks both endpoints even for narrow Manual position ranges.
        """
        if (not climb.CLS_TRACK_PRECHECK_ENABLED or not self.reference_replay
                or not self.active_cls_axes):
            return True
        init = dict(Pitch=float(initial_pitch), Roll=float(initial_roll),
                    Yaw=float(initial_yaw))
        print('[TRACK PRECHECK] START | axes=' + self._active_axis_summary()
              + ' | endpoint tests are separated by return-to-initial | keep E-stop accessible',
              flush=True)
        self.track_precheck_results = {}
        for name, axis, key in self._active_axis_specs():
            vals = []
            for frame in self.reference_replay:
                try:
                    v = frame.get(key)
                    if v is not None and math.isfinite(float(v)):
                        vals.append(float(v))
                except (TypeError, ValueError):
                    continue
            if not vals:
                self.track_precheck_results[name] = {'error': 'no native CLS data'}
                continue
            lo = max(-climb.CLS_TRACK_PRECHECK_LIMIT, min(vals))
            hi = min(climb.CLS_TRACK_PRECHECK_LIMIT, max(vals))
            t0 = max(-climb.CLS_TRACK_PRECHECK_LIMIT,
                     min(climb.CLS_TRACK_PRECHECK_LIMIT, init[name]))
            print(f'[TRACK PRECHECK] {name} manual range={min(vals):+.3f}..{max(vals):+.3f} '
                  f'| targets={lo:+.3f}/{hi:+.3f} | initial={t0:+.3f}',
                  flush=True)
            outcomes = {}
            for label, pos, hold in (
                ('start', t0, climb.CLS_TRACK_PRECHECK_RETURN_HOLD_SEC),
                ('negative', lo, climb.CLS_TRACK_PRECHECK_HOLD_SEC),
                ('return_after_negative', t0, climb.CLS_TRACK_PRECHECK_RETURN_HOLD_SEC),
                ('positive', hi, climb.CLS_TRACK_PRECHECK_HOLD_SEC),
                ('return_after_positive', t0, climb.CLS_TRACK_PRECHECK_RETURN_HOLD_SEC),
            ):
                outcomes[label] = self._command_and_wait_tracking_target(
                    axis, name, pos, hold_sec=hold)
                time.sleep(climb.CLS_TRACK_PRECHECK_INTER_TARGET_SETTLE_SEC)
            self.track_precheck_results[name] = outcomes
        failed = []
        for name, result in self.track_precheck_results.items():
            required = ('negative', 'positive', 'return_after_negative',
                        'return_after_positive')
            if ('error' in result or any(
                    not result[k].get('reached', False) for k in required)):
                failed.append(name)
        for name, axis, _key in self._active_axis_specs():
            self.cls.send(packet_setting_float(
                axis, SID_AP_POSITION,
                max(-climb.CLS_TRACK_PRECHECK_LIMIT,
                    min(climb.CLS_TRACK_PRECHECK_LIMIT, init[name]))))
        time.sleep(0.75)
        if failed:
            print('[TRACK PRECHECK RESULT] INCOMPLETE/FAIL: '
                  + ', '.join(failed)
                  + ' | this is now a genuine endpoint/return failure, '
                  'not the old extreme-to-extreme timeout', flush=True)
            print('[TRACK PRECHECK] END | active axes returned to initial point', flush=True)
            return False
        print('[TRACK PRECHECK RESULT] PASS for active axes: '
              + self._active_axis_summary(), flush=True)
        print('[TRACK PRECHECK] END | active axes returned to initial point', flush=True)
        return True
