"""Engine QTG: original-Air-Manager passive yaw policy.

No new Air Manager instrument/script, vJoy feeder, UDP yaw relay, P3D
rudder-axis command or BRUNNER Yaw AP replay is created by this module.

The inherited climb logger continues to run Pitch/Roll BRUNNER replay.
Air Manager owns the ordinary rudder path on the SIM PC.  Physical yaw
and P3D rudder data may still be logged for engineering diagnostics,
but they are not compared to the manual test or used to command Yaw.
"""

class EngineAirManagerPassiveYaw:
    """Apply the engine-only Yaw exclusion before automatic hardware arming."""

    def _summarize_reference_replay(self):
        ok = super()._summarize_reference_replay()
        if not ok:
            return False
        if getattr(self, 'test_mode', None) == 'auto':
            self.reference_cls_axes = [
                axis for axis in self.reference_cls_axes if axis in ('Pitch', 'Roll')
            ]
            self.reference_has_native_cls = bool(self.reference_cls_axes)
            if not self.reference_has_native_cls:
                print('[ENGINE / AIR MANAGER] Automatic blocked: no recorded '
                      'Pitch/Roll BRUNNER axis. Yaw is excluded because original '
                      'Air Manager remains its owner.', flush=True)
                return False
            print('[ENGINE / AIR MANAGER] BRUNNER Automatic replay axes: '
                  + ', '.join(self.reference_cls_axes)
                  + '; Yaw untouched, original Air Manager owns P3D rudder.',
                  flush=True)
        return True

    def _set_active_cls_axes(self, names):
        if getattr(self, 'test_mode', None) == 'auto':
            names = [axis for axis in names if axis in ('Pitch', 'Roll')]
        return super()._set_active_cls_axes(names)
