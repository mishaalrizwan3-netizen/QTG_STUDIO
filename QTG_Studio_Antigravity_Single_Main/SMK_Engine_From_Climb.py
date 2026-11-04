"""Engine modes implemented as a narrow derivative of the working climb V15.12.5 app."""
import ctypes
import json
import math
import sys
import time
from pathlib import Path
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication,QLabel,QComboBox,QDoubleSpinBox,QGroupBox,QFormLayout,QMessageBox,QCheckBox
import Super_Mushak_Logger_V15_FULL_CAUSAL_STATE as climb
from climb_brunner_v15125 import ClimbV15125BrunnerApp
from engine_airmanager_passive_yaw import EngineAirManagerPassiveYaw
from p3d_simconnect_v15 import SimConnectClient,InitPositionStruct
from engine_rules_v2 import EngineTiming
from engine_control_diagnostics import stable_rpm_sample
from throttle_timeline import load_manual_throttle

# BRUNNER replay and initial alignment are inherited; the separate
# compatibility module restores the compiled V15.12.5 endpoint and follow limits. In particular, do not make the engine wrapper's physical
# t=0 gates tighter than the working climb app (0.08 bumpless and 0.03 t=0).
# Keep all qualification assessments based on measured Manual/Auto evidence.

# IMPORTANT: Use the native CLS2Sim transport from the proven climb logger.
# Do NOT monkey-patch Cls2SimUdpTransport or rewrite SID_AP_POSITION packets.
# The former EngineTransport / PositionAlignment layer modified the outgoing
# target during preflight (persisting its offset into replay), so the sent
# target could differ from the recorded Manual normalized position.
# Native climb path: recorded norm -> packet_setting_float(...,0x80,norm)
#                   -> CLS2Sim -> physical feedback -> P3D forwarding.


class EngineClient(SimConnectClient):
    def __init__(self,*args,**kwargs):
        self.ground_mode=False
        super().__init__(*args,**kwargs)
    @property
    def natural_engine_mode(self):return True
    @natural_engine_mode.setter
    def natural_engine_mode(self,value):pass  # No RPM seed in either engine-test mode.
    def queue_position_airborne(self,alt_ft,speed_kts,pitch_deg,bank_deg,hdg_deg,lat,lon,throttle=0):
        # Same native INITPOSITION call as climb; ground flag and preserved flaps are engine-specific.
        pos=InitPositionStruct(Latitude=lat,Longitude=lon,Altitude=alt_ft,
            Pitch=pitch_deg,Bank=bank_deg,Heading=hdg_deg,
            OnGround=int(self.ground_mode),Airspeed=0 if self.ground_mode else int(round(speed_kts)))
        self._bridge.SetUserPosition(ctypes.byref(pos));self.set_throttle(throttle)
        return True
    def set_position_airborne(self,alt_ft=3000,speed_kts=80,pitch_deg=0,bank_deg=0,
        hdg_deg=48.4,lat=climb.RAHWALI_LAT,lon=climb.RAHWALI_LON,throttle=0):
        self.queue_position_airborne(alt_ft,speed_kts,pitch_deg,bank_deg,hdg_deg,lat,lon,throttle)
        self.pause_sim()  # Retain the working climb Manual setup handoff.
        if self.ground_mode:
            d=self.get_telemetry()
            if not self.set_aux_configuration(d.get('fuel_tank_selector_1',1),0,
                d.get('auto_coordination',0),d.get('spoilers_handle_position',0),
                d.get('gear_handle_position',1),1,d.get('engine_combustion_1',1)):
                raise RuntimeError('Parking brake/configuration command failed')
        self.pause_sim()

class EngineApp(EngineAirManagerPassiveYaw, ClimbV15125BrunnerApp):
    def __init__(self):
        self.engine_timing=None;self.engine_kind='acceleration';self.engine_post=15
        self.engine_start={};self.engine_last_elapsed=0
        self.engine_tracking_failure = None
        self.engine_tracking_bad_since = None
        self.engine_p3d_diagnostic_run = False
        self.engine_max_ground_speed_kt = 0.0
        self.engine_control_mapping_mismatch = None
        self.engine_control_mapping_mismatch_count = 0
        self._decel_rpm_wait_since = None
        self._decel_rpm_last_seq = None
        super().__init__()
        self.setWindowTitle('Super Mushshak Engine V2.1.3 - Original Air Manager / Passive Yaw')
        self.data_dir=Path.home()/'Documents'/'SMK_Engine_From_Climb'/'acceleration'
        self.data_dir.mkdir(parents=True,exist_ok=True)
        self._refresh_manual_reference_selector()
        self.lbl_csv_path.setText(str(self.data_dir))
        self.lbl_timer.setText('Ready - timer starts only at Start Recording')
        self.btn_one_visit.hide()
        box=QGroupBox('Engine test - same Manual / Start Recording / Automatic workflow as climb')
        box.setStyleSheet('QGroupBox {color:#cdd6f4;} QLabel {color:#cdd6f4;}')
        form=QFormLayout(box)
        self.engine_selector=QComboBox();self.engine_selector.addItem('Engine Acceleration','acceleration')
        self.engine_selector.addItem('Engine Deceleration','deceleration')
        form.addRow('Test',self.engine_selector)
        self.engine_speed=self.spin(0,250,80);form.addRow('Initial IAS (kt)',self.engine_speed)
        self.engine_altitude=self.spin(0,30000,2000);form.addRow('Initial altitude MSL (ft)',self.engine_altitude)
        self.engine_seconds=self.spin(15,120,15);form.addRow('Seconds after throttle movement',self.engine_seconds)
        self.engine_parameter=QComboBox()
        self.engine_parameter.addItem('Manifold pressure (inHg) - power proxy', 'map')
        self.engine_parameter.addItem('Engine RPM - diagnostic (requires justification)', 'rpm')
        self.engine_critical_parameter = 'map'
        self.engine_parameter.currentIndexChanged.connect(
            lambda: setattr(self, 'engine_critical_parameter', self.engine_parameter.currentData()))
        form.addRow('Ti/Tt critical parameter', self.engine_parameter)
        self.engine_p3d_diagnostic_check=QCheckBox(
            'P3D-first diagnostic: record control mapping differences (not qualification)')
        # Climb-equivalent verification is the default. This switch can only
        # relax control-mapping checks after explicit operator opt-in.
        self.engine_p3d_diagnostic_check.setChecked(False)
        self.engine_p3d_diagnostic_check.setToolTip(
            'Acceleration: permits a P3D input/surface mapping mismatch ONLY in a labelled diagnostic run, while preserving other P3D and physical BRUNNER checks. '
            'Retains CLS physical, P3D pose, trim, engine and weather checks; only a *measured* control mapping mismatch is warning-only for acceleration. NOT a qualification run.')
        # Engine runs never gate start on P3D flight-control equality.
        # Retain the internal checkbox for older code paths, but hide it.
        self.engine_p3d_diagnostic_check.hide()
        mode_note=QLabel('Engine test: Yaw is NOT calibrated, replayed or compared; your original Air Manager script owns P3D rudder. Pitch/Roll use the climb BRUNNER replay, with hardware safety checks. Flight-control equality checks and control graphs are omitted; engine throttle/RPM checks remain active.')
        mode_note.setWordWrap(True)
        form.addRow(mode_note)
        note=QLabel('Ti/Tt calculated from high-rate engine channels. Manual/Automatic timing '
                    'agreement is NOT an EASA qualification pass.')
        note.setWordWrap(True);form.addRow(note)
        self.engine_cue=QLabel('1. Manual Testing   2. Start Recording   3. Advance throttle at MOVE NOW')
        self.engine_cue.setWordWrap(True);self.engine_cue.setStyleSheet('font-size:18px;font-weight:bold;color:#a6e3a1')
        form.addRow(self.engine_cue)
        self.centralWidget().layout().insertWidget(1,box)
        self.engine_selector.currentIndexChanged.connect(self.change_engine)
        self.resize(1050,900)
    def _scan_manual_reference_catalog(self):
        return [item for item in super()._scan_manual_reference_catalog()
                if '_engine_channels' not in str(item)]
    def _manual_reference_search_dirs(self):
        return [Path(self.data_dir)]
    def spin(self,lo,hi,value):
        w=QDoubleSpinBox();w.setRange(lo,hi);w.setValue(value);return w
    def change_engine(self):
        if self.is_recording or self.auto_prepare_in_progress:
            self.engine_selector.blockSignals(True)
            self.engine_selector.setCurrentIndex(0 if self.engine_kind=='acceleration' else 1)
            self.engine_selector.blockSignals(False);return
        self.engine_kind=self.engine_selector.currentData()
        self.data_dir=Path.home()/'Documents'/'SMK_Engine_From_Climb'/self.engine_kind
        self.data_dir.mkdir(parents=True,exist_ok=True);self.lbl_csv_path.setText(str(self.data_dir));self._refresh_manual_reference_selector()
        accel=self.engine_kind=='acceleration'
        self.engine_speed.setEnabled(accel);self.engine_altitude.setEnabled(accel)
        self.engine_cue.setText('Manual Testing -> Start Recording -> '+('ADVANCE' if accel else 'REDUCE')+' throttle at MOVE NOW')
    def start_manual_test(self):
        self.engine_kind=self.engine_selector.currentData();self.engine_post=self.engine_seconds.value()
        self.engine_start=dict(ias=self.engine_speed.value(),alt_msl=self.engine_altitude.value())
        self.engine_timing=None;self.engine_last_elapsed=0
        self.engine_max_ground_speed_kt = 0.0
        self._decel_start_stable_confirmed = False
        self._decel_start_waiting = False
        if getattr(self, "_decel_stability_timer", None): self._decel_stability_timer.stop()
        self.engine_selector.setEnabled(False)
        self.engine_cue.setText('Setting initial position; P3D will pause. Then press Start Recording.')
        super().start_manual_test()
    def _start_connection(self,*args,**kwargs):
        if self.sim_client is None:self.sim_client=EngineClient(app_name='SMK Engine derivative of V15.12.5')
        self.sim_client.ground_mode=self.engine_kind=='deceleration'
        if self.test_mode=='manual':
            kwargs.update(place_airborne=True,alt_ft=self.engine_start['alt_msl'],
                speed_kts=self.engine_start['ias'],initial_throttle=0)
            # Retain climb's known Rahwali starting location for airborne tests.
            # Ground deceleration uses the operator's loaded runway/parking location.
            if self.sim_client.ground_mode:
                kwargs.update(place_airborne=False)
        return super()._start_connection(*args,**kwargs)
    def _on_connection_success(self):
        if self.test_mode=='manual' and self.engine_kind=='deceleration':
            c=self.sim_client
            seq=c.get_telemetry_sequence();c.request_telemetry_once()
            d=c.wait_for_fresh_telemetry(seq,1)
            if not d or d.get('sim_on_ground',0)<.5:
                self._on_connection_error('Deceleration: load a running-engine flight at a ground parking/runway position first.')
                return
            c.set_position_airborne(alt_ft=d['pressure_alt'],speed_kts=0,
                pitch_deg=d['pitch_angle'],bank_deg=-d['bank_angle'],hdg_deg=d['true_heading'],
                lat=d['plane_latitude'],lon=d['plane_longitude'],throttle=0)
            self.engine_cue.setText('At idle: apply parking and wheel brakes. Unpause, increase power manually; remain stationary. Then Start Recording.')
        return super()._on_connection_success()
    def _capture_v1430_initial_dynamic_state(self,data):
        state=super()._capture_v1430_initial_dynamic_state(data)
        state.update(engine_test_kind=self.engine_kind,engine_test_post_seconds=self.engine_post,
            engine_test_base='climb V15.12.5')
        return state
    def _load_selected_manual_reference(self,path):
        if path is None:return False
        path=Path(path)
        meta_path=path.with_name(path.stem+'_engine_test').with_suffix('.json')
        try:
            metadata=json.loads(meta_path.read_text())
            if metadata['kind']!=self.engine_kind or metadata['status']!='complete':
                raise ValueError('Select a completed Manual for the selected engine test')
            self.engine_post=metadata['post_seconds']
            self.engine_throttle_timeline=load_manual_throttle(path)
        except Exception as exc:
            self.last_manual_reference_error=str(exc);return False
        return super()._load_selected_manual_reference(path)
    def _v1468_manual_throttle_at_elapsed(self, elapsed_sec):
        timeline = getattr(self, 'engine_throttle_timeline', None)
        if timeline is None:
            raise RuntimeError('High-rate Manual throttle was not loaded; Automatic cannot replay throttle')
        return timeline.at(elapsed_sec)
    def start_auto_test(self):
        self.engine_kind=self.engine_selector.currentData();self.engine_timing=None;self.engine_last_elapsed=0
        self.engine_max_ground_speed_kt = 0.0
        self.engine_tracking_failure = None
        self.engine_tracking_bad_since = None
        self.engine_control_mapping_mismatch = None
        self.engine_control_mapping_mismatch_count = 0
        self.engine_p3d_diagnostic_run = False  # flight-control mismatches never gate engine tests
        self.engine_selector.setEnabled(False)
        self.engine_p3d_diagnostic_check.setEnabled(False)
        super().start_auto_test()
        if not self.auto_prepare_in_progress and self.operating_stage in (climb.STAGE_NOT_CONNECTED,climb.STAGE_COMPLETED):
            self.engine_selector.setEnabled(True)
            self.engine_p3d_diagnostic_check.setEnabled(True)
    def _v1458_control_surfaces_match(self, data, context):
        """ENGINE-ONLY start gate: do not compare flight controls to Manual.

        There is deliberately NO pitch, roll, yaw, elevator/aileron/rudder
        surface, or trim *matching* test here. The engine reference still
        needs the correct throttle and RPM at t=0. This method is also used
        by the climb handover at live, paused and first-live checks.

        NOTE: this does not disable BRUNNER hardware interlocks, endpoint
        readiness, stale-feedback watchdogs, AP status or safe shutdown.
        These are operational safeguards for moving control loaders, not
        engine-response qualification channels.
        """
        state = dict(getattr(self, 'reference_initial_dynamic_state', {}) or {})
        d = data or {}
        parts = ['P3D flight-control/surface/trim equality checks OMITTED (engine test)']
        ok = True

        # Throttle is an ENGINE test input, not an unwanted control-surface gate.
        throttle_ref = self._v1430_finite(state.get('throttle_pct'))
        throttles = d.get('throttles', []) or []
        throttle_live = self._v1430_finite(
            throttles[0] if throttles else d.get('max_throttle'))
        if throttle_ref is not None and throttle_live is not None:
            err = float(throttle_live) - float(throttle_ref)
            passed = abs(err) <= float(climb.V14_58_THROTTLE_TOL_PCT)
            ok = ok and passed
            parts.append(f'Throttle={throttle_live:.2f}/{throttle_ref:.2f} '
                         f'd={err:+.2f} ({"OK" if passed else "FAIL"})')
        else:
            # Preserve inherited climb semantics: missing channels are
            # assessed later in engine telemetry/TiTt, not at this startup gate.
            parts.append('Throttle=MISSING (not assessed at startup)')

        # Measured engine RPM must not be silently synthesized or bypassed.
        rpm_ref = self._v1430_finite(state.get('engine_rpm'))
        rpms = d.get('eng_rpms', []) or []
        rpm_live = self._v1430_finite(rpms[0] if rpms else None)
        if rpm_ref is not None and rpm_live is not None:
            err = float(rpm_live) - float(rpm_ref)
            passed = abs(err) <= float(climb.V14_58_ENGINE_RPM_TOL)
            ok = ok and passed
            parts.append(f'RPM={rpm_live:.1f}/{rpm_ref:.1f} '
                         f'd={err:+.1f} ({"OK" if passed else "FAIL"})')
        else:
            # Missing engine telemetry must be evident to downstream QTG
            # measurement assessment; do not introduce an unrelated startup veto.
            parts.append('RPM=MISSING (not assessed at startup)')

        detail = ' | '.join(parts)
        self.auto_last_p3d_control_handover_detail = f'{context}: {detail}'
        return bool(ok), detail

    def _v1430_dynamic_matches(self, data, state):
        """ENGINE-ONLY: compare the dynamics, excluding flight-control trims.

        WORLD/BODY linear and rotational velocities remain verified. Trim
        mismatch is not an engine-response rejection. Actual trim still remains
        in the original raw commissioning/diagnostic CSV evidence.
        """
        checks = (
            ('velocity_world_x', 'velocity_world_x_fps', climb.V14_68_WORLD_XZ_TOL_FPS),
            ('velocity_world_y', 'velocity_world_y_fps', climb.V14_68_WORLD_Y_TOL_FPS),
            ('velocity_world_z', 'velocity_world_z_fps', climb.V14_68_WORLD_XZ_TOL_FPS),
            ('velocity_body_x', 'velocity_body_x_fps', climb.V14_68_BODY_XZ_TOL_FPS),
            ('velocity_body_y', 'velocity_body_y_fps', climb.V14_68_BODY_Y_TOL_FPS),
            ('velocity_body_z', 'velocity_body_z_fps', climb.V14_68_BODY_XZ_TOL_FPS),
            ('rotation_velocity_body_x', 'rotation_velocity_body_x_rad_s', climb.V14_30_ROTATION_TOL_RAD_S),
            ('rotation_velocity_body_y', 'rotation_velocity_body_y_rad_s', climb.V14_30_ROTATION_TOL_RAD_S),
            ('rotation_velocity_body_z', 'rotation_velocity_body_z_rad_s', climb.V14_30_ROTATION_TOL_RAD_S),
        )
        d = data or {}
        state = state or {}
        errs = []
        ok = True
        for live_key, ref_key, tol in checks:
            live = self._v1430_finite(d.get(live_key))
            ref = self._v1430_finite(state.get(ref_key))
            if live is None or ref is None:
                ok = False
                errs.append(f'{live_key}=MISSING')
            else:
                err = live - ref
                if abs(err) > float(tol):
                    ok = False
                errs.append(f'{live_key}:{err:+.4f}')
        return bool(ok), (' | '.join(errs)
                          + ' | flight-control trim equality OMITTED (engine test)')

    def _v1481_plot_failed_start(self, timestamp, reason):
        # Keep the original climb logger/BRUNNER module byte-for-byte intact.
        # Engine-only failure PDF excludes physical CLS control plots.
        from engine_failed_start_report import plot_engine_failed_start
        return plot_engine_failed_start(self, timestamp, reason)

    def _decel_stability_tick(self):
        """Nonblocking wait for ~2700 engine AND prop RPM for >=2.0 seconds."""
        if (self.test_mode != 'manual' or self.engine_kind != 'deceleration'
                or self.is_recording or not getattr(self, '_decel_start_waiting', False)):
            self._decel_start_waiting = False
            if getattr(self, '_decel_stability_timer', None):
                self._decel_stability_timer.stop()
            return
        c = self.sim_client
        if not c:
            self._decel_rpm_wait_since = None
            return
        try:
            prev = c.get_telemetry_sequence()
            if not c.request_telemetry_once():
                self._decel_rpm_wait_since = None
                self.engine_cue.setText('Waiting for fresh P3D engine telemetry')
                return
            d = c.wait_for_fresh_telemetry(prev, 0.12)
            passed, detail = stable_rpm_sample(d)
            seq = int((d or {}).get('bridge_telemetry_sequence', -1))
            if seq == self._decel_rpm_last_seq:
                passed, detail = False, 'Waiting for a new P3D engine frame'
            self._decel_rpm_last_seq = seq
            now = time.monotonic()
            if not passed:
                self._decel_rpm_wait_since = None
                self.engine_cue.setText('WAIT — ' + detail)
                return
            if self._decel_rpm_wait_since is None:
                self._decel_rpm_wait_since = now
            elapsed = now - self._decel_rpm_wait_since
            self.engine_cue.setText(f'RPM settling: {elapsed:.1f}/2.0 s | {detail}')
            if elapsed < 2.0:
                return
            self._decel_start_waiting = False
            self._decel_start_stable_confirmed = True
            self._decel_stability_timer.stop()
            self.engine_cue.setText('2700 RPM stable for 2 seconds — recording starts')
            self.arm_recording()
        except Exception as exc:
            self._decel_rpm_wait_since = None
            self.engine_cue.setText('Waiting for valid P3D RPM: ' + str(exc))

    def arm_recording(self):
        # Deceleration Manual: start recording ONLY after ~2700 RPM is stable.
        # This is a nonblocking timer, not a forced RPM write.
        if (self.engine_kind == 'deceleration' and self.test_mode == 'manual'
                and not self.is_recording and not getattr(self, '_decel_start_waiting', False)
                and not getattr(self, '_decel_start_stable_confirmed', False)):
            self._decel_start_waiting = True
            self._decel_rpm_wait_since = None
            self._decel_rpm_last_seq = None
            if not getattr(self, '_decel_stability_timer', None):
                self._decel_stability_timer = QTimer(self)
                self._decel_stability_timer.setInterval(140)
                self._decel_stability_timer.timeout.connect(self._decel_stability_tick)
            self._decel_stability_timer.start()
            self.engine_cue.setText('Waiting for engine and propeller RPM to settle near 2700 for 2 seconds...')
            return
        if self.engine_kind == 'deceleration' and self.test_mode == 'manual':
            self._decel_start_stable_confirmed = True
        # Let the inherited climb startup/bumpless/force lock verify physical
        # t=0. Do not add a second position gate that conflicts with climb's
        # commissioning tolerance and prevents its normal start sequence.
        if self.engine_kind == 'deceleration' and self.test_mode == 'manual':
            c = self.sim_client
            seq = c.get_telemetry_sequence(); c.request_telemetry_once()
            d = c.wait_for_fresh_telemetry(seq, 1.0)
            throttle = ((d or {}).get('throttles') or [0])[0]
            # Ground SPEED intentionally not gated in diagnostic deceleration mode.
            # Keep ground CONTACT, parking brake and throttle readiness checks.
            valid = (d and d.get('sim_on_ground', 0) > .5
                     and d.get('brake_parking_position', 0) > .9 and throttle >= 95)
            if not valid:
                QMessageBox.warning(self, 'Deceleration not ready',
                    'Deceleration setup requires ground contact, parking brake applied and throttle at or above 95%.\n'
                    'Ground speed is recorded but does not block the diagnostic test.')
                return
        auto_duration=self.target_duration_sec
        self.engine_timing=None;self.engine_last_elapsed=0;self.engine_cue_beeped=False
        super().arm_recording()
        if self.is_recording:
            self.target_duration_sec=auto_duration if self.test_mode=='auto' else 35
            self.expected_total_rows=self.target_duration_sec+1
            self.rec_progress.setRange(0,self.target_duration_sec)
            self.engine_cue.setText(
                'P3D DIAGNOSTIC recording - BRUNNER/P3D control mapping errors logged, not a QTG pass'
                if self.test_mode == 'auto' and self.engine_p3d_diagnostic_run else
                'Recording baseline - hold starting throttle for five seconds')
    def _format_data_row(self, sec, data):
        row = super()._format_data_row(sec, data)
        # Joystick force recording contract:
        # During Manual test: record applied force in Newtons (N).
        # During Automatic test: record virtual force in Newtons (N).
        j_vals = getattr(self, 'current_joystick_values', {}) or {}
        if self.test_mode == 'auto':
            def _vf_num(k1, k2, fallback):
                vf = j_vals.get(k1)
                if vf is None and hasattr(self, 'cls') and self.cls is not None:
                    vf = getattr(self.cls, k2, None)
                if vf is not None:
                    try:
                        fv = float(vf)
                        if math.isfinite(fv): return round(fv, 3)
                    except Exception: pass
                try:
                    fb = float(fallback)
                    if math.isfinite(fb): return round(fb, 3)
                except Exception: pass
                return 0.0

            p_vf = _vf_num('cls_pitch_virtual_force_eng', 'last_pitch_virtual_force_eng', row.get('Joystick Pitch Force (N)'))
            r_vf = _vf_num('cls_roll_virtual_force_eng', 'last_roll_virtual_force_eng', row.get('Joystick Roll Force (N)'))
            y_vf = _vf_num('cls_yaw_virtual_force_eng', 'last_rudder_virtual_force_eng', row.get('Rudder Pedal Force (N)'))

            row['Joystick Pitch Force (N)'] = p_vf
            row['Joystick Roll Force (N)'] = r_vf
            row['Rudder Pedal Force (N)'] = y_vf
            row['Pitch Virtual Force (N)'] = p_vf
            row['Roll Virtual Force (N)'] = r_vf
            row['Yaw Virtual Force (N)'] = y_vf
        return row
    def _make_legacy_plot_csv(self, filepath: Path) -> Path:
        out = super()._make_legacy_plot_csv(filepath)
        if self.test_mode == 'auto' and out.exists():
            try:
                import csv
                with open(out, 'r', encoding='utf-8') as f:
                    rows = list(csv.DictReader(f))
                if rows:
                    for rr in rows:
                        for ax in ('Pitch', 'Roll', 'Yaw'):
                            vf_eng = rr.get(f'CLS {ax} Virtual Force (eng)')
                            if vf_eng:
                                rr[f'{ax} Virtual Force (N)'] = vf_eng
                                force_col = 'Rudder Pedal Force (N)' if ax == 'Yaw' else f'Joystick {ax} Force (N)'
                                rr[force_col] = vf_eng
                    with open(out, 'w', encoding='utf-8', newline='') as f:
                        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                        w.writeheader()
                        w.writerows(rows)
            except Exception as e:
                print(f"[ENGINE LEGACY CSV VIRTUAL FORCE] {e}", flush=True)
        return out
    def _on_telemetry_tick(self):
        # For deceleration diagnostics, ground speed does not trigger an
        # automatic idle/pause during the Manual high-power preparation.
        if self.is_recording and self.sim_client:
            elapsed=max(0,time.time()-self.record_start_time)
            d=self.sim_client.get_telemetry()
            if self.engine_kind == 'deceleration':
                try:
                    gs = float(d.get('ground_speed', 0.0))
                    if math.isfinite(gs):
                        self.engine_max_ground_speed_kt = max(self.engine_max_ground_speed_kt, abs(gs))
                except (ValueError, TypeError):
                    pass
            throttle=(d.get('throttles') or [None])[0]
            if self.engine_timing is None:
                self.engine_timing=EngineTiming(self.engine_kind,self.engine_post)
                initial=(self.manual_t0_frame if self.test_mode=='manual' else getattr(self,'auto_t0_cls_values',{}))
                # Use P3D reference throttle, never CLS force or position.
                if self.test_mode=='auto':
                    baseline=(self.reference_initial_dynamic_state or {}).get('throttle_pct')
                else:baseline=(initial.get('throttles') or [None])[0]
                if baseline is not None:self.engine_timing.baseline=float(baseline)
            if throttle is not None:
                proposed_duration=self.engine_timing.update(elapsed,throttle)
                if self.test_mode=='manual':
                    self.target_duration_sec=proposed_duration
                    self.expected_total_rows=self.target_duration_sec+1
                    self.rec_progress.setRange(0,self.target_duration_sec)
            self.engine_last_elapsed=elapsed
            if self.test_mode=='manual':
                if self.engine_timing.event is not None:
                    self.engine_cue.setText('Recording engine response - keep flying' if self.engine_kind=='acceleration' else 'Recording engine response - remain stationary')
                elif elapsed>=5:
                    if not getattr(self,'engine_cue_beeped',False):self._beep_start();self.engine_cue_beeped=True
                    self.engine_cue.setText('MOVE NOW - '+('ADVANCE throttle to go-around power' if self.engine_kind=='acceleration' else 'REDUCE throttle to idle'))
            if self.engine_kind=='acceleration' and d.get('sim_on_ground',0)>.5:
                self.engine_cue.setText('STOPPED - touchdown during airborne engine test');self.stop_test();return
            # Ground contact remains required. Ground SPEED is deliberately
            # not an abort condition for this simulator diagnostic.
            if self.engine_kind=='deceleration' and d.get('sim_on_ground',0)<.5:
                self.sim_client.set_throttle(0)
                self.sim_client.pause_sim()
                self.engine_cue.setText('STOPPED - aircraft left the ground; throttle commanded idle and P3D paused');self.stop_test();return
        # All BRUNNER replay safety checks and physical tracking are owned by
        # the inherited climb logger, including stale feedback, AP disable,
        # consecutive UDP failures and settled following-error shutdown.
        # Do not impose an engine-only following-error/stop loop: climb's
        # position lookahead intentionally commands a future target, which can
        # transiently differ from its current-time Manual reference.
        return super()._on_telemetry_tick()
    @property
    def engine_run_metadata(self):
        result=(self.engine_timing.metadata() if self.engine_timing else dict(kind=self.engine_kind,
            post_seconds=self.engine_post,event_time_s=None,status='incomplete'))
        event=result.get('event_time_s')
        if event is None or self.engine_last_elapsed<event+self.engine_post:result['status']='incomplete'
        if self.engine_tracking_failure:
            result['tracking_diagnostic'] = self.engine_tracking_failure
        if self.engine_control_mapping_mismatch:
            result['p3d_control_mapping_mismatch'] = self.engine_control_mapping_mismatch
            result['qualification_eligible'] = False
            result['mapping_diagnostic_samples'] = self.engine_control_mapping_mismatch_count
        if self.test_mode == 'auto' and self.engine_p3d_diagnostic_run:
            result['p3d_diagnostic_only'] = True
            result['qualification_eligible'] = False
            result['cls_tracking_assessment'] = (
                'MISMATCH_RECORDED_NO_STOP' if self.engine_tracking_failure
                else 'NOT_CONFIRMED_FOR_QUALIFICATION')
            if result['status'] == 'complete':
                result['status'] = ('p3d_diagnostic_cls_mismatch' if self.engine_tracking_failure or self.engine_control_mapping_mismatch
                                    else 'p3d_diagnostic_cls_unverified')
            else:
                result['status'] = 'incomplete_p3d_diagnostic'
        elif self.engine_tracking_failure:
            result['status'] = 'invalid_cls_tracking'
        if self.engine_kind == 'deceleration':
            # Audit trail remains explicit even though the speed gate is disabled.
            result['ground_speed_check_enabled'] = False
            result['max_observed_ground_speed_kt'] = round(self.engine_max_ground_speed_kt, 3)
            result['ground_speed_test_basis'] = 'diagnostic only; aircraft may roll'
            result['qualification_eligible'] = False
        result.update(control_equality_assessed=False, control_graphs_in_report=False,
            last_elapsed_s=self.engine_last_elapsed,base='climb V15.12.5 compatible BRUNNER',
            brunner_precheck_tolerance_norm=0.12,
            brunner_follow_abort_tolerance_norm=0.90,
            brunner_control_accuracy_independently_verified=False,
            qualification_eligible=False,
            start_requests=dict(self.engine_start),active_cls_axes=list(self.active_cls_axes),
            critical_parameter_kind=self.engine_critical_parameter,
            yaw_p3d_authority='Original Air Manager / existing CLS2Sim vJoy (QTG does not write Yaw)',
            yaw_cls_hardware_qualification_assessed=False,
            yaw_qtg_calibration_enabled=False, yaw_qtg_replay_enabled=False,
            yaw_control_equalities_assessed=False,
            external_airmanager_rudder_effect_uncontrolled=True)
        return result
    def _on_report_complete(self,path):
        super()._on_report_complete(path)
        titt_str = ''
        try:
            p = Path(path)
            t_json = p.with_name(p.stem + '_TiTt_results.json')
            if not t_json.exists() and p.stem.endswith('_legacy_plot'):
                t_json = p.with_name(p.stem[:-12] + '_TiTt_results.json')
            if t_json.exists():
                d = json.loads(t_json.read_text())
                m = d.get('measurements', {}).get('Automatic') or d.get('measurements', {}).get('Manual')
                if m and m.get('Ti_s') is not None and m.get('Tt_s') is not None:
                    titt_str = (f" | Ti={m['Ti_s']:.2f}s, Tt={m['Tt_s']:.2f}s "
                                f"({d.get('status', m.get('status', ''))}; engineering only)")
        except Exception:
            pass
        self.engine_cue.setText('Finished - results saved. '+self.engine_run_metadata['status'] + titt_str)
        self.engine_cue_beeped=False;self.engine_selector.setEnabled(True)
        self.engine_p3d_diagnostic_check.setEnabled(True)

        # Sync immediately into QTG Studio's recorded_tests and test_logs.json
        try:
            cand_roots = [
                Path(__file__).resolve().parent,
                Path(__file__).resolve().parent.parent,
                Path(sys.executable).resolve().parent,
                Path(sys.executable).resolve().parent.parent,
            ]
            qtg_rec = None
            for cr in cand_roots:
                if (cr / "recorded_tests").is_dir():
                    qtg_rec = cr / "recorded_tests"
                    break
            if qtg_rec is None:
                qtg_rec = cand_roots[0] / "recorded_tests"
                qtg_rec.mkdir(parents=True, exist_ok=True)

            p_src = Path(path)
            if p_src.is_file():
                import shutil
                shutil.copy2(p_src, qtg_rec / p_src.name)
                for companion in p_src.parent.glob(p_src.stem[:18] + "*"):
                    if companion.is_file():
                        try:
                            shutil.copy2(companion, qtg_rec / companion.name)
                        except Exception:
                            pass

                log_file = qtg_rec / "test_logs.json"
                logs = []
                if log_file.is_file():
                    try:
                        logs = json.loads(log_file.read_text(encoding="utf-8"))
                    except Exception:
                        logs = []
                now_str = time.strftime("%Y-%m-%d %H:%M:%S")
                ref = "1.f.(1)" if self.engine_kind == "acceleration" else "1.f.(2)"
                title = "Engine acceleration" if self.engine_kind == "acceleration" else "Engine deceleration"
                mode = "Automatic Test" if self.test_mode == "auto" else "Manual Test"
                new_entry = {
                    "timestamp": now_str,
                    "year": "2026",
                    "reference": ref,
                    "title": title,
                    "category": "Performance",
                    "mode": mode,
                    "saved_in_yearly_result": True,
                    "pdf_name": p_src.name,
                    "pdf_path": str((qtg_rec / p_src.name).resolve())
                }
                # Check for existing
                if not any(e.get("pdf_name") == p_src.name for e in logs):
                    logs.insert(0, new_entry)
                    log_file.write_text(json.dumps(logs, indent=2), encoding="utf-8")
        except Exception as copy_err:
            print(f"[SMK Report Sync Note]: {copy_err}", flush=True)
    def stop_test(self):
        super().stop_test()
        if not self.recorded_rows:
            self.engine_selector.setEnabled(True)
            self.engine_p3d_diagnostic_check.setEnabled(True)
    def _on_connection_error(self,error_msg=''):
        super()._on_connection_error(error_msg)
        self.engine_selector.setEnabled(True)
        self.engine_p3d_diagnostic_check.setEnabled(True)

def main():
    app=QApplication(sys.argv);app.setStyle('Fusion');window=EngineApp();window.show();sys.exit(app.exec())
if __name__=='__main__':main()
