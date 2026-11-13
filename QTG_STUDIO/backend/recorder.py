"""Measured worker status snapshot for the existing GUI/worker JSON contract."""

import math
import time


def snapshot(window, context):
    def label(name):
        widget=getattr(window,name,None)
        return widget.text() if widget is not None else ''
    telemetry={}
    for key,value in getattr(window,'latest_telemetry',{}).items():
        if isinstance(value,(int,float)):
            telemetry[key]=value if math.isfinite(value) else None
    return dict(context,stage=getattr(window,'current_stage','Connecting'),
        details=getattr(window,'_stage_details',''),updated=time.time(),
        connected=bool(window.p3d and window.p3d.is_connected()),
        brunner_connected=bool(window.brunner.sock),
        connection_text=label('lbl_conn_status'),throttle_text=label('lbl_throttle_val'),
        engine_text=label('lbl_engine_rpm'),flight_text=label('lbl_ias_alt'),
        attitude_text=label('lbl_pitch_bank'),controls_text=label('lbl_brunner_sticks'),
        recording_text=label('lbl_rec_status'),progress_text=label('lbl_rec_progress'),
        elapsed=window.rec_time[-1] if window.rec_time else 0,
        samples=len(window.rec_time),duration_s=window.target_duration_sec,
        pid_ready=getattr(window,'pid_ready',False),pid_paused=getattr(window,'pid_paused',True),pid_phase=getattr(window,'pid_phase',''),pid_live=getattr(window,'pid_live',{}),
        controlling=bool(window.is_recording or getattr(window,'pid_controller_active',False)),recording=bool(window.is_recording and (getattr(window,'_control_method','replay')!='pid' or getattr(window,'pid_phase','')=='recording')),can_record=bool(getattr(window,'_control_method','replay')!='pid' and getattr(window,'_prepared',False)
            and window.current_stage=='Connected & Ready to Record' and not window.is_recording
            and window.p3d and window.p3d.is_connected() and window.brunner.sock) or bool(not getattr(window,'pid_automatic_workflow',False) and getattr(window,'pid_controller_active',False) and not window.is_recording and not getattr(window,'pid_paused',True)),
        data_directory=str(window.data_dir),outputs=list(getattr(window,'_outputs',[])),telemetry=telemetry)


