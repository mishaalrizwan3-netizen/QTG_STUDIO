"""Opt-in observer beside an already-running Deceleration worker. Never starts a test."""
import argparse
import ctypes
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from backend.brake_recording import FLAG, align_capture, digest, eligible


class CaptureLease:
    """One observer per worker context; OS releases ownership if the caller dies."""
    def __init__(self, path): self.path, self.handle = str(Path(path).resolve()), None
    def __enter__(self):
        if os.name != 'nt': raise ValueError('Live capture requires Windows QPC')
        import hashlib
        self.kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        self.kernel.CreateMutexW.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_wchar_p]
        self.kernel.CreateMutexW.restype=ctypes.c_void_p
        self.kernel.CloseHandle.argtypes=[ctypes.c_void_p]
        self.kernel.ReleaseMutex.argtypes=[ctypes.c_void_p]
        ctypes.set_last_error(0)
        self.handle=self.kernel.CreateMutexW(None,1,'Local\\QTG_BrakeCapture_'+hashlib.sha256(self.path.lower().encode()).hexdigest()[:24])
        if not self.handle: raise OSError('Cannot acquire capture ownership')
        if ctypes.get_last_error()==183:
            self.kernel.CloseHandle(self.handle);self.handle=None
            raise ValueError('A brake observer already owns this worker context')
        return self
    def __exit__(self,*args):
        if self.handle:
            self.kernel.ReleaseMutex(self.handle);self.kernel.CloseHandle(self.handle);self.handle=None


def read_status(context):
    status=json.loads(Path(context['status_file']).read_text(encoding='utf-8-sig'))
    if status.get('command_directory') != context['command_directory']:
        raise ValueError('Worker identity does not match context')
    if not 0 <= time.time()-status.get('updated',0) <= 5:
        raise ValueError('Worker heartbeat unavailable/stale')
    if not eligible(status):
        raise ValueError('Only Engine Deceleration Manual / Recorded Replay is supported')
    if status.get('universal_initialization_enabled'):
        raise ValueError('Universal Initialization must remain disabled')
    return status


class NativeOwner:
    """Own child process and private stop file only; no worker commands or handles."""
    def __init__(self, executable, folder, duration_s, *, popen=subprocess.Popen):
        self.folder = Path(folder)
        self.stop_file = self.folder/'observer.stop'
        self.console = (self.folder/'observer_console.log').open('x',encoding='utf-8')
        try:
            self.process = popen([str(executable),'--connect-read-only',str(self.folder/'native'),
                '--duration-s',str(duration_s),'--parking-only','--stop-file',str(self.stop_file),
                '--owner-pid',str(os.getpid())],stdout=self.console,stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        except Exception:
            self.console.close(); raise

    def stop(self):
        self.stop_file.touch(exist_ok=True)
        try:
            self.process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            # Only our child is terminated. Never select a worker by name/PID.
            self.process.terminate()
            self.process.wait(timeout=3.)
        finally:
            self.console.close()


def choose_csv(status, recording_id):
    for raw in status.get('outputs',[]):
        path = Path(raw)
        if path.suffix.lower() == '.csv' and path.is_file():
            try:
                with path.open(encoding='utf-8-sig') as stream:
                    header = stream.readline()
                metadata=json.loads(path.with_suffix('.qtg_test.json').read_text(encoding='utf-8-sig'))
            except (OSError,ValueError): continue
            if (all(key in header for key in ('Time (s)','Throttle 1 Position (%)','Ground Speed (kt)'))
                    and (metadata.get('parking_brake_recording_anchor') or {}).get('recording_id')==recording_id):
                return path
    return None


class SavedRecordings:
    """Watch only new completed-run metadata, never the live worker status file."""
    def __init__(self, status):
        self.status=status
        self.directory=Path(status['result_directory'])
        self.previous=set(self.directory.glob('*.qtg_test.json'))

    def __call__(self):
        for path in sorted(set(self.directory.glob('*.qtg_test.json'))-self.previous):
            try:
                metadata=json.loads(path.read_text(encoding='utf-8-sig'))
            except (OSError,ValueError):continue # Metadata may still be being saved.
            if (not eligible(metadata) or metadata.get('command_directory')!=self.status['command_directory']
                    or metadata.get('mode')!=self.status['mode']):continue
            anchor=metadata.get('parking_brake_recording_anchor') or {}
            csv_path=path.with_name(path.name[:-len('.qtg_test.json')]+'.csv')
            if not anchor.get('recording_id'):continue
            selected=choose_csv(dict(outputs=[str(csv_path)]),anchor['recording_id'])
            if selected is not None:return selected,metadata
        return None


def capture(context_path, executable, output, **kwargs):
    context=json.loads(Path(context_path).read_text(encoding='utf-8-sig'))
    with CaptureLease(context['status_file']):
        return _capture(context_path,executable,output,**kwargs)


def _capture(context_path, executable, output, *, duration_s=180., max_age_s=1.,
            owner_factory=NativeOwner, clock=time.monotonic, sleep=time.sleep, status_reader=read_status,
            metadata_reader=None):
    if not 0 < duration_s <= 600 or not 0 < max_age_s < float('inf'):
        raise ValueError('Finite diagnostic lifetime (<=600 s) and positive age required')
    if os.environ.get(FLAG) != '1':
        raise ValueError('Explicit QTG opt-in required: set '+FLAG+'=1 before launching QTG and this caller')
    for key in ('QTG_UNIVERSAL_ENGINE_ACCELERATION','QTG_UNIVERSAL_ENGINE_DECELERATION'):
        if os.environ.get(key) not in (None,'0',''):
            raise ValueError('Both Universal Initialization switches must be disabled')
    context = json.loads(Path(context_path).read_text(encoding='utf-8-sig'))
    first = status_reader(context)
    if first.get('recording') or not first.get('connected'):
        raise ValueError('Attach before recording, with a connected idle QTG worker')
    # Pin the additive, paired-capable executable by its local build manifest.
    executable = Path(executable).resolve()
    manifest = json.loads(executable.with_name('manifest.json').read_text())
    if manifest.get('parking_pair_protocol') != 1 or manifest['executable_sha256'] != digest(executable):
        raise ValueError('A reviewed paired-parking diagnostic build is required')
    folder = Path(output)/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid4().hex[:8])
    folder.mkdir(parents=True,exist_ok=False)
    record = dict(context_file=str(Path(context_path).resolve()), context_sha256=digest(context_path),
        worker_status=context['status_file'], diagnostic_only=True, executable=str(executable),
        executable_sha256=manifest['executable_sha256'], max_age_s=max_age_s, anchor=None,
        reason='Waiting for recording clock marker', wheel_brakes='NOT REQUESTED: decoding unresolved')
    previous = (first.get('parking_brake_recording_anchor') or {}).get('recording_id')
    metadata_reader=metadata_reader or SavedRecordings(first)
    record['command_directory']=context['command_directory']
    record['capture_begin_perf_counter']=time.perf_counter()
    native_ready = False
    owner = None
    latest, qtg_csv = first, None
    deadline = clock()+duration_s
    try:
        owner = owner_factory(executable,folder,duration_s)
        print('Brake evidence:',folder.resolve(),flush=True)
        print('Buffering before operator recording; waiting for saved run metadata. No QTG command will be sent.',flush=True)
        while clock() < deadline:
            if not native_ready:
                candidates=list((folder/'native').glob('brakes_*/brake_samples.csv'))
                if len(candidates)==1:
                    try:
                        import csv
                        with candidates[0].open(newline='') as stream:
                            native_ready=any(r.get('simvar')=='BRAKE PARKING POSITION'
                                and r.get('status')=='NEW_CORRELATED_RESPONSE' and r.get('value') in ('0','1')
                                for r in csv.DictReader(stream))
                    except OSError: pass
                    if native_ready:
                        print('Native parking response observed. Operator recording remains controlled by QTG.',flush=True)
            saved=metadata_reader()
            if saved is not None:
                qtg_csv,latest=saved
                anchor=latest.get('parking_brake_recording_anchor') or {}
                if anchor.get('recording_id')==previous:
                    raise ValueError('Saved metadata reused a previous recording marker')
                if anchor.get('mode') != latest['mode'] or anchor.get('test_reference') != '1.f.(2)':
                    raise ValueError('Recording marker test/mode mismatch')
                if anchor.get('t0_perf_counter',0)<record['capture_begin_perf_counter']:
                    raise ValueError('Recording started before observer attachment')
                record['anchor'] = dict(anchor)
                record['mode'] = latest['mode']
                record['selected_reference'] = latest.get('reference_csv')
                if latest.get('reference_csv'):
                    path = Path(latest['reference_csv'])
                    record['reference_sha256'] = digest(path)
                    side = path.with_name(path.stem+'_init_conditions.json')
                    record['reference_sidecar_sha256'] = digest(side) if side.is_file() else None
                print('Recording clock captured:',anchor['recording_id'],flush=True)
                record['reason']='Recording saved'
                break
            if owner.process.poll() is not None:
                record['reason'] = 'Observer exited; inspect native summary. QTG recording was not stopped.'
                break
            sleep(.5)
        else:
            record['reason'] = 'Diagnostic lifetime expired; no QTG stop command sent'
    except KeyboardInterrupt:
        record['reason'] = 'Operator cancelled diagnostic only'
    except Exception as exc:
        record['reason'] = 'Diagnostic failure: '+str(exc)
    finally:
        record['capture_end_perf_counter'] = time.perf_counter()
        if record['anchor'] is not None:
            record['anchor']['capture_end_perf_counter'] = record['capture_end_perf_counter']
        if owner is not None:
            try: owner.stop()
            except Exception as exc: record['cleanup_error'] = str(exc)
        # Every file is private to this new folder. Never publish worker status.
        raw = list((folder/'native').glob('brakes_*/brake_samples.csv'))
        if len(raw)==1: record['native_csv']=str(raw[0].resolve())
        record['observer_exit_code'] = owner.process.poll() if owner is not None else None
        if qtg_csv is not None and record.get('native_csv'):
            try:
                report=align_capture(record['native_csv'],record['anchor'],qtg_csv,folder,max_age_s=max_age_s)
                record['qtg_csv']=str(qtg_csv.resolve())
                record['alignment']='SAVED_DIAGNOSTIC_ONLY'
                record['valid_pairs']=report['native_valid_pairs']
                print('Valid native pairs:',record['valid_pairs'],'; inspect start/event gaps in alignment JSON.',flush=True)
            except Exception as exc: record['alignment_error']=str(exc)
        with (folder/'capture.json').open('x',encoding='utf-8') as stream:
            json.dump(record,stream,indent=2,allow_nan=False)
        print(record['reason'],flush=True)
        print('Evidence:',folder.resolve(),flush=True)
        if 'qtg_csv' not in record:
            print('Alignment unavailable/pending; use the offline align command after the CSV is saved.',flush=True)
    return record


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='action',required=True)
    start=commands.add_parser('start')
    start.add_argument('--context',required=True)
    start.add_argument('--executable',required=True)
    start.add_argument('--output',required=True)
    start.add_argument('--duration-s',type=float,default=180.)
    start.add_argument('--max-age-s',type=float,default=1.)
    align=commands.add_parser('align')
    align.add_argument('--capture',required=True)
    align.add_argument('--csv',required=True)
    args=parser.parse_args(argv)
    try:
        if args.action=='start':
            record=capture(args.context,args.executable,args.output,duration_s=args.duration_s,max_age_s=args.max_age_s)
            return 0 if (record.get('valid_pairs',0)>0 and record.get('observer_exit_code')==0
                         and not record.get('cleanup_error')) else 1
        record=json.loads(Path(args.capture).read_text())
        if record.get('anchor') is None:
            metadata=json.loads(Path(args.csv).with_suffix('.qtg_test.json').read_text())
            anchor=metadata.get('parking_brake_recording_anchor') or {}
            if (not eligible(metadata) or metadata.get('command_directory')!=record.get('command_directory')
                    or not record['capture_begin_perf_counter']<=anchor.get('t0_perf_counter',0)<=record['capture_end_perf_counter']):
                raise ValueError('No matching recording clock anchor within this capture interval')
            record['anchor']=dict(anchor,capture_end_perf_counter=record['capture_end_perf_counter'])
        folder=Path(args.capture).parent/('alignment_'+uuid4().hex[:8]);folder.mkdir()
        align_capture(record['native_csv'],record['anchor'],args.csv,folder,max_age_s=record['max_age_s'])
        print(folder.resolve());return 0
    except (OSError,ValueError,KeyError) as exc:
        print('Brake diagnostic:',exc,file=sys.stderr);return 1


if __name__=='__main__': raise SystemExit(main())
