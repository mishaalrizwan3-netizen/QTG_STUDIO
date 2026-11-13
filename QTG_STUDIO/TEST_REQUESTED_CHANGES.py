"""Offline source / PDF regression tests; no P3D or BRUNNER access."""
import ast, os, sys, tempfile, shutil
from pathlib import Path
BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(BASE))
import plot_graphs
from SMK_Engine_TiTt_V2_1_3_Addon import measure

def check_source():
  src=(BASE/'SMK_Engine_From_Climb.py').read_text(encoding='utf-8')
  tree=ast.parse(src)
  methods={x.name:x for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='EngineApp' for x in c.body if isinstance(x,(ast.FunctionDef,ast.AsyncFunctionDef))}
  for meth in ('arm_recording','_on_telemetry_tick'):
   strings=ast.get_source_segment(src,methods[meth])
   assert "and abs(d.get('ground_speed'" not in strings
   assert "d.get('ground_speed',0)>.5" not in strings
   assert "if abs(d.get('ground_speed'" not in strings
   print('PASS: no deceleration speed condition in',meth)
  assert "d.get('sim_on_ground', 0) > .5" in src
  assert "d.get('brake_parking_position', 0) > .9" in src
  assert "d.get('sim_on_ground',0)<.5" in src
  assert 'max_observed_ground_speed_kt' in src
  print('PASS: ground contact, brake and audit retained')
  rp=(BASE/'plot_graphs.py').read_text(encoding='utf-8')
  assert 'P3D-ONLY DIAGNOSTIC: CLS tracking' not in rp
  assert 'P3D DIAGNOSTIC ONLY: BRUNNER tracking' not in rp
  assert 'Recording status: {status}' not in rp
  assert 'CONTROL TRACKING FAILED' not in rp
  assert 'f"Engine test | {status}' not in rp
  tit=(BASE/'SMK_Engine_TiTt_V2_1_3_Addon.py').read_text(encoding='utf-8')
  assert 'NOT AN EASA QUALIFICATION PASS: independent approved aircraft' not in tit
  print('PASS: repeated PDF banners removed from generation code')

def pdf_check():
  old=Path('/mnt/data/Super_Mushak_auto_takeoff_data_2026-10-22_12-31-26_legacy_plot.csv')
  ch=Path('/mnt/data/Super_Mushak_auto_takeoff_data_2026-10-22_12-31-26_engine_channels.csv')
  if not old.exists() or not ch.exists():print('No sample data for PDF check');return
  with tempfile.TemporaryDirectory() as d:
   d=Path(d)
   stem='Super_Mushak_auto_takeoff_data_2026-10-22_12-31-26'
   stem2='Super_Mushak_manual_takeoff_data_2026-10-22_12-30-00'
   auto=d/(stem+'_legacy_plot.csv');manual=d/(stem2+'_legacy_plot.csv')
   shutil.copy2(old,auto);shutil.copy2(old,manual)
   shutil.copy2(ch,d/(stem+'_engine_channels.csv'));shutil.copy2(ch,d/(stem2+'_engine_channels.csv'))
   try:
    m1, tr1=measure(manual,'Manual','acceleration',parameter_kind='map')
    m2, tr2=measure(auto,'Automatic','acceleration',parameter_kind='map')
    titt={'raw_measurements':{'Manual':m1,'Automatic':m2},'raw_traces':{'Manual':tr1,'Automatic':tr2},'mode':'acceleration','critical_parameter':'manifold_inhg'}
   except Exception as ex:
    print('Unable to make Ti/Tt sample:',type(ex).__name__,str(ex));titt=None
   outfile=Path('/mnt/data/V213_Clean_Report_Sample.pdf')
   plot_graphs._render_report(auto,ref_path=manual,out_path=outfile,titt_results=titt,
       metadata={'kind':'acceleration','status':'invalid_cls_tracking','p3d_diagnostic_only':True,'event_time_s':6.0})
   import fitz
   doc=fitz.open(outfile); content='\n'.join(p.get_text() for p in doc)
   for unwanted in ('NOT AN EASA QUALIFICATION PASS','P3D DIAGNOSTIC ONLY:','P3D-ONLY DIAGNOSTIC:','BRUNNER NOT QUALIFIED','Recording status:','CONTROL TRACKING FAILED'):
    assert unwanted not in content,unwanted
   assert 'Time zero: measured throttle movement' in content
   assert 'Ti (10% response)' in content
   assert len(doc)==5,len(doc)
   print('PASS: PDF smoke ->',outfile,'pages',len(doc),'warnings absent')

if __name__=='__main__':
 check_source();pdf_check();print('ALL TESTS PASSED')
