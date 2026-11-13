"""Safe executable smoke test: no hardware or simulator connection is opened."""
import os,json,re,traceback,sys
from pathlib import Path

def run(argv):
    destination=Path(argv[argv.index('--self-test')+1]).resolve()
    destination.parent.mkdir(parents=True,exist_ok=True)
    report={"ok":False}
    try:
        os.environ['QT_QPA_PLATFORM']='offscreen'
        from PySide6.QtWidgets import QApplication,QWidget,QLabel
        app=QApplication.instance() or QApplication(['packaging-check'])
        window_class=sys.modules['__main__'].SuperMushakLoggerApp
        original_connect=window_class._start_hardware_connections
        startup_calls=[]
        window_class._start_hardware_connections=lambda self:startup_calls.append(self)
        try:
            widget=window_class()
            assert startup_calls == [widget], "GUI must request hardware connections exactly once at startup"
            report["startup_connection_requested"]=True
        finally:
            window_class._start_hardware_connections=original_connect
        widget.ui_timer.stop()
        widget.show();app.processEvents()
        assert widget.grab().save(str(destination.with_suffix('.png')))
        report['gui_created']=True
        report['hardware_connected']=False
        from p3d_simconnect_v15 import SimConnectClient
        client=SimConnectClient()
        report['bridge_version']=client.bridge_version
        report['bridge_path']=client._dll_path
        report['frozen']=bool(getattr(sys,'frozen',False))
        if report['frozen']:
            assert Path(client._dll_path).is_relative_to(Path(sys._MEIPASS)), 'Bridge loaded outside the executable bundle'
        import plot_graphs,pandas as pd
        reference=Path(argv[argv.index('--reference-csv')+1]) if '--reference-csv' in argv else None
        if reference:
            frame=pd.read_csv(reference)
        else:
            frame=pd.DataFrame({'Time (s)':[0.,1.,2.],'Indicated Airspeed (kts)':[80.,81.,82.],'Indicated Altitude (ft)':[3000.,3010.,3020.],'Rate of Climb/Descent (fpm)':[600.,600.,600.]})
        sample=destination.with_name('packaging_manual_sample.csv');frame.to_csv(sample,index=False)
        pdf=Path(plot_graphs.plot_aligned_comparison(sample))
        pages=len(re.findall(rb'/Type\s*/Page\b',pdf.read_bytes()))
        assert pages==6, f'Expected six report pages, got {pages}'
        report.update(ok=True,qt_platform=app.platformName(),pdf=str(pdf),pdf_pages=pages,simulator_connected=False)
        widget.close()
    except Exception:
        report['error']=traceback.format_exc()
    destination.write_text(json.dumps(report,indent=2),encoding='utf-8')
    return 0 if report['ok'] else 1
