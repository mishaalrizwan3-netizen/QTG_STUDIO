"""Offline packaged-GUI smoke check; never connects to simulator or hardware."""
import importlib
import json
import os
from pathlib import Path
import tempfile
import traceback


def run(gui):
    os.environ['QTG_OFFLINE_GUI_CHECK'] = '1'
    result = {'ok': False, 'checks': []}
    app = None
    try:
        for name in ('backend.deceleration_config', 'backend.pdf_graph_settings',
                     'backend.initial_condition_settings', 'qtg_common_engine',
                     'qtg_climb_pid', 'SMK_Engine_From_Climb', 'reports.plot_graphs',
                     'PySide6.QtWidgets', 'pandas', 'numpy', 'matplotlib'):
            importlib.import_module(name)
        result['checks'].append('packaged imports')
        import qtg_common_engine as engine
        from backend.registry import suite_root
        assert (engine.brunner_root() / 'native/bin/p3d_bridge_engine_init_v1517.dll').is_file()
        assert (suite_root() / 'engine_deceleration/initialization.json').is_file()
        assert (suite_root() / 'engine_acceleration/procedure.py').is_file()
        result['checks'].append('worker DLL and test resources')
        with tempfile.TemporaryDirectory() as tmp:
            gui.RESULTS_ROOT = Path(tmp)
            app = gui.App()
            app.withdraw()
            index = next(i for i, s in enumerate(gui.TESTS) if s.reference == '1.f.(2)')
            app.select_test(index)
            spec = gui.TESTS[index]
            fields, graphs = app._test_settings[spec.reference]
            fields['Alt[ft]'].set('756')
            app._save_test_initial_conditions(spec, quiet=True)
            from backend.initial_condition_settings import load
            assert load(Path(tmp) / 'initial_condition_settings', spec.reference)['Alt[ft]'] == '756'
            graphs[0][2].set(not graphs[0][2].get())
            from backend.pdf_graph_settings import load as load_graphs
            assert load_graphs(Path(tmp) / 'pdf_graph_settings', spec.reference) is not None
            assert load_graphs(Path(tmp) / 'pdf_graph_settings', '1.f.(1)') is None
            app.update_idletasks()
            result['checks'].append('Tk test panel, editable settings and per-test persistence')
            source = os.environ.get('QTG_OFFLINE_RECORDED_CSV', '')
            if source:
                from reports.plot_graphs import plot_aligned_comparison
                output = Path(tmp) / 'recorded_report.pdf'
                plot_aligned_comparison(source, output_pdf_path=output, test_reference='1.f.(2)', selected_graphs=['ground_speed_kt'])
                assert output.is_file() and output.read_bytes().startswith(b'%PDF')
                result['checks'].append('PDF from existing recorded CSV')
        result['ok'] = True
    except BaseException:
        result['error'] = traceback.format_exc()
    finally:
        if app is not None:
            app.destroy()
        output = os.environ.get('QTG_DEPLOYMENT_CHECK_OUTPUT')
        if output:
            Path(output).write_text(json.dumps(result, indent=2), encoding='utf-8')
    return 0 if result['ok'] else 1
