import json
from pathlib import Path
import tempfile
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from frontend.telemetry_diagnostic_window import presentation, confirmed_outputs, TelemetryDiagnosticWindow


class DiagnosticPresentationTests(unittest.TestCase):
    def data(self, active=True):
        return {'active': active, 'issues': ['Native timing unavailable'], 'measurements': [
            {'channel': 'engine_rpm', 'value': 2700., 'unit': 'rpm', 'source': 'eng_rpms[0]'},
            {'channel': 'ias_kt', 'value': 0., 'unit': 'kt'},
            {'channel': 'brunner.pitch', 'value': .2, 'unit': 'normalized'}]}

    def test_zero_is_valid_and_missing_is_not_zero(self):
        model=presentation(self.data(), connected=True)
        self.assertEqual(model['readings']['ias_kt'][0], '0')
        self.assertEqual(model['readings']['mixture'][0], 'Not available')
        self.assertIn('Rounded', model['readings']['brunner.pitch'][2])
        self.assertEqual(model['quality'], 'Unverified')

    def test_disconnection_and_frozen_status_do_not_show_live_readings(self):
        for connected,stale in ((False,False),(True,True)):
            model=presentation(self.data(),connected=connected,stale=stale)
            self.assertEqual(model['quality'],'Unavailable')
            self.assertIn('not live',model['readings']['engine_rpm'][2])

    def test_stopped_samples_and_warnings_remain_visible(self):
        model=presentation(self.data(False),connected=True)
        self.assertFalse(model['running'])
        self.assertIn('Saved sample',model['readings']['engine_rpm'][2])
        self.assertEqual(model['issues'],['Native timing unavailable'])

    def test_nonfinite_values_display_unavailable(self):
        data=self.data()
        data['measurements'][0]['value']=float('nan')
        self.assertEqual(presentation(data)['readings']['engine_rpm'][0],'Not available')

    def test_save_success_requires_both_files_and_matching_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            data={'active':False,'outputs':{'csv':str(root/'data.csv'),'summary':str(root/'summary.json')}}
            self.assertIsNone(confirmed_outputs(data))
            (root/'data.csv').write_text('channel,value\nengine_rpm,2700\n')
            (root/'summary.json').write_text('{}')
            self.assertIsNone(confirmed_outputs(data))
            (root/'summary.json').write_text(json.dumps(data))
            self.assertEqual(confirmed_outputs(data),(root/'data.csv',root/'summary.json'))
            self.assertIsNone(confirmed_outputs(dict(data,active=True)))


THEME={'blue':'#1D4ED8','dim':'#64748B','text':'#0F172A',
       'green':'#047857','amber':'#B45309','red':'#991B1B'}


class DiagnosticWindowTests(unittest.TestCase):
    def setUp(self):
        try: self.root=tk.Tk()
        except tk.TclError as exc: self.skipTest(str(exc))
        self.root.withdraw()
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.addCleanup(self.root.destroy)
        self.directory=Path(self.temp.name)
        self.session=SimpleNamespace(kind='not connected', directory=self.directory,
            status_file=self.directory/'status.json', command=Mock(), connect=Mock())

    def window(self):
        view=TelemetryDiagnosticWindow(self.root,self.session,theme=THEME)
        view.window.withdraw()
        self.addCleanup(view.close)
        return view

    def test_initial_steps_and_unavailable_values(self):
        view=self.window()
        self.assertEqual(str(view.buttons[0]['state']),'normal')
        self.assertEqual(str(view.buttons[1]['state']),'disabled')
        self.assertEqual(str(view.buttons[2]['state']),'disabled')
        self.assertEqual(view.values['engine_rpm'].get(),'Not available')
        self.session.command.assert_not_called()

    def test_connected_running_stopping_and_confirmed_save(self):
        self.session.kind='borrowed dashboard (limited channels)'
        self.session.dashboard=SimpleNamespace(running=True,snapshot=lambda:SimpleNamespace(connected=True))
        view=self.window()
        self.assertEqual(str(view.buttons[1]['state']),'normal')
        view.action('start')
        self.session.command.assert_called_once_with('start')
        data=DiagnosticPresentationTests().data()
        self.session.status_file.write_text(json.dumps(data))
        view.refresh(False)
        self.assertEqual(view.badges['Data Monitoring']['text'],'Running')
        self.assertEqual(str(view.buttons[1]['state']),'disabled')
        self.assertEqual(str(view.buttons[2]['state']),'normal')
        view.action('stop')
        data.update(active=False,outputs={'csv':str(self.directory/'data.csv'),
                                         'summary':str(self.directory/'summary.json')})
        self.session.status_file.write_text(json.dumps(data))
        view.refresh(False)
        self.assertNotIn('successfully',view.message.get())
        (self.directory/'data.csv').write_text('channel,value\n')
        (self.directory/'summary.json').write_text(json.dumps(data))
        view.refresh(False)
        self.assertIn('successfully',view.message.get())
        self.assertEqual(str(view.folder['state']),'normal')
        view.copy_path('csv')
        self.assertEqual(view.window.clipboard_get(),str(self.directory/'data.csv'))

    def test_reopen_close_and_resize_without_connections(self):
        for _ in range(2):
            view=self.window()
            view.window.geometry('1120x660')
            view.window.deiconify()
            self.root.update_idletasks()
            self.assertLessEqual(view.canvas.winfo_width(),1120)
            view.close()
            self.assertFalse(view.window.winfo_exists())
        self.assertEqual(self.session.command.call_count,2)
        self.session.connect.assert_not_called()


if __name__=='__main__': unittest.main()
