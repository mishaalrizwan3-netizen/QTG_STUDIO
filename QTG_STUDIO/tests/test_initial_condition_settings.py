"""Offline tests for per-test GUI conditions and persistence."""
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend.catalog import TESTS
from backend.initial_condition_settings import FIELDS, catalog_defaults, load, save, validate


class InitialConditionSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_catalog_entries_without_requirements_remain_unset(self):
        missing = [s for s in TESTS if s.reference not in ('1.c.(1)', '1.f.(1)', '1.f.(2)')]
        self.assertEqual(len(missing), 27)
        for spec in missing:
            with self.subTest(reference=spec.reference):
                self.assertTrue(all(value == '' for value in catalog_defaults(spec).values()))

    def test_catalog_preserves_established_test_specific_values(self):
        specs = {s.reference:s for s in TESTS}
        self.assertEqual(catalog_defaults(specs['1.f.(1)'])['Alt[ft]'], '2000')
        self.assertEqual(catalog_defaults(specs['1.f.(2)'])['IAS[kts]'], '0')
        self.assertEqual(catalog_defaults(specs['1.c.(1)'])['Alt[ft]'], '3000')

    def test_independent_saved_settings_persist_and_zero_is_valid(self):
        one=dict.fromkeys(FIELDS, '')
        one.update({'Flt cond':'Ground', 'IAS[kts]':'0', 'Flaps':'0'})
        two=dict.fromkeys(FIELDS, '')
        two.update({'Flt cond':'Airborne', 'IAS[kts]':'73'})
        save(self.root,'2.a.(1)',one)
        save(self.root,'2.a.(2)',two)
        self.assertEqual(load(self.root,'2.a.(1)'),one)
        self.assertEqual(load(self.root,'2.a.(2)'),two)
        self.assertEqual(load(self.root,'2.a.(3)'),{})

    def test_missing_fields_are_explicit_and_partial_save_is_allowed(self):
        values=dict.fromkeys(FIELDS, '')
        values['IAS[kts]']='0'
        self.assertIn('Alt[ft]',validate(values))
        with self.assertRaisesRegex(ValueError,'Initial conditions not set:.*Alt'):
            validate(values,require_complete=True)
        save(self.root,'2.b.(1)',values)
        self.assertEqual(load(self.root,'2.b.(1)')['IAS[kts]'],'0')

    def test_invalid_and_mismatched_data_are_not_accepted(self):
        values=dict.fromkeys(FIELDS, '')
        for field,value in (('IAS[kts]','nan'),('Flaps','1.5'),('Latitude[deg]','91')):
            with self.subTest(field=field),self.assertRaises(ValueError):
                save(self.root,'2.a.(1)',dict(values,**{field:value}))
        with self.assertRaises(ValueError):save(self.root,'../unsafe',values)
        path=self.root/'2.a.(1).json'
        path.write_text(json.dumps({'reference':'2.a.(2)','initial_conditions':values}))
        with self.assertRaisesRegex(ValueError,'match this test'):load(self.root,'2.a.(1)')

    def test_unsupported_multiengine_entry_never_launches(self):
        from frontend.main import App
        gui=SimpleNamespace(common_status=Mock())
        App._launch_common_test(gui,'manual',SimpleNamespace(reference='1.c.(2)'))
        self.assertIn('settings review only',gui.common_status.configure.call_args.kwargs['text'])

    def test_missing_manual_conditions_are_rejected_before_launch(self):
        from frontend import main
        values=dict.fromkeys(FIELDS, '')
        fields={k:Mock(get=Mock(return_value=v)) for k,v in values.items()}
        gui=SimpleNamespace(_test_settings={'2.a.(1)':(fields,[])},
                            _test_defaults={'2.a.(1)':values})
        with patch.object(main.messagebox,'showerror') as show:
            result=main.App._apply_test_settings(gui,SimpleNamespace(reference='2.a.(1)'))
        self.assertFalse(result)
        self.assertIn('not set',show.call_args.args[1])

    def test_automatic_with_missing_gui_fields_passes_only_selected_reference(self):
        from frontend.main import App
        import qtg_common_engine as engine
        reference='2.a.(1)'
        fields={key:Mock(get=Mock(return_value='')) for key in FIELDS}
        gui=SimpleNamespace(_common_duration_var=Mock(get=Mock(return_value='21')),
            _test_settings={reference:(fields,[])},_apply_test_settings=Mock(return_value=True),
            _common_reference_var=Mock(get=Mock(return_value='manual.csv')),
            _common_process=None,_manual_gui_bindings={},after=Mock(),_poll_common_test=Mock(),
            common_status=Mock(),common_record_button=Mock())
        spec=SimpleNamespace(reference=reference,title='Test',pdf_graphs=[])
        flags={'QTG_UNIVERSAL_MANUAL_PLANS':'{}','QTG_UNIVERSAL_AUTOMATIC_PLANS':'{}',
               'QTG_UNIVERSAL_ENGINE_ACCELERATION':'0','QTG_UNIVERSAL_ENGINE_DECELERATION':'0',
               'QTG_UNIVERSAL_INITIALIZATION_ONLY':'0'}
        with patch.dict(os.environ,flags), patch.object(engine,'validate_reference',return_value='selected.csv'), \
                patch.object(engine,'launch',return_value=(Mock(),{})) as launch:
            App._launch_common_test(gui,'auto',spec)
        self.assertEqual(launch.call_args.args[3],'selected.csv')
        self.assertIsNone(launch.call_args.kwargs['initial_conditions'])
        gui._apply_test_settings.assert_called_once_with(spec,allow_incomplete=True)

    def test_each_gui_test_gets_distinct_fields_and_saved_values_reload(self):
        import tkinter as tk
        from frontend import main
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        host=SimpleNamespace(_test_settings={},_manual_conditions_changed=Mock())
        with patch.object(main,'RESULTS_ROOT',self.root):
            for spec in TESTS:
                main.App._ensure_test_settings(host,spec)
            self.assertEqual(len(host._test_settings),30)
            first=host._test_settings['2.a.(1)'][0]
            second=host._test_settings['2.a.(2)'][0]
            first['IAS[kts]'].set('67')
            self.assertEqual(second['IAS[kts]'].get(),'')
            save(self.root/'initial_condition_settings','2.a.(1)',
                 {key:var.get() for key,var in first.items()})
            reloaded=SimpleNamespace(_test_settings={},_manual_conditions_changed=Mock())
            spec=next(s for s in TESTS if s.reference=='2.a.(1)')
            main.App._ensure_test_settings(reloaded,spec)
            self.assertEqual(reloaded._test_settings['2.a.(1)'][0]['IAS[kts]'].get(),'67')


if __name__ == '__main__':
    unittest.main()
