"""Offline checks: never connect to or modify a running simulator."""
import ctypes
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import qtg_position_setter as setter


class Function:
    def __init__(self, call): self.call = call
    def __call__(self, *args): return self.call(*args)


class Simulator:
    def __init__(self, reject_weather=False, mismatch=False):
        self.calls = []
        self.pending = []
        self.state = [74., 270., 4., 32., 74., 2000., 0., 0., 15., 7., 80.]
        self.reject_weather = reject_weather
        self.mismatch = mismatch
        self.events = {}
        self.position = None
        self.position_type = None

    def __getattr__(self, name):
        def call(*args):
            self.calls.append((name, args))
            if name.endswith('_Open'):
                ctypes.cast(args[0], ctypes.POINTER(ctypes.c_void_p))[0] = ctypes.c_void_p(1)
            elif name.endswith('_RequestDataOnSimObject'):
                if args[4] != 1: return 0  # A paused simulator may not emit frame-period updates.
                state = list(self.state)
                if self.mismatch and args[1] == 2: state[8] = -80
                self.pending.append(struct.pack('<10I11d', 128, 0, 8, args[1], 0, 100, 0, 0, 0, 11, *state))
            elif name.endswith('_WeatherRequestObservationAtNearestStation'):
                text = b'GLOB 010000Z 27007KT 9999 BKN020 15/05 Q1013\0'
                self.pending.append(struct.pack('<4I', 16+len(text), 0, 10, 500)+text)
            elif name.endswith('_MapClientEventToSimEvent'):
                self.events[args[1]] = args[2]
            elif name.endswith('_TransmitClientEvent'):
                event, value = self.events[args[2]], args[3]
                if event == b'THROTTLE_SET': self.state[6] = max(5., value*100/16383)
                elif event == b'FLAPS_UP': self.state[7] = 0
                elif event == b'FLAPS_INCR': self.state[7] += 1
            elif name.endswith('_AddToDataDefinition') and args[1] == 101:
                self.position_type = args[4]
            elif name.endswith('_SetDataOnSimObject'):
                if args[1] == 102:
                    if args[5] != 8: return -1
                    self.state[6] = ctypes.cast(args[-1], ctypes.POINTER(ctypes.c_double)).contents.value
                    return 0
                if self.position_type != 12 or args[5] != 56:
                    self.pending.append(struct.pack('<6I', 24, 0, 1, 19, 21, 5))
                    return 0
                pos = ctypes.cast(args[-1], ctypes.POINTER(setter.SIMCONNECT_DATA_INITPOSITION)).contents
                self.position = (pos.Latitude, pos.Longitude, pos.Altitude, pos.Heading, pos.Airspeed)
                self.state[3:6] = [pos.Latitude, pos.Longitude, pos.Altitude]
                self.state[10] = pos.Airspeed
            elif name.endswith('_WeatherSetObservation'):
                if self.reject_weather:
                    self.pending.append(struct.pack('<6I', 24, 0, 1, 13, 1, 0))
                else:
                    self.state[8], self.state[9] = 10., 12.
                self.metar = args[2].decode()
            elif name.endswith('_CallDispatch'):
                for packet in self.pending:
                    buffer = ctypes.create_string_buffer(packet)
                    args[1](ctypes.addressof(buffer), len(packet), None)
                self.pending.clear()
            return 0
        fn = Function(call)
        setattr(self, name, fn)
        return fn


class ConditionsTest(unittest.TestCase):
    def apply(self, simulator, **changes):
        options = dict(alt_ft=3000., ias_kt=90., on_ground=False, lat=33.2, lon=73.4,
                       flaps=2, throttle=35., wind_kt=12., temperature_c=10.)
        options.update(changes)
        clock = iter(i/10 for i in range(1000))
        with patch.object(setter, 'find_simconnect_dll', return_value=Path('fake.dll')), \
             patch.object(setter.ctypes, 'CDLL', return_value=simulator), \
             patch.object(setter.time, 'sleep'), \
             patch.object(setter.time, 'monotonic', side_effect=lambda: next(clock)):
            setter.set_initial_conditions(**options)

    def test_all_conditions_and_pause_cleanup(self):
        sim = Simulator()
        self.apply(sim)
        self.assertEqual(sim.position, (33.2, 73.4, 3000., 74., 90))
        self.assertEqual(sim.state[7], 2)
        self.assertAlmostEqual(sim.state[6], 35, places=2)
        self.assertIn('27012KT', sim.metar)
        self.assertIn('10/05&A00914', sim.metar)
        self.assertIn('BKN020', sim.metar)
        self.assertIn('Q1013', sim.metar)
        pauses = [args[3] for name,args in sim.calls if name.endswith('_TransmitClientEvent') and args[2]==200]
        self.assertEqual(pauses, [1,0])
        self.assertEqual(sim.calls[-1][0], 'SimConnect_Close')

    def test_zero_throttle_overrides_event_idle_floor(self):
        sim = Simulator()
        self.apply(sim, throttle=0.)
        self.assertEqual(sim.state[6],0.)

    def test_position_layout_matches_sdk(self):
        header = (Path(__file__).resolve().parents[1]/'p3d_sdk/SimConnect.h').read_text()
        entries = header.split('enum SIMCONNECT_DATATYPE {',1)[1].split('};',1)[0]
        import re
        names = re.findall(r'^\s*(SIMCONNECT_DATATYPE_\w+)\s*[,=]',entries,re.MULTILINE)
        self.assertEqual(names.index('SIMCONNECT_DATATYPE_INITPOSITION'), setter.SIMCONNECT_DATATYPE_INITPOSITION)
        self.assertEqual(ctypes.sizeof(setter.SIMCONNECT_DATA_INITPOSITION),56)

    def test_ground_and_existing_pause(self):
        sim = Simulator()
        self.apply(sim, on_ground=True, was_paused=True)
        self.assertEqual(sim.position[-1], 0)
        self.assertFalse(any(name.endswith('_TransmitClientEvent') and args[2]==200 for name,args in sim.calls))

    def test_rejection_unpauses_and_closes(self):
        sim = Simulator(reject_weather=True)
        with self.assertRaisesRegex(RuntimeError, 'rejected'): self.apply(sim)
        self.assertEqual(sim.calls[-2][1][3], 0)
        self.assertEqual(sim.calls[-1][0], 'SimConnect_Close')

    def test_readback_mismatch_is_failure(self):
        sim = Simulator(mismatch=True)
        with self.assertRaisesRegex(RuntimeError, 'Timed out'): self.apply(sim)
        self.assertEqual(sim.calls[-1][0], 'SimConnect_Close')

    def test_invalid_flaps_rejected_before_any_write(self):
        sim = Simulator()
        with self.assertRaisesRegex(ValueError, 'flap handle'): self.apply(sim, flaps=4)
        self.assertIsNone(sim.position)
        self.assertEqual(sim.calls[-1][0], 'SimConnect_Close')

    def test_invalid_coordinates_no_connection(self):
        with patch.object(setter.ctypes, 'CDLL') as loader:
            with self.assertRaises(ValueError): self.apply(Simulator(), lat=91)
            loader.assert_not_called()

    def test_weather_preserves_clouds_pressure_dewpoint_and_negative_temperatures(self):
        text = setter._condition_metar('OPRW&A200 010000Z 27007KT 9999 BKN020 15/05 Q1013', 0, -5, 2000, 270).decode()
        self.assertTrue(text.startswith('GLOB '))
        self.assertIn('-05/-05&A00610', text)
        self.assertIn('BKN020', text)
        self.assertIn('Q1013', text)

    def test_missing_dll_is_failure(self):
        with patch.object(setter, 'find_simconnect_dll', return_value=None):
            with self.assertRaisesRegex(RuntimeError, 'not found'):
                setter.set_initial_conditions(alt_ft=2000, ias_kt=80, on_ground=False,
                    lat=32,lon=74,flaps=0,throttle=0,wind_kt=0,temperature_c=15)


class PanelTest(unittest.TestCase):
    def test_password_visibility_fields_and_set(self):
        import tkinter as tk
        import main
        root = tk.Tk()
        root.withdraw()
        try:
            host = SimpleNamespace(settings_panel=tk.Frame(root), _work_canvas=tk.Canvas(root),
                                   _test_settings={}, recording_active=False, p3d_paused=False)
            host._work_canvas.pack()
            host._label = lambda parent,text,size=11,color=None,weight='normal',**kw: tk.Label(parent,text=text,**kw)
            host._build_settings_panel = lambda spec: main.App._build_settings_panel(host,spec)
            host._open_test_password_dialog = lambda i: main.App._open_test_password_dialog(host,i)
            host._lock_test_settings = lambda spec: main.App._lock_test_settings(host,spec)
            host._apply_test_settings = lambda spec: main.App._apply_test_settings(host,spec)
            host._fly_to_initial_conditions = lambda spec,**kw: main.App._fly_to_initial_conditions(host,spec,**kw)
            host.update_idletasks = root.update_idletasks
            spec = main.TESTS[2]
            host._build_settings_panel(spec)
            self.assertEqual(host.settings_panel.winfo_manager(), '')
            for password in (None, 'wrong'):
                with patch.object(main.simpledialog,'askstring',return_value=password), patch.object(main.messagebox,'showerror'):
                    host._open_test_password_dialog(2)
                self.assertEqual(host.settings_panel.winfo_manager(), '')
            with patch.object(main.simpledialog,'askstring',return_value=main.TEST_ADMIN_PASSWORD):
                host._open_test_password_dialog(2)
            self.assertEqual(host.settings_panel.winfo_manager(), 'pack')
            fields,graphs = host._test_settings[spec.reference]
            self.assertIn('Heading[deg]', fields)
            self.assertIn('Pitch[deg]', fields)
            self.assertIn('Bank[deg]', fields)
            self.assertNotIn('Wind direction[deg]', fields)
            self.assertIn('Latitude[deg]', fields)
            self.assertIn('Longitude[deg]', fields)
            self.assertEqual(len(graphs),25)
            self.assertNotIn('Wind speed[kt]', fields)
            host._launch_common_test = Mock()
            self.assertTrue(host._fly_to_initial_conditions(spec))
            host._launch_common_test.assert_called_once_with('manual', spec, force_initialize=True)
            fields['Alt[ft]'].set('')
            self.assertTrue(host._fly_to_initial_conditions(spec))
            self.assertEqual(fields['Alt[ft]'].get(), '2000')
            fields['Latitude[deg]'].set('91')
            host._launch_common_test.reset_mock()
            with patch.object(main.messagebox,'showerror'):
                self.assertFalse(host._fly_to_initial_conditions(spec))
            host._launch_common_test.assert_not_called()
            host._lock_test_settings(spec)
            self.assertEqual(host.settings_panel.winfo_manager(), '')
        finally:
            root.destroy()


if __name__ == '__main__': unittest.main()
