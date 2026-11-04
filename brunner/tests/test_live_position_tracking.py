import sys, math, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/"src"))
sys.path.insert(0,str(ROOT/'src'))
from replay_support import BrunnerPositionTracker

class TrackingTests(unittest.TestCase):
    def test_changing_bias_is_corrected_instead_of_keeping_paused_offset(self):
        feedback=BrunnerPositionTracker((-.03,0.,0.))
        measured=-.2
        fixed=-.2
        target=-.2
        errors=[]
        fixed_errors=[]
        for index in range(1000):
            now=index*.02
            # Model a motor with 0.2 s response and a bias that reverses in flight.
            bias=.03 if now<10 else -.02
            command=feedback.command(now,(target,0.,0.),(target,0.,0.),(measured,0.,0.))[0]
            measured+=.1*(command+bias-measured)
            fixed+=.1*(target-.03+bias-fixed)
            if now>17:
                errors.append(abs(measured-target))
                fixed_errors.append(abs(fixed-target))
        self.assertLess(sum(errors)/len(errors),.005)
        self.assertGreater(sum(fixed_errors)/len(fixed_errors),.04)

    def test_correction_is_bounded_and_rate_limited(self):
        feedback=BrunnerPositionTracker()
        previous=0.
        for index in range(150):
            command=feedback.command(index*.02,(.2,0.,0.),(.2,0.,0.),(-.5,0.,0.))[0]
            correction=command-.2
            self.assertLessEqual(abs(correction),.0500001)
            self.assertLessEqual(abs(correction-previous),.0020001)
            previous=correction

    def test_invalid_feedback_stops_instead_of_sending_a_command(self):
        with self.assertRaisesRegex(ValueError,'feedback'):
            BrunnerPositionTracker().command(0.,(0.,0.,0.),(0.,0.,0.),(float('nan'),0.,0.))

    def test_travel_limits_are_preserved(self):
        feedback=BrunnerPositionTracker((.05,0.,0.))
        for index in range(100):
            command=feedback.command(index*.02,(1.,0.,0.),(1.,0.,0.),(0.,0.,0.))
            self.assertLessEqual(command[0],1.)

    def test_completed_display_uses_actual_duration_and_ignores_late_progress(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import Super_Mushak_Logger_GUI as gui
        fake=SimpleNamespace(is_recording=True,p3d=Mock(),brunner=Mock(),
            rec_time=[.05,74.92],target_duration_sec=75,
            _play_beep=Mock(),_update_stage_ui=Mock(),_save_csv_and_generate_reports=Mock(),
            lbl_rec_status=Mock(),lbl_rec_progress=Mock(),rec_progress=Mock())
        gui.SuperMushakLoggerApp._on_recording_finished(fake)
        self.assertFalse(fake.is_recording)
        fake.lbl_rec_status.setText.assert_called_with('COMPLETED - simulation paused')
        fake.lbl_rec_progress.setText.assert_called_with('74.9 seconds (2 recorded rows)')
        gui.SuperMushakLoggerApp._update_record_progress(fake,74)
        self.assertEqual(fake.lbl_rec_progress.setText.call_count,1)

    def test_zero_error_does_not_create_controller_bias(self):
        feedback=BrunnerPositionTracker()
        self.assertEqual(feedback.command(0.,(.2,-.1,0.),(.2,-.1,0.),(.2,-.1,0.)),(.2,-.1,0.))

if __name__=='__main__': unittest.main(verbosity=2)
