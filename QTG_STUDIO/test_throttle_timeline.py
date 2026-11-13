import tempfile
import unittest
from pathlib import Path
from throttle_timeline import ThrottleTimeline, load_manual_throttle

class ThrottleReplayTests(unittest.TestCase):
    def test_rapid_movement_is_not_stretched_to_one_second(self):
        timeline=ThrottleTimeline([(i*.04,5 if i<153 else 100) for i in range(200)])
        self.assertEqual(timeline.at(6.1),5)
        self.assertEqual(timeline.at(6.12),100)
        self.assertEqual(timeline.at(6.2),100)
    def test_deceleration_preserves_measured_steps(self):
        timeline=ThrottleTimeline([(i*.04,100 if i<125 else 5) for i in range(200)])
        self.assertEqual(timeline.at(4.99),100)
        self.assertEqual(timeline.at(5.0),5)
    def test_one_second_csv_is_rejected(self):
        with self.assertRaises(ValueError):ThrottleTimeline([(0,5),(1,100),(2,100)])
    def test_existing_channel_export_loads(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'manual.csv'
            path.with_name('manual_engine_channels.csv').write_text(
                'time_s,throttle_pct\n0.0,5\n0.04,5\n0.08,100\n0.12,100\n')
            timeline=load_manual_throttle(path)
            self.assertEqual(timeline.at(.079),5)
            self.assertEqual(timeline.at(.08),100)
    def test_gap_is_not_silently_interpolated(self):
        with self.assertRaises(ValueError):ThrottleTimeline([(0,5),(.04,5),(1,100),(1.04,100)])

if __name__=='__main__':unittest.main()
