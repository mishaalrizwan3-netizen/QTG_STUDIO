import unittest
from engine_control_diagnostics import classify_handover, stable_rpm_sample

class EngineControlsTest(unittest.TestCase):
    def test_mapping_only(self):
        mismatch, others, controls = classify_handover('ElevInput=+0.00116/+0.03455 d=-0.03339 (FAIL) | AilInput=-0.00171/-0.00192 (OK) | ElevTrim=+0.00000/+0.00000 (OK) | Throttle=5.00/5.00 (OK) | RPM=1526.0/1581.0 (OK)')
        self.assertTrue(mismatch)
        self.assertFalse(others)
        self.assertEqual(len(controls), 1)
    def test_rpm_failure_still_blocks(self):
        mismatch, others, _ = classify_handover('ElevInput=+0.00116/+0.03455 (FAIL) | RPM=1500/1800 (FAIL)')
        self.assertTrue(mismatch)
        self.assertEqual(len(others), 1)
    def test_missing_controls_are_not_accepted(self):
        mismatch, others, _ = classify_handover('ElevInput=MISSING(None/0.2) | Throttle=5/5 (OK)')
        self.assertFalse(mismatch)
        self.assertTrue(others)
    def test_missing_unrelated_blocks(self):
        mismatch, others, _ = classify_handover('ElevInput=+0.001/+0.034 (FAIL) | ElevTrim=MISSING(None/0.03)')
        self.assertTrue(mismatch)
        self.assertTrue(others)
    def sample(self, eng=2700, prop=2699, thr=100):
        return dict(bridge_telemetry_sequence=3, eng_rpms=[eng],prop_rpms=[prop],throttles=[thr],sim_on_ground=1,brake_parking_position=1)
    def test_2700(self):
        self.assertTrue(stable_rpm_sample(self.sample())[0])
    def test_transient(self):
        self.assertFalse(stable_rpm_sample(self.sample(2501,2501))[0])
    def test_brake(self):
        s=self.sample();s['brake_parking_position']=0
        self.assertFalse(stable_rpm_sample(s)[0])
    def test_missing_telemetry(self):
        self.assertFalse(stable_rpm_sample({})[0])
    def test_engine_split(self):
        self.assertFalse(stable_rpm_sample(self.sample(2670,2740))[0])

if __name__=='__main__':unittest.main()
