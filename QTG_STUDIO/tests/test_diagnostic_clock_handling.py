import unittest
from unittest.mock import Mock

from backend.telemetry_verification import ReadOnlyTelemetry, TelemetryUnavailable


class DiagnosticClockTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.wall=1760000000.
        self.client=Mock()
        self.client.is_connected.return_value=True
        self.client.get_telemetry_sequence.return_value=1
        self.raw={'bridge_telemetry_sequence':1,'last_updated':self.wall,
                  'eng_rpms':[983.],'prop_rpms':[984.],'airspeed_indicated':0.}
        self.client.get_telemetry.return_value=self.raw
        self.reader=ReadOnlyTelemetry(self.client,clock=lambda:self.now,wall_clock=lambda:self.wall)

    def test_epoch_roundoff_can_reproduce_strict_failure_but_diagnostic_preserves_values(self):
        self.raw['last_updated']=self.wall+.000001
        with self.assertRaises(TelemetryUnavailable): self.reader.feedback()
        frame=self.reader.diagnostic_feedback()
        self.assertEqual(frame.values['engine_rpm'].value,983.)
        self.assertIsNone(frame.measured_at)
        self.assertIn('UNVERIFIED',self.reader.diagnostic_state)

    def test_mismatched_domain_and_missing_native_timing_remain_unverified(self):
        for timestamp in (100.,None):
            self.raw['last_updated']=timestamp
            frame=self.reader.diagnostic_feedback()
            self.assertEqual(frame.values['engine_rpm'].value,983.)
            self.assertIsNone(frame.measured_at)
        self.assertIsNone(self.reader.diagnostic_timing['native_acquired_at'])

    def test_future_and_wall_clock_jump_do_not_claim_freshness(self):
        self.raw['last_updated']=self.wall+10.
        self.reader.diagnostic_feedback()
        self.assertEqual(self.reader.diagnostic_state,'FUTURE_TIMESTAMP_UNVERIFIED')
        self.wall+=20
        self.reader.diagnostic_feedback()
        self.assertEqual(self.reader.diagnostic_state,'CLOCK_UNCERTAIN_UNVERIFIED')

    def test_paused_or_unchanged_then_frozen_even_if_copy_timestamp_changes(self):
        self.reader.diagnostic_feedback()
        self.now+=.5; self.wall+=.5; self.raw['last_updated']=self.wall
        self.reader.diagnostic_feedback()
        self.assertEqual(self.reader.diagnostic_state,'UNCHANGED_FRESHNESS_UNVERIFIED')
        self.now+=1.; self.wall+=1.; self.raw['last_updated']=self.wall
        frame=self.reader.diagnostic_feedback()
        self.assertEqual(self.reader.diagnostic_state,'STALE_CACHE')
        self.assertEqual(frame.values['engine_rpm'].value,983.)

    def test_changing_native_sequence_and_cache_time_are_available_not_verified(self):
        self.reader.diagnostic_feedback()
        self.now+=.5; self.wall+=.5
        self.raw.update(bridge_telemetry_sequence=2,last_updated=self.wall,eng_rpms=[1200.])
        self.client.get_telemetry_sequence.return_value=2
        self.assertEqual(self.reader.diagnostic_feedback().values['engine_rpm'].value,1200.)
        self.assertEqual(self.reader.diagnostic_state,'CONNECTED_FRESHNESS_UNVERIFIED')

    def test_strict_verification_still_rejects_duplicate_frames(self):
        self.reader.feedback()
        with self.assertRaises(TelemetryUnavailable): self.reader.feedback()

    def test_genuinely_old_cache_and_disconnection_remain_distinct(self):
        self.raw['last_updated']=self.wall-5.
        frame=self.reader.diagnostic_feedback()
        self.assertEqual(self.reader.diagnostic_state,'STALE_CACHE')
        self.assertEqual(frame.values['engine_rpm'].value,983.)
        with self.assertRaises(TelemetryUnavailable): self.reader.feedback()
        self.client.is_connected.return_value=False
        frame=self.reader.diagnostic_feedback()
        self.assertFalse(frame.connected)
        self.assertEqual(frame.values,{})
        self.assertEqual(self.reader.diagnostic_state,'DISCONNECTED')

    def test_operator_message_for_uncertain_clock_is_plain_english(self):
        from frontend.telemetry_diagnostic_window import presentation
        model=presentation({'active':True,'source_state':'FUTURE_TIMESTAMP_UNVERIFIED',
            'measurements':[{'channel':'engine_rpm','value':983.,'freshness':'UNVERIFIED'}]}, connected=True)
        self.assertIn('timing cannot be confirmed',model['message'])
        self.assertEqual(model['quality'],'Unverified')


if __name__=='__main__': unittest.main()
