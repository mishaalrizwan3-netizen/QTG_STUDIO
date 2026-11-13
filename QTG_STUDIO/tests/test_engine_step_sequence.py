import unittest
from backend.engine_step_sequence import EngineStepSequence, Settings


def settings(deceleration=False):
    return Settings(target_rpm=972 if deceleration else 2700,
                    initial_throttle=100 if deceleration else 5,
                    final_throttle=5 if deceleration else 100,
                    starting_throttle_tolerance=1, throttle_event_delta=2,
                    engine_response_delta=100, max_receipt_age=2)


def sample(seq, t, rpm, throttle, **kwargs):
    return seq.sample(now=t, receipt_time=t, sequence=int(t * 10), rpm=rpm, throttle=throttle, **kwargs)


def baseline(seq, rpm, throttle):
    for t in (1, 2, 3, 4): sample(seq, t, rpm, throttle)
    return sample(seq, 5, rpm, throttle)


class EngineStepTests(unittest.TestCase):
    def test_manual_and_automatic_use_same_independent_sequence(self):
        for mode in ('manual', 'auto'):
            for decel in (False, True):
                with self.subTest(mode=mode, decel=decel):
                    cfg = settings(decel); seq = EngineStepSequence(cfg); seq.start(0)
                    initial_rpm = 2700 if decel else 1000
                    for t in (1, 2, 3, 4):
                        self.assertIsNone(sample(seq, t, initial_rpm, cfg.initial_throttle))
                    self.assertEqual(sample(seq, 5, initial_rpm, cfg.initial_throttle),
                                     {'type': 'set_throttle', 'percent': cfg.final_throttle})
                    for t in (6, 7, 8, 9): sample(seq, t, cfg.target_rpm, cfg.final_throttle)
                    self.assertEqual(seq.phase, 'COMPLETE')
                    self.assertEqual(seq.events['throttle_onset'], 6)
                    self.assertEqual(seq.events['recording_start'], 0)

    def test_fluctuation_resets_stability(self):
        seq=EngineStepSequence(settings()); seq.start(0)
        baseline(seq,1000,5); sample(seq,6,2700,100); sample(seq,8,2800,100)
        sample(seq,9,2700,100); sample(seq,11,2700,100)
        self.assertEqual(seq.phase, 'STABILIZING')
        sample(seq,12,2700,100); self.assertEqual(seq.phase, 'COMPLETE')

    def test_no_completion_without_measured_event_or_engine_response(self):
        seq=EngineStepSequence(settings()); seq.start(0)
        baseline(seq,2700,5)
        for t in (6,7,8,9): sample(seq,t,2700,5)
        self.assertNotEqual(seq.phase, 'COMPLETE')
        for t in (10,11,12,13): sample(seq,t,2700,100)
        self.assertNotEqual(seq.phase, 'COMPLETE')

    def test_timeout_abort_and_disconnect(self):
        seq=EngineStepSequence(settings()); seq.start(0); baseline(seq,1000,5)
        for t in range(6,36): sample(seq,t,1000,100)
        self.assertEqual(seq.phase, 'NOT STABILIZED')
        seq=EngineStepSequence(settings()); seq.start(0); seq.abort(1)
        self.assertIsNone(sample(seq,5,1000,5)); self.assertEqual(seq.phase, 'ABORTED')
        seq=EngineStepSequence(settings()); seq.start(0); sample(seq,1,1000,5,connected=False)
        self.assertEqual(seq.phase, 'FAILED')

    def test_stale_duplicate_missing_and_wrong_start_throttle_fail(self):
        for kwargs in (dict(now=2,receipt_time=0,sequence=1,rpm=1000,throttle=5),
                       dict(now=1,receipt_time=1,sequence=1,rpm=float('nan'),throttle=5),
                       dict(now=1,receipt_time=1,sequence=1,rpm=1000,throttle=50)):
            seq=EngineStepSequence(settings()); seq.start(0); seq.sample(**kwargs)
            if kwargs['receipt_time'] == 0: seq.tick(3)
            self.assertEqual(seq.phase, 'FAILED')
        seq=EngineStepSequence(settings()); seq.start(0); sample(seq,1,1000,5); sample(seq,1,1000,5)
        self.assertEqual(seq.phase, 'FAILED')
