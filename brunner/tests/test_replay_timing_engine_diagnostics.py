import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from replay_support import estimate_replay_lead,replay_feedforward,validate_reference_power

class TimingAndEngineTests(unittest.TestCase):
    def test_measured_latency_compensation_improves_a_delayed_motor(self):
        t=np.arange(0.,30.,.02)
        reference=.2*np.sin(t*1.7)+.1*np.sin(t*.4)
        old_lead=.08
        latency=.4
        measured=np.interp(t-latency+old_lead,t,reference)
        lead=estimate_replay_lead(t,reference,t,measured,old_lead)
        self.assertAlmostEqual(lead,latency,delta=.02)
        improved=np.interp(t-latency+lead,t,reference)
        keep=t>1.
        self.assertLess(np.mean((improved[keep]-reference[keep])**2),np.mean((measured[keep]-reference[keep])**2)*.1)

    def test_constant_axis_does_not_invent_a_delay(self):
        t=np.arange(0.,10.,.02)
        self.assertEqual(estimate_replay_lead(t,t*0,t,t*0,.08),.08)

    def test_lookahead_preserves_start_and_axis_specific_timing(self):
        self.assertEqual(replay_feedforward([0.,1.],([.2,.8],[0.,1.],[0.,0.]),0.,(.3,.5,.08)),(.2,0.,0.))
        p,r,y=replay_feedforward([0.,2.],([0.,2.],[0.,2.],[0.,0.]),1.,(.3,.5,.08))
        self.assertAlmostEqual(p,1.3)
        self.assertAlmostEqual(r,1.5)
        self.assertEqual(y,0.)

    def test_recorded_engine_levers_are_verified_without_modifying_them(self):
        reference={'telemetry':{'engine_mixture_percent':63.,'engine_propeller_percent':100.}}
        actual={'engine_mixture_percent':56.,'engine_propeller_percent':100.}
        with self.assertRaisesRegex(ValueError,'Mixture'):
            validate_reference_power(actual,reference)
        self.assertEqual(actual['engine_mixture_percent'],56.)
        actual['engine_mixture_percent']=63.
        validate_reference_power(actual,reference)

if __name__=='__main__':unittest.main()
